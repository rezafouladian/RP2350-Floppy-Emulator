//! Minimal Macintosh floppy drive emulator for the Waveshare RP2350-PiZero.
//!
//! Serves a disk image from the SD card (see `image.rs`): raw 400K/800K GCR or
//! 1.44 MB MFM, or a MOOF of any of those. A KY-040 rotary encoder chooses, browsing folders
//! (see `knob.rs` and [`Menu`]); `settings.txt` in the root can have one go in at power-on (see
//! `config.rs`). The drive is always a SuperDrive, holding DD (GCR) or HD (1.44 MB) media. Cylinders
//! are read from the card as the head moves; writes are decoded and saved back to the card.
//! With `mode = hd20` in the settings file it is a Hard Disk 20 instead (see `hd20.rs`), with
//! `mode = apple2` an Apple II's Disk II 5.25" drive (see `a2.rs`, `woz.rs`, `drive::run_a2`), and
//! with `mode = apple35` an Apple 3.5 Drive for a IIgs or IIc+ (`drive::run_a35`; the floppy
//! code otherwise, with Apple II images), and with `mode = apple2dual` both at once
//! (`drive::run_dual`): two images, one cylinder built at a time, for the drive being read. With
//! `mode = smartport` it is SmartPort units for an Apple II (see `smartport.rs`, `sp.rs`).
//! Status and debug info go to the SSD1322 OLED.
//!
//! Waveshare RP2350-PiZero wiring (from the board schematic, SD card in SPI mode on SPI1):
//!   SD_SCK=GP30, SD_MOSI=GP31, SD_MISO=GP40, SD_CS=GP43
//! Display (SPI0): SCK=GP18, MOSI=GP19, CS=GP17, DC=GP20, RST=GP21.
//! Floppy interface (GPIO header): see the `PIN_*` constants in `drive.rs`.
//! Knob (GPIO header): see the `PIN_*` constants in `knob.rs`.
#![no_std]
#![no_main]

mod a2;
mod a2r;
mod clusters;
mod config;
mod cylinder;
mod dc42;
mod dcd;
mod drive;
mod gcr;
mod hd20;
mod image;
mod knob;
mod mfm;
mod moof;
mod smartport;
mod sp;
#[allow(dead_code)] // library-style API; not every method is used by main
mod ssd1322;
mod track;
mod weak;
mod woz;
mod writes;

use core::cell::UnsafeCell;
use core::fmt::Write;
use core::sync::atomic::Ordering::{Acquire, Release, Relaxed};
use core::sync::atomic::{AtomicI32, AtomicU32};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::exception;

use defmt::info;
use defmt_rtt as _;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::mono_font::ascii::FONT_5X8;
use embedded_graphics::pixelcolor::Gray4;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use embedded_hal::spi::{MODE_0, MODE_3};
use embedded_hal_bus::spi::ExclusiveDevice;
use config::{Autoload, Mode};
use cylinder::Cylinder;
use image::{Candidate, Catalog, Dir, Filter, Name};
use embedded_sdmmc::{Block, SdCard};
use track::{BLOCK, MAX_CYLINDER_BLOCKS, MAX_TRACK_WORDS, TrackBuf};
use panic_probe as _;
use rp235x_hal::clocks::init_clocks_and_plls;
use rp235x_hal::fugit::RateExtU32;
use rp235x_hal::gpio::{FunctionPio0, FunctionPio2, FunctionSpi};
use rp235x_hal::multicore::{Multicore, Stack};
use rp235x_hal::timer::Timer;
use rp235x_hal::{self as hal, entry};
use rp235x_hal::{Clock, pac};
use ssd1322::Display;

/// Text rows that fit on the 64 px tall display with the 5x8 font.
const ROWS: usize = 8;
const LINE_H: i32 = 8;
/// Framebuffer rows sent to the OLED a main-loop pass.
const PRESENT_ROWS: usize = 8;

const DISPLAY_EVERY_US: u64 = 250_000;
/// How often the playing tracks' weak bits and the blank-track noise get new random values:
/// a few times per revolution (100–200 ms), so each pass reads differently.
const WEAK_EVERY_US: u64 = 50_000;
/// The menu goes back to the status screen after this long without the knob being used.
const MENU_TIMEOUT_US: u64 = 15_000_000;
/// Menu item that opens the settings page, after Eject (item 0).
const MENU_SETTINGS: usize = 1;
/// Fixed items before the catalog's entries.
const MENU_FIXED: usize = 2;
/// Dual mode: one more, Eject for the 5.25" drive (item 0 is the 3.5" drive's).
const MENU_EJECT_B: usize = 2;
/// Items of the settings page (see [`settings_press`]).
const SET_BACK: usize = 0;
const SET_MODE: usize = 1;
const SET_AUTOLOAD: usize = 2;
/// The seek setting ([`drive::KEEP_ROTATION`]).
const SET_ROTATION: usize = 3;
/// Whether the images chosen are put back at power-on ([`remember`]).
const SET_REMEMBER: usize = 4;
const SET_RESTART: usize = 5;
const SET_ITEMS: usize = 6;
/// Menu items moved per clockwise detent. Make it −1 if the knob turns the wrong way.
const KNOB_DIRECTION: i64 = 1;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

static CORE1_STACK: Stack<1024> = Stack::new();

/// A static only core 0 touches (and the DMA reads), handed out once via raw pointer.
struct Ram<T>(UnsafeCell<T>);

// Safety: accessed only from core 0 (each from either its main loop or its SysTick handler, not
// both); the DMA only reads the buffer set core 1 is playing, which core 0 leaves alone (see
// `drive::PLAYING`).
unsafe impl<T> Sync for Ram<T> {}

/// Where each image block is on the card, filled in by `image::load`.
static BLOCK_MAP: Ram<image::BlockMap> = Ram(UnsafeCell::new(image::BlockMap::new()));
/// The same for the second drive's image (dual mode: the 5.25" drive).
static BLOCK_MAP_B: Ram<image::BlockMap> = Ram(UnsafeCell::new(image::BlockMap::new()));
/// The current cylinder's blocks (see `cylinder.rs` for the layout).
static CYLINDER: Ram<[Block; MAX_CYLINDER_BLOCKS]> =
    Ram(UnsafeCell::new([const { Block { contents: [0; BLOCK] } }; MAX_CYLINDER_BLOCKS]));
/// Track buffers, `[set][side]`; core 1 plays one set while core 0 fills the other.
static TRACK_BUFS: Ram<[[TrackBuf; 2]; 2]> = Ram(UnsafeCell::new([[[0; MAX_TRACK_WORDS]; 2]; 2]));
/// Random cells for blank MOOF track sides: what a real drive reads from unformatted media.
/// Core 0 rewrites it every `WEAK_EVERY_US` while it plays.
static NOISE: Ram<[u32; NOISE_WORDS]> = Ram(UnsafeCell::new([0; NOISE_WORDS]));
/// The weak bits of each track buffer, `[set][side]` like `TRACK_BUFS` (see `weak.rs`).
static WEAK: Ram<[[weak::WeakList; 2]; 2]> = Ram(UnsafeCell::new([const { [const { weak::WeakList::new() }; 2] }; 2]));
const NOISE_WORDS: usize = 2_048;
/// Knob decoder state; only the SysTick handler touches it.
static KNOB: Ram<knob::Knob> = Ram(UnsafeCell::new(knob::Knob::new()));
/// Detents turned and presses counted by the SysTick handler, taken by the main loop.
static KNOB_TURNS: AtomicI32 = AtomicI32::new(0);
static KNOB_PRESSES: AtomicU32 = AtomicU32::new(0);

/// Fixed-size text for `write!`, up to `N` bytes; overlong text is truncated, non-ASCII becomes
/// `?`.
#[derive(Clone)]
pub struct FixedStr<const N: usize> {
    buf: [u8; N],
    len: usize,
}

/// A line of display text or a message.
pub type Line = FixedStr<128>;

impl<const N: usize> FixedStr<N> {
    pub fn new() -> Self {
        Self { buf: [0; N], len: 0 }
    }

    pub const CAPACITY: usize = N;

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn truncate(&mut self, len: usize) {
        self.len = self.len.min(len);
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("?")
    }
}

impl<const N: usize> From<&str> for FixedStr<N> {
    fn from(s: &str) -> Self {
        let mut l = Self::new();
        let _ = l.write_str(s);
        l
    }
}

impl<const N: usize> Write for FixedStr<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = if b.is_ascii() { b } else { b'?' };
                self.len += 1;
            }
        }
        Ok(())
    }
}

/// Name of a drive command, indexed by `CA1 CA0 SEL CA2` (see `drive::LAST_CMD`).
fn command_name(cmd: u8) -> &'static str {
    match cmd {
        0b0000 => "DIR IN",
        0b0001 => "DIR OUT",
        0b0011 => "RESET EJECT LATCH",
        0b0100 => "STEP",
        0b0110 => "MFM MODE",
        0b0111 => "GCR MODE",
        0b1000 => "MOTOR ON",
        0b1001 => "MOTOR OFF",
        0b1101 => "EJECT",
        0xFF => "-",
        _ => "?",
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut sio = hal::Sio::new(pac.SIO);

    // External high-speed crystal on the board is 12Mhz
    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let sys_hz = clocks.system_clock.freq().to_Hz();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_hz);
    let timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // ---- Display (SPI0) ----
    let sck = pins.gpio18.into_function::<FunctionSpi>();
    let mosi = pins.gpio19.into_function::<FunctionSpi>();
    let mut spi0 = hal::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        MODE_3,
    );
    // Dummy byte to force SCK idle high before display init
    let _ = embedded_hal::spi::SpiBus::write(&mut spi0, &[0]);

    let cs = pins.gpio17.into_push_pull_output();
    let dc = pins.gpio20.into_push_pull_output();
    let rst = pins.gpio21.into_push_pull_output();
    let mut display = Display::new(spi0, cs, dc, rst, &mut delay).unwrap();

    // ---- Floppy interface (GPIO header) ----
    // The Mac drives these; the pull-ups only keep them defined while it is unplugged
    // (/ENBL high = drive not selected).
    let _sel = pins.gpio2.into_pull_up_input();
    let _ca0 = pins.gpio3.into_pull_up_input();
    let _ca1 = pins.gpio4.into_pull_up_input();
    let _ca2 = pins.gpio5.into_pull_up_input();
    let _lstrb = pins.gpio8.into_pull_up_input();
    let _enbl = pins.gpio9.into_pull_up_input();
    let _wrreq = pins.gpio10.into_pull_up_input();
    let _wr = pins.gpio11.into_pull_up_input();
    // RD pulled up, so it reads high while no drive drives it. RD is shared by every drive on the
    // controller, and an LC III's /ENBL2 goes nowhere: with the pad's reset pull-down it read
    // all of drive 2's registers as 0 (an 800K drive, installed, with a disk in), and floating
    // it read a disk there minutes later. Either way the Mac asked to initialize a disk that
    // wasn't there.
    let _rd = pins.gpio12.into_pull_up_disabled().into_function::<FunctionPio0>();
    // The PIO state machines are set up once the settings say for what (see below).
    let (pio0, pio1) = (pac.PIO0, pac.PIO1);

    // ---- Knob (GPIO header) ----
    // The KY-040 has its own pull-ups; ours just keep the pins high if it is unplugged.
    let _knob_clk = pins.gpio22.into_pull_up_input();
    let _knob_dt = pins.gpio23.into_pull_up_input();
    let _knob_sw = pins.gpio24.into_pull_up_input();
    const { assert!(knob::PIN_CLK == 22 && knob::PIN_DT == 23 && knob::PIN_SW == 24) };
    // Sample it every millisecond from SysTick, so turns aren't missed during SD reads.
    let mut syst = delay.free();
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(sys_hz / 1000 - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();

    // Core 1 answers the Mac once the settings say what as: a floppy drive or an HD20.
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let core1 = &mut mc.cores()[1];
    let stack = CORE1_STACK.take().unwrap();

    show(&mut display, &["RP2350 Floppy Emulator", "Reading SD card..."], None);

    // ---- SD card (SPI1) ----
    let sd_sck = pins.gpio30.into_function::<FunctionSpi>();
    let sd_mosi = pins.gpio31.into_function::<FunctionSpi>();
    let sd_miso = pins.gpio40.into_function::<FunctionSpi>();
    let sd_cs = pins.gpio43.into_push_pull_output();
    // The card must be initialized at <= 400 kHz; speed is raised afterward.
    let spi1 = hal::Spi::<_, _, _, 8>::new(pac.SPI1, (sd_mosi, sd_miso, sd_sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        400.kHz(),
        MODE_0,
    );
    let sd_dev = ExclusiveDevice::new(spi1, sd_cs, timer).unwrap();
    let sdcard = SdCard::new(sd_dev, timer);

    // Safety: the only reference to BLOCK_MAP, and nothing else runs on core 0.
    let map = unsafe { &mut *BLOCK_MAP.0.get() };
    // First access initializes the card.
    let catalog = match sdcard.num_bytes() {
        Ok(_) => {
            // Card is up; run the bus faster for reads.
            sdcard.spi(|dev| {
                dev.bus_mut()
                    .set_baudrate(clocks.peripheral_clock.freq(), 16.MHz());
            });
            image::scan(&sdcard, None, Filter::Floppy)
        }
        Err(e) => {
            let mut l = Line::new();
            let _ = write!(l, "{:?}", e);
            Err(l)
        }
    };
    // The directory the menu shows; the knob moves it around the card.
    let mut catalog = match catalog {
        Ok(c) => c,
        Err(e) => {
            info!("SD error: {}", e.as_str());
            // Still a drive, with no disk.
            let _rd = _rd.into_function::<FunctionPio2>();
            let _rd_stream = pins.gpio27.into_function::<FunctionPio0>();
            let stream = drive::Stream::new(pio0, &mut pac.RESETS, sys_hz, drive::Pulses::MacMux);
            let mux = drive::Mux::new(pac.PIO2, &mut pac.RESETS, false);
            let writer = drive::Writer::new(pio1, &mut pac.RESETS, sys_hz);
            let (player, port, capture) = (stream.player, mux.port, writer.capture);
            core1.spawn(stack, move || drive::run(player, port, capture)).unwrap();
            show(&mut display, &["RP2350 Floppy Emulator", "SD card error:", e.as_str()], None);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    info!("{} entries in the root", catalog.len());

    // ---- Settings ----
    // The file's text, which the menu's settings page changes and saves.
    let mut settings_file = config::Text::new();
    // Why the settings file wasn't (all) understood, shown in the menu once the first image is in.
    let mut settings_note: Option<Line> = None;
    match image::read_settings(&sdcard, &catalog, &mut settings_file) {
        Ok(true) => {}
        Ok(false) => {
            // None yet: write one with the defaults and what they mean, to edit on a computer.
            settings_file.defaults();
            if let Err(e) = image::write_settings(&sdcard, catalog.dir(), settings_file.as_bytes()) {
                let mut l = Line::new();
                let _ = write!(l, "Can't create {}: {}", config::FILE_NAME, e.as_str());
                settings_note = Some(l);
            }
        }
        Err(e) => {
            // Saving from the menu would lose what couldn't be read.
            settings_file.partial = true;
            let mut l = Line::new();
            let _ = write!(l, "{}: {}", config::FILE_NAME, e.as_str());
            settings_note = Some(l);
        }
    }
    let settings = settings_file.config();
    if let Some(n) = settings.bad_line {
        let mut l = Line::new();
        let _ = write!(l, "{} line {}: not understood", config::FILE_NAME, n);
        settings_note = Some(l);
    }
    let mode = settings.mode;
    // The seek setting. KEEP_ROTATION is this, or on for a synchronized MOOF (`synced`).
    let mut keep_rotation = settings.keep_rotation;
    // What goes in at power-on, apart from the text so that the menu can change it.
    let autoload_name = Name::from(settings.autoload.value());
    let autoload = match settings.autoload {
        Autoload::Off => Autoload::Off,
        Autoload::First => Autoload::First,
        Autoload::Named(_) => Autoload::Named(autoload_name.as_str()),
    };

    // A Disk II: quarter tracks, no register file (`apple2`); or an Apple 3.5 Drive, which is the
    // Mac's drive with the Apple II's images (`apple35`).
    let apple2 = mode == Mode::Apple2;
    let apple35 = mode == Mode::Apple35;
    // Both: drive 0 the Apple 3.5 Drive, drive 1 the Disk II.
    let dual = mode == Mode::Dual;
    // An HD20 with a floppy drive behind it: drive 0 the floppy (the SuperDrive), drive 1 the HD20.
    let chain = mode == Mode::Hd20Floppy;
    // WRPROT: core 1 drives it through the pad overrides, high-impedance while the drive isn't
    // selected. In the Mac modes the pin is left alone: it's the Mac's PWM output.
    const { assert!(drive::PIN_WRPROT == 13 && drive::PIN_EN35 == 14 && drive::PIN_DRIVE2 == 15) };
    let wrprot_hiz = || unsafe { (*pac::IO_BANK0::ptr()).gpio(drive::PIN_WRPROT as usize).gpio_ctrl().modify(|_, w| w.oeover().disable()) };
    match mode {
        Mode::Apple2 => {
            let _wrprot = pins.gpio13.into_push_pull_output();
            wrprot_hiz();
            // /3.5DISK pulled down: unwired (or ground, on a IIe's 20-pin cable) the Disk II
            // answers whenever /ENBL selects it; on a IIgs it's high but for its 3.5" drives.
            let _en35 = pins.gpio14.into_pull_down_input();
        }
        Mode::SmartPort => {
            // ACK: core 1 drives it low, or leaves it high-impedance (the host pulls it up).
            let _ack = pins.gpio13.into_pull_up_input().into_push_pull_output();
            wrprot_hiz();
            // /3.5DISK pulled up: unwired, the bus is never taken for the 3.5" drives'.
            let _en35 = pins.gpio14.into_pull_up_input();
        }
        Mode::Apple35 => {
            // It carries what RD does: PIO2's register mux and track stream (`drive::Mux`).
            let _wrprot = pins.gpio13.into_function::<FunctionPio2>();
            wrprot_hiz();
            // /3.5DISK pulled down: unwired, the drive answers whenever /ENBL selects it.
            // /DRIVE2 pulled up: unwired, it never deselects it.
            let _en35 = pins.gpio14.into_pull_down_input();
            let _drive2 = pins.gpio15.into_pull_up_input();
        }
        Mode::Dual => {
            // It carries what RD does: the register's level, and PIO0's pulses (for a Disk II,
            // its write protect, overriding them).
            let _wrprot = pins.gpio13.into_function::<FunctionPio0>();
            wrprot_hiz();
            let _en35 = pins.gpio14.into_pull_down_input();
            let _drive2 = pins.gpio15.into_pull_up_input();
        }
        _ => {}
    }
    // Floppy and Apple II 3.5" modes: PIO2 answers on RD (and WRPROT), and the track plays on a
    // pin of its own for it (`drive::Mux`).
    let muxed = matches!(mode, Mode::Floppy | Mode::Apple35);
    if muxed {
        let _rd = _rd.into_function::<FunctionPio2>();
        let _rd_stream = pins.gpio27.into_function::<FunctionPio0>();
    }
    let pulses = match mode {
        Mode::Apple2 => drive::Pulses::DiskIIExact,
        Mode::SmartPort => drive::Pulses::DiskII,
        Mode::Apple35 => drive::Pulses::Apple35,
        Mode::Dual => drive::Pulses::Dual,
        Mode::Floppy => drive::Pulses::MacMux,
        Mode::Hd20 | Mode::Hd20Floppy => drive::Pulses::Mac,
    };
    // These keep the PIO state machines alive; core 1 drives them from here on.
    let stream = drive::Stream::new(pio0, &mut pac.RESETS, sys_hz, pulses);
    let mux = muxed.then(|| drive::Mux::new(pac.PIO2, &mut pac.RESETS, mode == Mode::Apple35));
    let port = mux.as_ref().map(|m| m.port);
    let writer = drive::Writer::new(pio1, &mut pac.RESETS, sys_hz);
    let (player, capture) = (stream.player, writer.capture);
    let title = match mode {
        Mode::Apple2 => "Apple II floppy",
        Mode::Apple35 => "Apple II 3.5\"",
        Mode::Dual => "Apple II 5.25\" + 3.5\"",
        Mode::SmartPort => "Apple II SmartPort",
        Mode::Hd20Floppy => "Mac HD20 + floppy",
        _ => "Mac floppy",
    };
    let filter = match mode {
        Mode::Apple2 => Filter::Apple2,
        Mode::Apple35 => Filter::Apple35,
        Mode::Dual => Filter::Dual,
        Mode::Hd20Floppy => Filter::Hd20Floppy,
        _ => Filter::Floppy,
    };
    // Who the drive answers, for the status screen.
    let host = if apple2 || apple35 || dual { "Apple" } else { "Mac" };

    if mode == Mode::Hd20 {
        // Commands come in through the GCR separator; replies go out through PIO0 SM0.
        writer.start(false);
        let port = stream.dcd_port();
        core1.spawn(stack, move || drive::run_dcd(port, capture)).unwrap();
        // It lists the root itself, so the catalog isn't in this frame too.
        hd20::run(&sdcard, autoload, settings_note, &mut settings_file, &mut display, || timer.get_counter().ticks());
    }
    if mode == Mode::SmartPort {
        // Packets at the Apple II's 4 µs cells both ways: replies through PIO0 SM0 a byte at a
        // time, the host's through the GCR separator, pushed 8 cells at a time so core 1 sees a
        // packet's end at once.
        stream.set_cell_rate(sys_hz, drive::A2_PIO_HZ);
        writer.set_gcr_rate(sys_hz, drive::A2_CAPTURE_HZ);
        writer.set_push_bits(8);
        writer.start(false);
        let port = stream.dcd_port();
        core1.spawn(stack, move || drive::run_smartport(port, capture)).unwrap();
        sp::run(&sdcard, autoload, settings_note, &mut settings_file, &mut display, || timer.get_counter().ticks());
    }
    if apple2 || apple35 || dual || chain {
        // The root again, for Apple II images (or hard disks too).
        catalog = match image::scan(&sdcard, None, filter) {
            Ok(c) => c,
            Err(e) => {
                show(&mut display, &[title, "SD card error:", e.as_str()], None);
                loop {
                    cortex_m::asm::wfi();
                }
            }
        };
    }
    if dual {
        // Each drive's cells and writes at its own rate, which core 1 switches between.
        drive::CLKDIV[0].store(drive::clkdiv_word(sys_hz, drive::GCR_PIO_HZ), Relaxed);
        drive::CLKDIV[1].store(drive::clkdiv_word(sys_hz, drive::A2_PIO_HZ), Relaxed);
        drive::CAP_CLKDIV[0].store(drive::clkdiv_word(sys_hz, drive::A35_CAPTURE_HZ), Relaxed);
        drive::CAP_CLKDIV[1].store(drive::clkdiv_word(sys_hz, drive::A2_CAPTURE_HZ), Relaxed);
        drive::NOISE_PTR.store(NOISE.0.get() as u32, Relaxed);
        drive::NOISE_LEN.store(NOISE_WORDS as u32, Relaxed);
        // One separator for both, started once: a load into one drive mustn't cut off a write to
        // the other.
        writer.start(false);
        // Core 1 is both drives from now on: present, no disks yet.
        core1.spawn(stack, move || drive::run_dual(player, capture)).unwrap();
    } else if chain {
        // The floppy's cells are GCR's until an image says otherwise (`CLKDIV[0]`, which core 1
        // applies: SM0 may be sending an HD20 reply), the HD20's always (`CLKDIV[1]`). The
        // capture DMA starts on the GCR separator; core 1 points it at the MFM one for an MFM
        // floppy's writes.
        drive::CLKDIV[0].store(drive::clkdiv_word(sys_hz, drive::GCR_PIO_HZ), Relaxed);
        drive::CLKDIV[1].store(drive::clkdiv_word(sys_hz, drive::GCR_PIO_HZ), Relaxed);
        writer.start(false);
        let port = stream.dcd_port();
        // Core 1 is both from now on: the HD20 with nothing to serve yet, and the floppy, empty.
        core1.spawn(stack, move || drive::run_chain(player, port, capture)).unwrap();
    } else if apple35 {
        // The IIgs writes at its own cell, 4% short of the Mac's.
        writer.set_gcr_rate(sys_hz, drive::A35_CAPTURE_HZ);
        // Core 1 is the Apple 3.5 Drive from now on: present, no disk yet.
        let port = port.unwrap();
        core1.spawn(stack, move || drive::run_a35(player, port, capture)).unwrap();
    } else if apple2 {
        writer.set_gcr_rate(sys_hz, drive::A2_CAPTURE_HZ);
        drive::NOISE_PTR.store(NOISE.0.get() as u32, Relaxed);
        drive::NOISE_LEN.store(NOISE_WORDS as u32, Relaxed);
        // Core 1 is the Apple II's drive from now on: present, no disk yet.
        core1.spawn(stack, move || drive::run_a2(player, capture)).unwrap();
    } else {
        // Core 1 answers the Mac from now on: drive present, no disk yet.
        let port = port.unwrap();
        core1.spawn(stack, move || drive::run(player, port, capture)).unwrap();
    }

    // ---- Main loop: keep core 1 supplied with the head's cylinder, show status ----
    // Safety: only core 0 touches these; core 1 and the DMA read only the set in PLAYING.
    let cylinder = unsafe { &mut *CYLINDER.0.get() };
    let bufs = unsafe { &mut *TRACK_BUFS.0.get() };
    let noise = unsafe { &mut *NOISE.0.get() };
    let weak = unsafe { &mut *WEAK.0.get() };
    let mut rng = weak::Rng::new(0x2545_F491);
    for w in noise.iter_mut() {
        *w = rng.noise();
    }
    let noise_buf = (noise.as_ptr() as u32, noise.len() as u32);
    let mut next_weak = 0u64;
    #[cfg(feature = "diag")]
    let mut diag_log = DiagLog::new();
    let mut published = 0u32;
    // The buffers last built, for publishing again at another quarter track that hears the same
    // (Apple II).
    let mut last_sides = [(0u32, 0u32, 0u32); 2];
    let mut last_geometry = (0u32, 0u32);
    let mut last_rate = (0u32, 0u32);
    let mut sd_us = 0u64;
    let mut build_us = 0u64;
    let mut sd_errors = 0u32;
    let mut writes_ok = 0u32;
    let mut write_errors = 0u32;
    let mut last_write_error = "";
    let mut flush_us = 0u64;
    let mut seen_ejects = 0u32;
    let mut next_display = 0u64;
    // The next framebuffer row to send to the OLED, while a screen is going out.
    let mut presenting: Option<usize> = None;
    let mut last_loops = (0u32, 0u64);
    let mut loop_ns = 0u64;
    let fixed = if dual { MENU_FIXED + 1 } else { MENU_FIXED };
    let mut menu = Menu { open: false, settings: false, cursor: 0, top: 0, used_at: 0, note: None, fixed };
    // The image to load next and its drive: the knob's choice (its directory and entry). At
    // power-on, a remembered image (below), or none and `autoload` says what goes in.
    let mut choice: Option<(usize, Dir, Candidate)>;
    // Per drive: one, or in dual mode drive 0 (the 3.5" one) and drive 1 (the 5.25"). The image
    // in it, where it's from, how it loaded (for the display), and whether it waits to go in.
    let drives = if dual { 2 } else { 1 };
    // Raw, dereferenced where used: in each mode a map is one drive's (dual mode's 5.25" drive and
    // HD20 + floppy mode's HD20 have the second), so a drive's cylinder holding one never meets a
    // load into it (the outer loop drops the cylinder before it loads; an HD20 load only touches
    // the second, which no floppy cylinder uses in that mode).
    let maps: [*mut image::BlockMap; 2] = [map, BLOCK_MAP_B.0.get()];
    // HD20 + floppy mode's HD20: its image and what serves it.
    let mut hd: Option<image::HdLoaded> = None;
    let mut server = hd20::Server::new();
    let mut images: [Option<image::Loaded>; 2] = [None, None];
    let mut current: [Option<(Dir, Name)>; 2] = [None, None];
    let mut sources = [Line::from("No disk"), Line::from("No disk")];
    let mut insert = [false; 2];
    // Each image's track rotations found so far (`Cylinder::rotations`): dual mode takes a new
    // Cylinder at every switch of drives, and finding a WOZ's all takes ~0.4 s.
    let mut rotations: [Option<[u32; moof::TRK_COUNT]>; 2] = [None, None];
    // A drive the knob emptied (Eject): what `remember` notes for it, though its image stays loaded.
    let mut knob_out = [false; 2];

    // Power-on with `remember` on: what the knob last put in each drive goes back, in place of
    // autoload. The first goes in as if chosen; dual mode's other then follows as a swap would.
    choice = None;
    let mut then: Option<(usize, Dir, Candidate)> = None;
    let mut remembering = false;
    for d in 0..drives {
        match remembered(&sdcard, &settings_file, d, filter, &mut catalog) {
            None => {}
            Some(Ok(image)) => {
                remembering = true;
                let image = image.map(|(dir, file)| (d, dir, file));
                if choice.is_none() { choice = image } else { then = image }
            }
            Some(Err(e)) => {
                remembering = true;
                settings_note = Some(e);
            }
        }
    }
    if chain {
        // The HD20's at power-on: what `remember` noted for it, else autoload's if it names a hard
        // disk image (autoload then leaves the floppy empty).
        let restored = remembered(&sdcard, &settings_file, 1, filter, &mut catalog);
        remembering |= restored.is_some();
        // A hard disk in `last` (as HD20 mode notes its image) is the HD20's here.
        // It's no floppy either way.
        let from_last = choice.take_if(|(_, _, file)| file.is_hard_disk());
        let restored = match (restored, from_last) {
            (None, Some((_, dir, file))) => Some(Ok(Some((dir, file)))),
            (restored, _) => restored,
        };
        let pick = match (restored, autoload) {
            (Some(Ok(image)), _) => image,
            (Some(Err(e)), _) => {
                settings_note = Some(e);
                None
            }
            (None, Autoload::Named(name)) if !remembering => (0..catalog.len())
                .filter_map(|i| catalog.get(i))
                .find(|c| c.is_hard_disk() && c.name.as_str().eq_ignore_ascii_case(name))
                .map(|c| (catalog.dir().clone(), c.clone())),
            (None, _) => None,
        };
        if let Some((dir, file)) = pick {
            remembering |= matches!(autoload, Autoload::Named(_));
            match load_chain_hd(&sdcard, &dir, &file, unsafe { &mut *maps[1] }, &mut server, &mut hd, &mut current[1], &mut sources[1], &mut display) {
                // Noted as the HD20's (`last2`), also when it came from `last`.
                Ok(()) => {
                    if let Some(note) = remember(&sdcard, &mut settings_file, 1, Some((&dir, file.name.as_str())), &dir) {
                        settings_note = Some(note);
                    }
                }
                Err(e) => {
                    let mut l = Line::new();
                    let _ = write!(l, "HD20: can't load: {}", e.as_str());
                    settings_note = Some(l);
                }
            }
        }
    }
    let autoload = if remembering { Autoload::Off } else { autoload };

    loop {
        // ---- Load the chosen image ----
        let t0 = timer.get_counter().ticks();
        // What was to be loaded, and into which drive. None: nothing was.
        let mut loaded: Option<(usize, Result<image::Loaded, Line>)> = None;
        // The last image tried, to point the menu at it if it didn't load.
        let mut tried: Option<(Dir, Name)> = None;
        let mut load = |d: usize, dir: &Dir, file: &Candidate| {
            tried = Some((dir.clone(), file.name.clone()));
            show(&mut display, &[title, "Loading:", dir.path(), file.name.as_str()], None);
            // Set R15 (HD media) as soon as the image type is known.
            image::load(&sdcard, dir, file, unsafe { &mut *maps[d] }, |hd| drive::HD_MEDIA.store(hd && d == 0, Relaxed))
        };
        match (choice.take(), autoload) {
            (Some((d, dir, file)), _) => loaded = Some((d, load(d, &dir, &file))),
            (None, Autoload::Off) => {}
            (None, Autoload::First) => {
                loaded = Some((0, Err(Line::from("no disk image in root"))));
                for file in (0..catalog.len()).filter_map(|i| catalog.get(i)).filter(|c| c.is_image()) {
                    // HD20 + floppy mode: the first floppy image (the HD20's goes in by name).
                    let Ok(d) = drive_of(dual, chain, &sdcard, catalog.dir(), file) else { continue };
                    if chain && d == 1 {
                        continue;
                    }
                    let result = load(d, catalog.dir(), file);
                    let ok = result.is_ok();
                    loaded = Some((d, result));
                    if ok {
                        break;
                    }
                }
            }
            (None, Autoload::Named(name)) => {
                let file = (0..catalog.len())
                    .filter_map(|i| catalog.get(i))
                    .find(|c| c.is_image() && c.name.as_str().eq_ignore_ascii_case(name));
                loaded = Some(match file.map(|file| (file, drive_of(dual, chain, &sdcard, catalog.dir(), file))) {
                    Some((file, Ok(d))) => (d, load(d, catalog.dir(), file)),
                    Some((_, Err(e))) => (0, Err(e)),
                    None => {
                        let mut l = Line::new();
                        let _ = write!(l, "{} isn't in the root", name);
                        (0, Err(l))
                    }
                });
            }
        }
        let load_ms = (timer.get_counter().ticks() - t0) / 1000;
        // Dual mode: a disk in the drive its kind said (a `.2mg`'s header could disagree with its
        // size).
        let loaded = loaded.map(|(d, result)| match result {
            Ok(l) if dual && l.disk.is_apple2() != (d == 1) => (d, Err(Line::from("not the disk its name and size say"))),
            other => (d, other),
        });
        match loaded {
            Some((d, Ok(l))) => {
                let disk = &l.disk;
                let mut source = Line::new();
                let _ = write!(
                    source,
                    "{} {} {}frag {}ms {}",
                    disk.describe(),
                    l.filesystem,
                    l.fragments,
                    load_ms,
                    l.note
                );
                if !dual {
                    drive::HD_MEDIA.store(disk.is_hd(), Relaxed);
                    drive::MFM.store(disk.is_mfm(), Relaxed);
                    drive::A2_CELLS_Q32.store(drive::a2_cells_q32(sys_hz, disk.cell_hz()), Relaxed);
                    if chain {
                        // Core 1 sets SM0's rate (it may be sending an HD20 reply) and points the
                        // capture DMA at the right separator.
                        drive::CLKDIV[0].store(drive::clkdiv_word(sys_hz, disk.cell_hz()), Relaxed);
                    } else {
                        stream.set_cell_rate(sys_hz, disk.cell_hz());
                        writer.start(disk.is_mfm());
                    }
                } else if d == 1 {
                    // The 5.25" drive's cells (a WOZ may have its own), which core 1 plays it at
                    // and counts its disk's turning in.
                    drive::CLKDIV[1].store(drive::clkdiv_word(sys_hz, disk.cell_hz()), Relaxed);
                    drive::A2_CELLS_Q32.store(drive::a2_cells_q32(sys_hz, disk.cell_hz()), Relaxed);
                }
                writable_of(d).store(disk.writable(), Relaxed);
                info!("Loaded {} ({})", l.name.as_str(), source.as_str());
                sources[d] = source;
                current[d] = tried.clone();
                knob_out[d] = false;
                if let Some((dir, name)) = &current[d]
                    && let Some(note) = remember(&sdcard, &mut settings_file, d, Some((dir, name.as_str())), dir)
                {
                    settings_note = Some(note);
                }
                images[d] = Some(l);
                rotations[d] = None;
                // Once its first cylinder is ready and the computer knows the drive is empty.
                insert[d] = true;
            }
            Some((d, Err(e))) => {
                // The drive was emptied for it, and its block map is the failed load's now.
                writable_of(d).store(false, Relaxed);
                images[d] = None;
                rotations[d] = None;
                current[d] = None;
                sources[d] = Line::new();
                let _ = write!(sources[d], "No disk: {}", e.as_str());
                let name = tried.as_ref().map_or("", |(_, n)| n.as_str());
                info!("Can't load {}: {}", name, e.as_str());
                // Back to the menu, on the image that failed, with the reason in the title row.
                menu.show(loaded_index(&catalog, &tried), timer.get_counter().ticks());
                let mut note = Line::new();
                let _ = write!(note, "Can't load: {}", e.as_str());
                menu.note = Some(note);
            }
            None => {
                // Power-on without autoload: offer the menu (unless the HD20 has its disk).
                drive::WRITABLE.store(false, Relaxed);
                if hd.is_none() {
                    menu.show(None, timer.get_counter().ticks());
                }
            }
        }
        let synced = images[0].as_ref().is_some_and(|l| l.disk.synchronized());
        drive::KEEP_ROTATION.store(keep_rotation || synced, Relaxed);
        // A problem with the settings file, unless there's already a reason to show.
        if let Some(note) = settings_note.take()
            && menu.note.is_none()
        {
            menu.show(here(&catalog, &current), timer.get_counter().ticks());
            menu.note = Some(note);
        }
        // Knob use while loading doesn't count.
        KNOB_TURNS.store(0, Relaxed);
        KNOB_PRESSES.store(0, Relaxed);
        // The cylinder held, and whose: one drive's at a time, in dual mode the one the computer is
        // reading (see below).
        let mut cyl: Option<Cylinder<_>> = None;
        let mut cyl_drive = 0usize;
        // An image the knob chose while another is loaded, its drive, and when that was emptied.
        let mut swap: Option<(usize, Dir, Candidate, u64)> = then.take().map(|(d, dir, file)| (d, dir, file, timer.get_counter().ticks()));
        // When the drives were emptied for a restart the settings page asked for.
        let mut restart: Option<u64> = None;

        loop {
            // The cylinder to hold: in dual mode, the drive with a write waiting to be applied, or
            // else the one the computer is reading (`drive::STREAM_WANT`); a drive with no disk has
            // nothing to hold. Another drive's is let go once everything of it is saved.
            let tail = drive::CAP_TAIL.load(Relaxed);
            let queued = (tail != drive::CAP_HEAD.load(Acquire))
                .then(|| (drive::CAP_INFO[tail as usize % drive::CAPTURE_QUEUE].load(Relaxed) & drive::CYL_DRIVE2 != 0) as usize);
            let want = match (queued, drive::STREAM_WANT.load(Relaxed)) {
                _ if !dual => 0,
                (Some(d), _) => d,
                (None, 1) => 0,
                (None, 2) => 1,
                _ => cyl_drive,
            };
            let want = if images[want].is_some() { want } else { want ^ (drives - 1) };
            let switching = cyl.is_some() && cyl_drive != want && images[want].is_some();
            let let_go = !drive::WRITING.load(Acquire) && !cyl.as_ref().is_some_and(|c| c.is_dirty() || c.checksum_pending());
            if (cyl.is_none() || (switching && let_go))
                && let Some(l) = images[want].as_ref()
            {
                // Dropping the old one first frees its blocks for the new one.
                if let Some(c) = cyl.take() {
                    rotations[cyl_drive] = Some(*c.rotations());
                }
                // A Disk II's tracks loop at exactly their cells (`src/disk2.pio`), also dual
                // mode's drive B.
                let disk2 = apple2 || want == 1;
                let mut c = Cylinder::new(&sdcard, unsafe { &*maps[want] }, &l.disk, &mut *cylinder, noise_buf, disk2);
                if let Some(r) = &rotations[want] {
                    c.set_rotations(r);
                } else if disk2
                    && let Err(e) = c.prepare()
                {
                    sd_errors += 1;
                    info!("SD read error preparing tracks: {}", defmt::Debug2Format(&e));
                }
                cyl = Some(c);
                cyl_drive = want;
            }
            if chain {
                // The HD20's commands, in turn with the floppy's writes in the queue (a write for a
                // floppy that has gone is dropped), and the replies they're owed.
                let now_us = || timer.get_counter().ticks();
                loop {
                    let tail = drive::CAP_TAIL.load(Relaxed);
                    if tail == drive::CAP_HEAD.load(Acquire) {
                        break;
                    }
                    if drive::CAP_INFO[tail as usize % drive::CAPTURE_QUEUE].load(Relaxed) & drive::CAP_DCD != 0 {
                        server.command(&sdcard, unsafe { &*maps[1] }, hd.as_ref(), &now_us);
                    } else if cyl.is_none() {
                        drive::CAP_TAIL.store(tail.wrapping_add(1), Release);
                        write_errors += 1;
                        last_write_error = "write: to a drive with no disk";
                    } else {
                        break;
                    }
                }
                server.serve(&sdcard, unsafe { &*maps[1] }, hd.as_ref(), &now_us);
            }

            // Its head: on the Disk II a quarter track, where `key` (`Disk::a2_track`) is heard.
            let quarter = apple2 || cyl_drive == 1;
            let tag = if cyl_drive == 1 { drive::CYL_DRIVE2 } else { 0 };

            if let Some(cyl) = cyl.as_mut() {
                // Wait for core 1 to take the last cylinder before reusing the other set.
                let caught_up = drive::PLAYING.load(Acquire) == published;
                let free_set = match drive::cylinder_track(published) {
                    Some(_) => drive::cylinder_set(published) ^ 1,
                    None => 0,
                };

                // Writes first: they belong to the loaded cylinder, even if the head has moved since.
                // (A cylinder just taken on is read first, below.) Another drive's wait for it.
                let tail = drive::CAP_TAIL.load(Relaxed);
                if caught_up && cyl.track().is_some() && tail != drive::CAP_HEAD.load(Acquire) {
                    let mut changed = 0;
                    let mut tail = tail;
                    while tail != drive::CAP_HEAD.load(Acquire) {
                        let k = tail as usize % drive::CAPTURE_QUEUE;
                        let cap = cylinder::Capture {
                            start: drive::CAP_START[k].load(Relaxed),
                            words: drive::CAP_WORDS[k].load(Relaxed),
                            pos: drive::CAP_POS[k].load(Relaxed),
                            info: drive::CAP_INFO[k].load(Relaxed),
                        };
                        // An HD20 command (HD20 + floppy mode) is served above, in its turn.
                        if cap.info & drive::CAP_DCD != 0 {
                            break;
                        }
                        // The other drive's waits for its cylinder, unless its disk has gone.
                        let other = (cap.info & drive::CYL_DRIVE2 != 0) as usize;
                        if other != cyl_drive && images[other].is_some() {
                            break;
                        }
                        // The ring wraps: a capture is only good if nothing has written over it.
                        let seq = drive::CAP_SEQ[k].load(Relaxed);
                        let intact = || drive::CAP_TOTAL.load(Acquire).wrapping_sub(seq) <= drive::RING_WORDS as u32;
                        let result = if other != cyl_drive {
                            Err("write: to a drive with no disk")
                        } else if intact() {
                            cyl.apply(&cap, &mut bufs[free_set][0])
                        } else {
                            Err("write: capture overrun")
                        };
                        match result {
                            Ok(n) if intact() => {
                                writes_ok += n;
                                changed += n;
                            }
                            Ok(_) => {
                                write_errors += 1;
                                last_write_error = "write: capture overrun";
                            }
                            Err(e) => {
                                write_errors += 1;
                                last_write_error = e;
                                info!("{}", e);
                            }
                        }
                        tail = tail.wrapping_add(1);
                        drive::CAP_TAIL.store(tail, Release);
                    }
                    if changed > 0 {
                        // Republish the same cylinder (as the same head position): core 1 keeps
                        // its rotational position.
                        let sides = cyl.build(&mut bufs[free_set], &mut weak[free_set]);
                        let head = drive::cylinder_track(published).unwrap_or(0);
                        let span = if quarter { a2_span(cyl.disk(), head) } else { 0 };
                        publish(&mut published, free_set, head, tag, sides, span, cyl.a2_geometry(), a2_rate(sys_hz, cyl.a2_hz()));
                        last_sides = sides;
                        last_geometry = cyl.a2_geometry();
                        last_rate = a2_rate(sys_hz, cyl.a2_hz());
                    }
                }
                // Saved once the computer has stopped writing (an Apple II track's sectors are
                // only decoded then, `Cylinder::settle`).
                if cyl.is_dirty()
                    && drive::CAP_TAIL.load(Relaxed) == drive::CAP_HEAD.load(Acquire)
                    && !drive::WRITING.load(Acquire)
                {
                    let start = timer.get_counter().ticks();
                    match cyl.flush(start) {
                        Ok(()) => flush_us = timer.get_counter().ticks() - start,
                        Err(e) => {
                            sd_errors += 1;
                            info!("SD write error: {}", defmt::Debug2Format(&e));
                        }
                    }
                }

                // Then follow the head, once everything written has been saved.
                let head = if quarter { drive::HEAD_QUARTER.load(Relaxed) } else { drive::HEAD_TRACK.load(Relaxed) };
                let key = if quarter { cyl.disk().a2_track(head) } else { head };
                let caught_up = drive::PLAYING.load(Acquire) == published;
                // Writes above may have republished, so the free set may have changed.
                let free_set = match drive::cylinder_track(published) {
                    Some(_) => drive::cylinder_set(published) ^ 1,
                    None => 0,
                };
                // Not while a write is still to come in: DOS steps as soon as a sector write
                // ends, inside the capture's tail.
                let written = !cyl.is_dirty()
                    && drive::CAP_TAIL.load(Relaxed) == drive::CAP_HEAD.load(Acquire)
                    && !drive::WRITING.load(Acquire);
                if cyl.track() != Some(key) && caught_up && written {
                    let start = timer.get_counter().ticks();
                    match cyl.read(key) {
                        Ok(()) => {
                            let read = timer.get_counter().ticks();
                            let sides = cyl.build(&mut bufs[free_set], &mut weak[free_set]);
                            let span = if quarter { a2_span(cyl.disk(), head) } else { 0 };
                            publish(&mut published, free_set, head, tag, sides, span, cyl.a2_geometry(), a2_rate(sys_hz, cyl.a2_hz()));
                            last_sides = sides;
                            last_geometry = cyl.a2_geometry();
                            last_rate = a2_rate(sys_hz, cyl.a2_hz());
                            sd_us = read - start;
                            build_us = timer.get_counter().ticks() - read;
                        }
                        Err(e) => {
                            sd_errors += 1;
                            info!("SD read error on track {}: {}", key, defmt::Debug2Format(&e));
                        }
                    }
                } else if quarter && caught_up && published & !(1 << 8) != drive::cylinder(0, head) | tag && cyl.track() == Some(key) {
                    // Another quarter track that hears the same: the same buffers, for it.
                    let set = drive::cylinder_set(published);
                    publish(&mut published, set, head, tag, last_sides, a2_span(cyl.disk(), head), last_geometry, last_rate);
                }

                // DiskCopy: bring the header's data checksum up to date, a few blocks per pass, once
                // everything is saved and the head has stayed put (at once if another image waits).
                if !cyl.is_dirty()
                    && drive::CAP_TAIL.load(Relaxed) == drive::CAP_HEAD.load(Acquire)
                    && !drive::WRITING.load(Acquire)
                    && cyl.track() == Some(drive::HEAD_TRACK.load(Relaxed))
                    && let Err(e) = cyl.checksum_step(timer.get_counter().ticks(), swap.is_some() || restart.is_some() || switching)
                {
                    sd_errors += 1;
                    info!("SD error updating the DiskCopy checksum: {}", defmt::Debug2Format(&e));
                }

                // Weak bits: new random values in the playing set (never the one being built) and
                // the noise buffer.
                let now = timer.get_counter().ticks();
                if now >= next_weak {
                    next_weak = now + WEAK_EVERY_US;
                    let playing = drive::PLAYING.load(Acquire);
                    if playing == published && drive::cylinder_track(playing).is_some() {
                        let set = drive::cylinder_set(playing);
                        for side in 0..2 {
                            weak[set][side].randomize(&mut bufs[set][side], &mut rng);
                        }
                    }
                    for w in noise.iter_mut() {
                        *w = rng.noise();
                    }
                }

            } else {
                // No image: nothing to write to (no gate opens without a disk).
                drive::CAP_TAIL.store(drive::CAP_HEAD.load(Acquire), Release);
            }
            // Insert a disk, if the computer won't mistake it for the last one: once a cylinder of
            // it is playing, or in dual mode at once (the 3.5" drive's /READY covers the rest, and a
            // Disk II reads noise until then).
            let built = dual || cyl.as_ref().is_some_and(|c| c.track().is_some());
            for d in 0..drives {
                let seen_empty = d == 1 || drive::MAC_SAW_EMPTY.load(Relaxed);
                if insert[d] && images[d].is_some() && built && seen_empty {
                    insert[d] = false;
                    #[cfg(feature = "diag")]
                    if d == 0 {
                        drive::diag_reset();
                    }
                    disk_in_of(d).store(true, Relaxed);
                }
            }

            let now = timer.get_counter().ticks();
            // The event log, every few seconds while nothing is being read or written.
            #[cfg(feature = "diag")]
            if !apple2
                && !dual
                && !chain
                && now >= diag_log.next
                && !drive::MOTOR_ON.load(Relaxed)
                && !drive::WRITING.load(Acquire)
                && drive::CAP_TAIL.load(Relaxed) == drive::CAP_HEAD.load(Acquire)
                && !cyl.as_ref().is_some_and(|c| c.is_dirty())
            {
                diag_log.next = now + DiagLog::EVERY_US;
                if let Err(e) = diag_log.save(&sdcard, catalog.dir(), loop_ns) {
                    info!("diag log: {}", e.as_str());
                }
            }
            let ejects = drive::EJECTS.load(Relaxed);
            if ejects != seen_ejects {
                // The Mac ejected the disk: offer the menu to put one back.
                seen_ejects = ejects;
                menu.show(here(&catalog, &current), now);
                next_display = now;
            }

            // ---- Knob ----
            let turns = KNOB_TURNS.swap(0, Relaxed);
            let presses = KNOB_PRESSES.swap(0, Relaxed);
            if turns != 0 || presses != 0 {
                next_display = now;
                if presses != 0 {
                    menu.note = None;
                }
                if !menu.open {
                    // The first touch only opens the menu.
                    menu.show(here(&catalog, &current), now);
                } else if menu.settings {
                    menu.turn(turns, SET_ITEMS, now);
                    if presses != 0 {
                        let in_root = root_image(&current[0]).or(root_image(&current[1]));
                        match settings_press(menu.cursor, &mut settings_file, in_root, &sdcard, catalog.dir(), &mut menu.note) {
                            SettingsPress::Stay => {
                                // The seek setting applies at once (also if it couldn't be saved).
                                keep_rotation = settings_file.config().keep_rotation;
                                drive::KEEP_ROTATION.store(keep_rotation || synced, Relaxed);
                                // Remembering, if just turned on, starts from what's in the drives
                                // (and HD20 + floppy mode's HD20).
                                for d in 0..drives.max(1 + chain as usize) {
                                    let held = if chain && d == 1 { hd.is_some() } else { images[d].is_some() && !knob_out[d] };
                                    let image = current[d].as_ref().filter(|_| held);
                                    if let Some(note) = remember(&sdcard, &mut settings_file, d, image.map(|(dir, n)| (dir, n.as_str())), catalog.dir()) {
                                        menu.note = Some(note);
                                    }
                                }
                            }
                            SettingsPress::Back => menu.leave_settings(MENU_SETTINGS, now),
                            SettingsPress::Restart => {
                                // Once the disks are out and their writes are saved (below).
                                menu.open = false;
                                for d in 0..drives {
                                    eject(d);
                                }
                                swap = None;
                                insert = [false; 2];
                                restart = Some(now);
                            }
                        }
                    }
                } else {
                    menu.turn(turns, catalog.len() + menu.fixed, now);
                    let item = menu.cursor.checked_sub(menu.fixed);
                    if presses == 0 {
                        // Just turning.
                    } else if menu.cursor == MENU_SETTINGS {
                        menu.enter_settings(now);
                    } else if let Some(next) = item.and_then(|i| catalog.folder(i)) {
                        // A folder or `..`: list it, and stay in the menu.
                        let up = item.and_then(|i| catalog.get(i)).is_some_and(|c| c.is_up());
                        let left = Name::from(catalog.dir().path().rsplit('/').next().unwrap_or(""));
                        match next.and_then(|dir| image::scan(&sdcard, Some(&dir), filter)) {
                            Ok(c) => {
                                catalog = c;
                                // Going up, land on the folder just left.
                                let at = if up { catalog.find(left.as_str()) } else { None };
                                menu.show(at.or(here(&catalog, &current)), now);
                            }
                            Err(e) => menu.note = Some(e),
                        }
                    } else {
                        menu.open = false;
                        // Which drive the image (or Eject) is for: in dual mode its kind says.
                        let target = match item.and_then(|i| catalog.get(i)) {
                            None => Ok(if menu.cursor == MENU_EJECT_B { 1 } else { 0 }),
                            Some(file) => drive_of(dual, chain, &sdcard, catalog.dir(), file),
                        };
                        match (item, target) {
                            (_, Err(e)) => {
                                menu.open = true;
                                menu.note = Some(e);
                            }
                            (Some(i), Ok(1)) if chain => {
                                // The HD20's: served at once (nothing waits on an eject).
                                if Some(i) != loaded_index(&catalog, &current[1])
                                    && let Some(file) = catalog.get(i).cloned()
                                {
                                    let dir = catalog.dir().clone();
                                    match load_chain_hd(&sdcard, &dir, &file, unsafe { &mut *maps[1] }, &mut server, &mut hd, &mut current[1], &mut sources[1], &mut display) {
                                        Ok(()) => {
                                            if let Some(note) = remember(&sdcard, &mut settings_file, 1, Some((&dir, file.name.as_str())), &dir) {
                                                menu.show(here(&catalog, &current), now);
                                                menu.note = Some(note);
                                            }
                                        }
                                        Err(e) => {
                                            menu.show(Some(i), now);
                                            let mut l = Line::new();
                                            let _ = write!(l, "Can't load: {}", e.as_str());
                                            menu.note = Some(l);
                                        }
                                    }
                                    KNOB_TURNS.store(0, Relaxed);
                                    KNOB_PRESSES.store(0, Relaxed);
                                }
                            }
                            (None, Ok(d)) => {
                                // Eject, and forget any disk waiting to go in.
                                eject(d);
                                if swap.as_ref().is_some_and(|s| s.0 == d) {
                                    swap = None;
                                }
                                insert[d] = false;
                                knob_out[d] = true;
                                if let Some(note) = remember(&sdcard, &mut settings_file, d, None, catalog.dir()) {
                                    menu.show(here(&catalog, &current), now);
                                    menu.note = Some(note);
                                }
                            }
                            (Some(i), Ok(d)) if Some(i) == loaded_index(&catalog, &current[d]) => {
                                swap = None;
                                knob_out[d] = false;
                                if let Some(note) = remember(&sdcard, &mut settings_file, d, Some((catalog.dir(), current_name(&current[d]))), catalog.dir()) {
                                    menu.show(here(&catalog, &current), now);
                                    menu.note = Some(note);
                                }
                                if !disk_in_of(d).load(Relaxed) {
                                    // Putting the same disk back: whether or not the computer
                                    // noticed it go, what it believes will be right.
                                    drive::MAC_SAW_EMPTY.store(true, Relaxed);
                                    insert[d] = true;
                                }
                            }
                            (Some(i), Ok(d)) => {
                                if let Some(file) = catalog.get(i) {
                                    insert[d] = false;
                                    eject(d);
                                    swap = Some((d, catalog.dir().clone(), file.clone(), now));
                                }
                            }
                        }
                    }
                }
            }
            if menu.open && now - menu.used_at >= MENU_TIMEOUT_US {
                menu.open = false;
                next_display = now;
            }

            // Load the chosen image (or restart) once the old one is out and its writes are saved.
            // No gate opens once core 1 has seen the disk out (well within 1 ms), and WRITING
            // covers a capture until it is queued.
            let saved = !drive::WRITING.load(Acquire)
                && drive::CAP_TAIL.load(Relaxed) == drive::CAP_HEAD.load(Acquire)
                && !cyl.as_ref().is_some_and(|c| c.is_dirty() || c.checksum_pending());
            if let Some(emptied_at) = restart
                && now - emptied_at >= 1_000
                && saved
            {
                reboot(&mut display);
            }
            if let Some((_, _, _, emptied_at)) = swap
                && now - emptied_at >= 1_000
                && saved
            {
                choice = swap.map(|(d, dir, file, _)| (d, dir, file));
                // The other drive's image stays: what's known of it with it.
                if let Some(c) = cyl.as_ref() {
                    rotations[cyl_drive] = Some(*c.rotations());
                }
                break;
            }

            // The screen goes out a piece a pass (under 1 ms each): a whole redraw kept core 0 from
            // following the head for ~7 ms, and a loader that reads a track a revolution after
            // each step (Hard Hat Mack) can't wait that long on top of the SD read.
            if let Some(row) = presenting {
                let _ = display.present_rows(row, PRESENT_ROWS);
                presenting = (row + PRESENT_ROWS < ssd1322::HEIGHT).then_some(row + PRESENT_ROWS);
            }
            let disk_in = drive::DISK_IN.load(Relaxed);
            if now >= next_display {
                next_display = now + DISPLAY_EVERY_US;

                if menu.open && menu.settings {
                    let rows = settings_rows(&settings_file, mode, synced, root_image(&current[0]).or(root_image(&current[1])), &menu.note);
                    let mut lines = [""; ROWS];
                    for (line, row) in lines.iter_mut().zip(rows.iter()) {
                        *line = row.as_str();
                    }
                    draw(&mut display, &lines, Some(1 + menu.cursor));
                    presenting = Some(0);
                    continue;
                }
                if menu.open {
                    let mut rows: [Line; ROWS] = core::array::from_fn(|_| Line::new());
                    match &menu.note {
                        Some(note) => {
                            let _ = rows[0].write_str(note.as_str());
                        }
                        None => {
                            let entry = menu.cursor.saturating_sub(menu.fixed - 1);
                            let _ = write!(rows[0], "{:<44.44} {}/{}", catalog.dir().path(), entry, catalog.len());
                        }
                    }
                    let shown = menu.top..(menu.top + ROWS - 1).min(catalog.len() + menu.fixed);
                    let marked = [loaded_index(&catalog, &current[0]), loaded_index(&catalog, &current[1])];
                    for (row, item) in rows[1..].iter_mut().zip(shown) {
                        if item == MENU_SETTINGS {
                            let _ = row.write_str("  [Settings...]");
                            continue;
                        }
                        match item.checked_sub(menu.fixed).and_then(|i| catalog.get(i).map(|e| (i, e))) {
                            None if dual => {
                                let d = (item == MENU_EJECT_B) as usize;
                                let what = if d == 1 { "5.25" } else { "3.5" };
                                let empty = if disk_in_of(d).load(Relaxed) { "" } else { " (no disk in)" };
                                let _ = write!(row, "  [Eject {}\"]{}", what, empty);
                            }
                            None => {
                                let _ = row.write_str(if disk_in { "  [Eject]" } else { "  [Eject] (no disk in)" });
                            }
                            Some((i, entry)) => {
                                let mark = if marked.contains(&Some(i)) { '*' } else { ' ' };
                                let _ = write!(row, "{} {:<42.42} {:>5}", mark, entry.name.as_str(), entry.describe());
                            }
                        }
                    }
                    let mut lines = [""; ROWS];
                    for (line, row) in lines.iter_mut().zip(rows.iter()) {
                        *line = row.as_str();
                    }
                    draw(&mut display, &lines, Some(1 + menu.cursor - menu.top));
                    presenting = Some(0);
                    continue;
                }

                let loops = drive::LOOPS.load(Relaxed);
                // LOOPS is a 24-bit count of 256-pass units.
                let d_loops = u64::from(loops.wrapping_sub(last_loops.0) & 0xFF_FFFF) * 256;
                if d_loops > 0 {
                    loop_ns = (now - last_loops.1) * 1000 / d_loops;
                }
                last_loops = (loops, now);

                let selected = drive::SELECTED.swap(false, Relaxed);
                let motor = drive::MOTOR_ON.load(Relaxed);
                let track = drive::HEAD_TRACK.load(Relaxed);
                let inp = unsafe { (*pac::SIO::ptr()).gpio_in().read().bits() };
                let bit = |pin: u8| (inp >> pin & 1) as u8;
                // Weak-bit spots in the playing cylinder, if any.
                let playing = drive::PLAYING.load(Acquire);
                let weak_spots: usize = match drive::cylinder_track(playing) {
                    Some(_) => weak[drive::cylinder_set(playing)].iter().map(|w| w.len()).sum(),
                    None => 0,
                };
                let rw = |d: usize| if writable_of(d).load(Relaxed) { "RW" } else { "RO" };
                let saving = |l: &mut Line| {
                    let _ = write!(
                        l,
                        " wr:{} err:{} drop:{} save:{}ms{}{}",
                        writes_ok,
                        write_errors,
                        drive::CAP_DROPPED.load(Relaxed),
                        flush_us / 1000,
                        if drive::WRITING.load(Relaxed) { " W" } else { "" },
                        // A DiskCopy checksum still to be written.
                        if cyl.as_ref().is_some_and(|c| c.checksum_pending()) { " CK" } else { "" }
                    );
                };

                if dual {
                    // Both drives: each one's image (or how it loaded, every other 2 s), its state
                    // and its head.
                    let state = |d: usize| match (disk_in_of(d).load(Relaxed), insert[d], swap.as_ref().is_some_and(|s| s.0 == d)) {
                        (true, _, _) => "IN",
                        (false, true, _) => "WAIT",
                        (false, false, true) => "SWAP",
                        (false, false, false) => "OUT",
                    };
                    let disk = |d: usize, what: &str| {
                        let mut l = Line::new();
                        let _ = match &images[d] {
                            Some(img) if (now / 2_000_000) % 2 == 0 => write!(l, "{}: {}", what, img.name.as_str()),
                            _ => write!(l, "{}: {}", what, sources[d].as_str()),
                        };
                        l
                    };
                    let mut l0 = Line::new();
                    let _ = write!(l0, "{}/{}", rw(0), rw(1));
                    saving(&mut l0);
                    let mut l1 = disk(0, "3.5\"");
                    if restart.is_some() {
                        l1 = Line::from("Restarting once the disks' changes are saved");
                    } else if let Some((_, _, file, _)) = &swap {
                        l1 = Line::new();
                        let _ = write!(l1, "Ejecting for {}", file.name.as_str());
                    } else if write_errors > 0 && (now / 2_000_000) % 2 == 1 {
                        l1 = Line::from(last_write_error);
                    }
                    let l2 = disk(1, "5.25\"");
                    let mut l3 = Line::new();
                    let _ = write!(l3, "3.5\"  {} Motor:{} Trk:{} Side:{}", state(0), if motor { "ON" } else { "OFF" }, track, drive::SIDE.load(Relaxed));
                    if drive::KEEP_ROTATION.load(Relaxed) {
                        let _ = l3.write_str(" Spin");
                    }
                    let q = drive::HEAD_QUARTER.load(Relaxed);
                    let mut l4 = Line::new();
                    let _ = write!(
                        l4,
                        "5.25\" {} Motor:{} Trk:{}.{:02} Ph:{:04b}",
                        state(1),
                        if drive::B_MOTOR.load(Relaxed) { "ON" } else { "OFF" },
                        q / 4,
                        q % 4 * 25,
                        drive::PHASES.load(Relaxed)
                    );
                    let mut l5 = Line::new();
                    let held = match (&cyl, cyl_drive) {
                        (None, _) => "-",
                        (_, 0) => "3.5\"",
                        _ => "5.25\"",
                    };
                    let _ = write!(l5, "Held:{} sd:{}us enc:{}us SDerr:{}", held, sd_us, build_us, sd_errors);
                    if weak_spots > 0 {
                        let _ = write!(l5, " Weak:{}", weak_spots);
                    }
                    let mut l6 = Line::new();
                    let _ = write!(
                        l6,
                        "Last:{} Steps:{} Ejects:{} Core1:{}ns",
                        command_name(drive::LAST_CMD.load(Relaxed)),
                        drive::STEPS.load(Relaxed),
                        drive::EJECTS.load(Relaxed),
                        loop_ns
                    );
                    let mut l7 = Line::new();
                    let _ = write!(
                        l7,
                        "CA:{}{}{} SEL:{} STB:{} /EN:{} /35:{} /D2:{} /WRQ:{} WR:{}",
                        bit(drive::PIN_CA2),
                        bit(drive::PIN_CA1),
                        bit(drive::PIN_CA0),
                        bit(drive::PIN_SEL),
                        bit(drive::PIN_LSTRB),
                        bit(drive::PIN_ENBL),
                        bit(drive::PIN_EN35),
                        bit(drive::PIN_DRIVE2),
                        bit(drive::PIN_WRREQ),
                        bit(drive::PIN_WR)
                    );
                    let _ = selected;
                    let lines = [&l0, &l1, &l2, &l3, &l4, &l5, &l6, &l7].map(|l| l.as_str());
                    draw(&mut display, &lines, None);
                    presenting = Some(0);
                    continue;
                }

                let waiting = insert[0] && !drive::MAC_SAW_EMPTY.load(Relaxed);
                let mut l1 = Line::new();
                match (&images[0], &swap) {
                    _ if restart.is_some() => {
                        let _ = l1.write_str("Restarting once the disk's changes are saved");
                    }
                    (_, Some((_, _, file, _))) => {
                        let _ = write!(l1, "Ejecting for {}", file.name.as_str());
                    }
                    (_, None) if waiting && apple35 => {
                        let _ = l1.write_str("Waiting for the Apple II to see the drive empty");
                    }
                    (_, None) if waiting && !apple2 => {
                        let _ = l1.write_str("Waiting for the Mac to see the drive empty");
                    }
                    (_, None) if write_errors > 0 && (now / 2_000_000) % 2 == 1 => {
                        let _ = l1.write_str(last_write_error);
                    }
                    (Some(l), None) if disk_in || insert[0] || (now / 2_000_000) % 2 == 0 => {
                        let _ = l1.write_str(l.name.as_str());
                    }
                    _ => {
                        let _ = l1.write_str("No disk in: press the knob to choose one");
                    }
                }
                let mut l2 = Line::new();
                let _ = write!(
                    l2,
                    "Disk:{} Motor:{} {}:{}",
                    match (disk_in, insert[0], &swap) {
                        (true, _, _) => "IN",
                        (false, true, _) => "WAIT",
                        (false, false, Some(_)) => "SWAP",
                        (false, false, None) => "OUT",
                    },
                    if motor { "ON" } else { "OFF" },
                    host,
                    if selected { "ACTIVE" } else { "idle" },
                );
                if weak_spots > 0 {
                    let _ = write!(l2, " Weak:{}", weak_spots);
                }
                if drive::KEEP_ROTATION.load(Relaxed) && !apple2 {
                    let _ = l2.write_str(" Spin");
                }
                let mut l3 = Line::new();
                if apple2 {
                    let q = drive::HEAD_QUARTER.load(Relaxed);
                    let _ = write!(
                        l3,
                        "Trk:{}.{:02} Ph:{:04b} sd:{}us enc:{}us",
                        q / 4,
                        q % 4 * 25,
                        drive::PHASES.load(Relaxed),
                        sd_us,
                        build_us
                    );
                } else {
                    let _ = write!(
                        l3,
                        "Trk:{} Side:{} TACH:{}rpm sd:{}us enc:{}us",
                        track,
                        drive::SIDE.load(Relaxed),
                        gcr::rpm(track),
                        sd_us,
                        build_us
                    );
                }
                let mut l4 = Line::new();
                if apple2 {
                    let _ = write!(l4, "Steps:{} SDerr:{}", drive::STEPS.load(Relaxed), sd_errors);
                } else if chain {
                    // The HD20 in place of the floppy's counters.
                    let s = &server.session;
                    match &hd {
                        Some(d) if !drive::DCD_LIVE.load(Relaxed) => {
                            let _ = write!(l4, "HD20: restart the Mac to use {}", d.name.as_str());
                        }
                        Some(d) => {
                            let _ = write!(
                                l4,
                                "HD20 cmd:{} rd:{} wr:{} err:{} {}",
                                server.commands,
                                s.blocks_read,
                                s.blocks_written,
                                s.errors + server.garbled,
                                d.name.as_str()
                            );
                        }
                        None => {
                            let _ = l4.write_str("HD20: no disk");
                        }
                    }
                } else {
                    let _ = write!(
                        l4,
                        "Steps:{} Cmds:{} Ejects:{} SDerr:{}",
                        drive::STEPS.load(Relaxed),
                        drive::STROBES.load(Relaxed),
                        drive::EJECTS.load(Relaxed),
                        sd_errors
                    );
                }
                let mut l0 = Line::from(rw(0));
                saving(&mut l0);
                let mut l5 = Line::new();
                let mut l6 = Line::new();
                if apple2 {
                    let _ = write!(l5, "Core1 loop:{}ns", loop_ns);
                    let _ = write!(
                        l6,
                        "In PH3-0:{}{}{}{} /EN:{} /WRQ:{} WR:{}",
                        bit(drive::PIN_LSTRB),
                        bit(drive::PIN_CA2),
                        bit(drive::PIN_CA1),
                        bit(drive::PIN_CA0),
                        bit(drive::PIN_ENBL),
                        bit(drive::PIN_WRREQ),
                        bit(drive::PIN_WR)
                    );
                } else if apple35 {
                    let _ = write!(
                        l5,
                        "Last:{}  Core1 loop:{}ns",
                        command_name(drive::LAST_CMD.load(Relaxed)),
                        loop_ns
                    );
                    let _ = write!(
                        l6,
                        "CA:{}{}{} SEL:{} STB:{} /EN:{} /35:{} /D2:{} /WRQ:{} WR:{}",
                        bit(drive::PIN_CA2),
                        bit(drive::PIN_CA1),
                        bit(drive::PIN_CA0),
                        bit(drive::PIN_SEL),
                        bit(drive::PIN_LSTRB),
                        bit(drive::PIN_ENBL),
                        bit(drive::PIN_EN35),
                        bit(drive::PIN_DRIVE2),
                        bit(drive::PIN_WRREQ),
                        bit(drive::PIN_WR)
                    );
                } else {
                    let _ = write!(
                        l5,
                        "Last:{}  Core1 loop:{}ns",
                        command_name(drive::LAST_CMD.load(Relaxed)),
                        loop_ns
                    );
                    let _ = write!(
                        l6,
                        "In CA2-0:{}{}{} SEL:{} LSTRB:{} /EN:{} /WRQ:{} WR:{}",
                        bit(drive::PIN_CA2),
                        bit(drive::PIN_CA1),
                        bit(drive::PIN_CA0),
                        bit(drive::PIN_SEL),
                        bit(drive::PIN_LSTRB),
                        bit(drive::PIN_ENBL),
                        bit(drive::PIN_WRREQ),
                        bit(drive::PIN_WR)
                    );
                    #[cfg(feature = "diag")]
                    if !chain {
                        (l5, l6) = (Line::new(), Line::new());
                        diag_lines(&mut l5, &mut l6, loop_ns);
                    }
                }
                presenting = Some(0);
                draw(
                    &mut display,
                    &[
                        l0.as_str(),
                        l1.as_str(),
                        sources[0].as_str(),
                        l2.as_str(),
                        l3.as_str(),
                        l4.as_str(),
                        l5.as_str(),
                        l6.as_str(),
                    ],
                    None,
                );
            }
        }
    }
}

/// The image menu the knob drives. Item 0 is Eject, item 1 ([`MENU_SETTINGS`]) the settings page,
/// and item `MENU_FIXED + i` entry `i` of the catalog (`..`, a folder or an image).
struct Menu {
    open: bool,
    /// On the settings page ([`settings_rows`]); `cursor` is one of its `SET_*` items.
    settings: bool,
    cursor: usize,
    /// First item on screen.
    top: usize,
    /// When the knob was last used, for closing the menu.
    used_at: u64,
    /// Shown in place of the title row until the knob is next pressed: why an image didn't load
    /// or a folder couldn't be listed.
    note: Option<Line>,
    /// Items before the catalog's entries: [`MENU_FIXED`], or just Settings in HD20 mode.
    fixed: usize,
}

/// The position in `catalog` of image `current` (directory, name), if it is in that directory.
fn loaded_index(catalog: &Catalog, current: &Option<(Dir, Name)>) -> Option<usize> {
    let (_, name) = current.as_ref().filter(|(dir, _)| dir == catalog.dir())?;
    catalog.find(name.as_str())
}

impl Menu {
    /// Open with the cursor on entry `current` (or the first).
    fn show(&mut self, current: Option<usize>, now: u64) {
        self.open = true;
        self.settings = false;
        self.cursor = current.map_or(self.fixed, |i| i + self.fixed);
        self.top = self.cursor.saturating_sub(ROWS - 2);
        self.used_at = now;
    }

    /// Move `turns` items down (negative: up) among `items`, stopping at the ends, and scroll to
    /// keep the cursor on the `ROWS - 1` rows below the title.
    fn turn(&mut self, turns: i32, items: usize, now: u64) {
        let cursor = self.cursor as i64 + i64::from(turns) * KNOB_DIRECTION;
        self.cursor = cursor.clamp(0, items as i64 - 1) as usize;
        self.top = self.top.min(self.cursor).max((self.cursor + 2).saturating_sub(ROWS));
        self.used_at = now;
    }

    fn enter_settings(&mut self, now: u64) {
        self.settings = true;
        self.cursor = SET_MODE;
        self.used_at = now;
    }

    /// Back from the settings page, onto menu item `item`.
    fn leave_settings(&mut self, item: usize, now: u64) {
        self.settings = false;
        self.cursor = item;
        self.top = self.cursor.saturating_sub(ROWS - 2);
        self.used_at = now;
    }
}

/// The loaded image's name, if it is in the root (where `autoload` looks).
fn root_image(current: &Option<(Dir, Name)>) -> Option<&str> {
    current.as_ref().filter(|(dir, _)| dir.is_root()).map(|(_, name)| name.as_str())
}

/// What a press on the settings page asks for.
enum SettingsPress {
    Stay,
    Back,
    /// Start again from power-on, as for a new mode.
    Restart,
}

/// The knob pressed on settings item `item`. Mode, autoload, the seek setting and remembering step
/// to their next value, which is saved at once (turning remembering on doesn't note what's in the
/// drives yet: the caller does that with [`remember`]); autoload goes off → first → the loaded image `here` (if
/// it's in the root) → off. `dir` is any directory of the card. What was saved, or why not, goes in `note`.
fn settings_press<D>(item: usize, text: &mut config::Text, here: Option<&str>, card: &D, dir: &Dir, note: &mut Option<Line>) -> SettingsPress
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: core::fmt::Debug,
{
    let config = text.config();
    let (key, value) = match item {
        SET_BACK => return SettingsPress::Back,
        SET_RESTART => return SettingsPress::Restart,
        SET_MODE => {
            let next = match config.mode {
                Mode::Floppy => Mode::Hd20,
                Mode::Hd20 => Mode::Hd20Floppy,
                Mode::Hd20Floppy => Mode::Apple2,
                Mode::Apple2 => Mode::Apple35,
                Mode::Apple35 => Mode::Dual,
                Mode::Dual => Mode::SmartPort,
                Mode::SmartPort => Mode::Floppy,
            };
            (config::MODE, Name::from(next.value()))
        }
        SET_ROTATION => (config::KEEP_ROTATION, Name::from(if config.keep_rotation { "off" } else { "on" })),
        SET_REMEMBER => (config::REMEMBER, Name::from(if config.remember { "off" } else { "on" })),
        _ => {
            let next = match (config.autoload, here) {
                (Autoload::Off, _) => Autoload::First,
                (Autoload::First, Some(here)) => Autoload::Named(here),
                _ => Autoload::Off,
            };
            (config::AUTOLOAD, Name::from(next.value()))
        }
    };
    let mut l = Line::new();
    if text.partial {
        let _ = write!(l, "{} is too long to change here", config::FILE_NAME);
    } else if !text.set(key, value.as_str()) {
        let _ = write!(l, "{} is full", config::FILE_NAME);
    } else {
        match image::write_settings(card, dir, text.as_bytes()) {
            Ok(()) => {
                let _ = write!(l, "Saved {} = {}", key, value.as_str());
            }
            Err(e) => {
                let _ = write!(l, "Can't save: {}", e.as_str());
            }
        }
    }
    *note = Some(l);
    SettingsPress::Stay
}

/// The settings page: what the file says, for the mode the emulator is `running` in. `synced`: the
/// loaded image is a synchronized MOOF, which keeps the rotation whatever the setting. `here` is
/// the loaded image if it's in the root; `note` replaces the title.
fn settings_rows(text: &config::Text, running: Mode, synced: bool, here: Option<&str>, note: &Option<Line>) -> [Line; ROWS] {
    let mut rows: [Line; ROWS] = core::array::from_fn(|_| Line::new());
    let config = text.config();
    let _ = match note {
        Some(note) => rows[0].write_str(note.as_str()),
        None => write!(rows[0], "Settings, saved in {}", config::FILE_NAME),
    };
    let _ = rows[1 + SET_BACK].write_str("  [Back]");
    let _ = write!(
        rows[1 + SET_MODE],
        "  Mode: {}{}",
        match config.mode {
            Mode::Floppy => "Mac floppy drive",
            Mode::Hd20 => "HD20 hard disk",
            Mode::Hd20Floppy => "HD20 + Mac floppy drive",
            Mode::Apple2 => "Apple II Disk II",
            Mode::Apple35 => "Apple II 3.5\" drive",
            Mode::Dual => "Apple II 5.25\" + 3.5\"",
            Mode::SmartPort => "Apple II SmartPort",
        },
        if config.mode == running { "" } else { " (after restart)" }
    );
    let _ = match config.autoload {
        Autoload::Off => rows[1 + SET_AUTOLOAD].write_str("  Autoload: off (menu at power-on)"),
        Autoload::First => rows[1 + SET_AUTOLOAD].write_str("  Autoload: first image in root"),
        Autoload::Named(name) => write!(rows[1 + SET_AUTOLOAD], "  Autoload: {}", name),
    };
    let _ = write!(
        rows[1 + SET_ROTATION],
        "  Seek: {}{}",
        if config.keep_rotation { "keep rotation" } else { "from sector 0" },
        match running {
            Mode::Apple2 => " (a Disk II keeps it)",
            Mode::Hd20 | Mode::SmartPort => " (3.5\" drives)",
            _ if synced && !config.keep_rotation => " (this synced MOOF keeps it)",
            _ => "",
        }
    );
    let _ = rows[1 + SET_REMEMBER].write_str(if config.remember { "  Remember images: on (back at power-on)" } else { "  Remember images: off" });
    let _ = rows[1 + SET_RESTART].write_str(if config.mode == running { "  [Restart now]" } else { "  [Restart now in the new mode]" });
    let _ = rows[1 + SET_ITEMS].write_str(match here {
        None => "To autoload an image, load it from / first.",
        Some(_) => "Press to change; saved at once.",
    });
    rows
}

/// The name of image `current`, or "" for none.
fn current_name(current: &Option<(Dir, Name)>) -> &str {
    current.as_ref().map_or("", |(_, name)| name.as_str())
}

/// With `remember` on, what was last chosen for drive or unit `slot` (`config::LAST`), found
/// folder by folder in `filter`'s listing; `catalog` is left listing the image's folder. `None`
/// if remembering is off or nothing is noted for `slot` yet (then `catalog` is untouched and
/// `autoload` applies); `Some(Ok(None))` if it was empty.
#[inline(never)]
fn remembered<D>(card: &D, text: &config::Text, slot: usize, filter: Filter, catalog: &mut Catalog) -> Option<Result<Option<(Dir, Candidate)>, Line>>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: core::fmt::Debug,
{
    let config = text.config();
    let path = config.last.get(slot).copied().flatten().filter(|_| config.remember)?;
    let path = path.trim_start_matches('/');
    if path.is_empty() {
        return Some(Ok(None));
    }
    let missing = || {
        let mut l = Line::new();
        let _ = write!(l, "Remembered /{} isn't there", path);
        l
    };
    let mut found = || -> Result<(Dir, Candidate), Line> {
        // Listed in place, from the root down (`scan` would put catalogs on this frame).
        let mut parts = path.split('/').peekable();
        let mut dir: Option<Dir> = None;
        while let Some(part) = parts.next() {
            image::scan_into(card, dir.as_ref(), filter, catalog)?;
            // The name as written, else ignoring case.
            let i = catalog
                .find(part)
                .or_else(|| (0..catalog.len()).find(|&i| catalog.get(i).is_some_and(|c| !c.is_up() && c.name.as_str().eq_ignore_ascii_case(part))))
                .ok_or_else(missing)?;
            if parts.peek().is_none() {
                let file = catalog.get(i).filter(|c| c.is_image()).cloned().ok_or_else(missing)?;
                return Ok((catalog.dir().clone(), file));
            }
            dir = Some(catalog.folder(i).ok_or_else(missing)??);
        }
        Err(missing())
    };
    Some(found().map(Some))
}

/// With `remember` on, note `image` (its folder and name, or `None` for empty) as what's in drive
/// or unit `slot`, saving the settings file if that changes it. `dir` is any directory of the
/// card. Returns why it couldn't be saved, if it couldn't.
#[inline(never)]
fn remember<D>(card: &D, text: &mut config::Text, slot: usize, image: Option<(&Dir, &str)>, dir: &Dir) -> Option<Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: core::fmt::Debug,
{
    let mut path = FixedStr::<1300>::new();
    if let Some((folder, name)) = image {
        let _ = write!(path, "{}/{}", folder.path().trim_end_matches('/'), name);
    }
    let config = text.config();
    if !config.remember || config.last[slot] == Some(path.as_str()) {
        return None;
    }
    let mut l = Line::new();
    if text.partial {
        let _ = write!(l, "Can't remember the image: {} is too long", config::FILE_NAME);
    } else if !text.set(config::LAST[slot], path.as_str()) {
        let _ = write!(l, "Can't remember the image: {} is full", config::FILE_NAME);
    } else if let Err(e) = image::write_settings(card, dir, text.as_bytes()) {
        let _ = write!(l, "Can't remember the image: {}", e.as_str());
    } else {
        return None;
    }
    Some(l)
}

/// Start again from power-on (the ROM's reboot, through the watchdog).
fn reboot<SPI, CS, DC, RST>(display: &mut Display<SPI, CS, DC, RST>) -> !
where
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    show(display, &["Restarting..."], None);
    hal::reboot::reboot(hal::reboot::RebootKind::Normal, hal::reboot::RebootArch::Normal)
}

/// `diag` builds, floppy mode: what the Mac did since the disk last went in. Line 5 counts its
/// commands (direction in/out, steps, motor on/off, GCR/MFM mode); line 6 is how long it had the
/// motor on while selected, how much of that /READY was high, and the registers it addressed
/// meanwhile, most first, in percent of core 1's samples.
#[cfg(feature = "diag")]
fn diag_lines(l5: &mut Line, l6: &mut Line, loop_ns: u64) {
    let c = |cmd: usize| drive::CMD_COUNT[cmd].load(Relaxed);
    let _ = write!(
        l5,
        "In:{} Out:{} Stp:{} On:{} Off:{} G:{} M:{}",
        c(0b0000),
        c(0b0001),
        c(0b0100),
        c(0b1000),
        c(0b1001),
        c(0b0111),
        c(0b0110)
    );
    let mut n = [0u32; 16];
    for (v, s) in n.iter_mut().zip(drive::REG_SAMPLES.iter()) {
        *v = s.load(Relaxed);
    }
    let total: u32 = n.iter().sum();
    if total == 0 {
        let _ = l6.write_str("Motor not on since the disk went in");
        return;
    }
    let pct = |v: u32| u64::from(v) * 100 / u64::from(total);
    // Housekeeping samples once every 16 passes.
    let _ = write!(
        l6,
        "{}ms nrdy:{}%",
        u64::from(total) * 16 * loop_ns / 1_000_000,
        pct(drive::NOT_READY.load(Relaxed))
    );
    for _ in 0..5 {
        let (k, v) = n.iter().copied().enumerate().max_by_key(|&(_, v)| v).unwrap_or((0, 0));
        if v == 0 {
            break;
        }
        let _ = write!(l6, " R{}:{}", k, pct(v));
        n[k] = 0;
    }
}

/// `diag` builds: core 1's event log (`drive::LOG_*`) as text, appended to `emulator.log` in the
/// card's root (emptied at power-on).
#[cfg(feature = "diag")]
struct DiagLog {
    /// Events saved so far, and whether the file has been started this power-on.
    saved: u32,
    started: bool,
    next: u64,
}

#[cfg(feature = "diag")]
impl DiagLog {
    const FILE: &str = "emulator.log";
    const EVERY_US: u64 = 5_000_000;
    /// Events newer than this many short of the ring's length may be overwritten mid-save.
    const MARGIN: u32 = 256;

    fn new() -> Self {
        Self { saved: 0, started: false, next: 0 }
    }

    fn save<D>(&mut self, card: &D, dir: &Dir, loop_ns: u64) -> Result<(), Line>
    where
        D: embedded_sdmmc::BlockDevice,
        D::Error: core::fmt::Debug,
    {
        let head = drive::LOG_HEAD.load(Acquire);
        if self.started && head == self.saved {
            return Ok(());
        }
        let window = drive::LOG_LEN as u32 - Self::MARGIN;
        let lost = head.wrapping_sub(self.saved).saturating_sub(window);
        let from = self.saved.wrapping_add(lost);
        let header = !self.started;
        image::write_root_file(card, dir, Self::FILE, self.started, &mut |sink| {
            let mut out = FixedStr::<512>::new();
            let mut line = Line::new();
            if header {
                let _ = writeln!(
                    out,
                    "rp2350-floppy-emulator diag log. Times: core 1's timer, s. RD level: what the register mux \
                     showed for the register (R8/R9 are data, shown 0). Core 1 loop {} ns.",
                    loop_ns
                );
            }
            if lost > 0 {
                let _ = writeln!(out, "-- {} events lost (the ring wrapped between saves)", lost);
            }
            let mut k = from;
            while k != head {
                let t = drive::LOG_T[k as usize % drive::LOG_LEN].load(Relaxed);
                let w = drive::LOG_W[k as usize % drive::LOG_LEN].load(Relaxed);
                k = k.wrapping_add(1);
                line.truncate(0);
                let _ = write!(line, "{:>5}.{:06} ", t / 1_000_000, t % 1_000_000);
                let data = w & 0xFF_FFFF;
                let reg = |line: &mut Line, d: u32| {
                    let r = d & 0xF;
                    let _ = write!(line, "R{}={}", r, d >> 4 & 1);
                    if r & 0b1110 == 8 {
                        let _ = line.write_str(" (data)");
                    }
                };
                match w >> 24 {
                    drive::LOG_SEL => {
                        let _ = line.write_str("select    ");
                        reg(&mut line, data);
                    }
                    drive::LOG_DESEL => {
                        let _ = write!(line, "deselect  ({} register changes)", data);
                    }
                    drive::LOG_REG => {
                        let _ = line.write_str("          ");
                        reg(&mut line, data);
                    }
                    drive::LOG_CMD => {
                        let _ = write!(line, "command   {} (track {})", command_name(data as u8 & 0xF), data >> 8);
                    }
                    drive::LOG_DISK => {
                        let _ = write!(line, "disk      {}", if data != 0 { "in" } else { "out" });
                    }
                    kind => {
                        let _ = write!(line, "? {:#x}", kind);
                    }
                }
                let _ = line.write_str("\n");
                if out.len() + line.len() > FixedStr::<512>::CAPACITY {
                    sink(out.as_str().as_bytes())?;
                    out.truncate(0);
                }
                let _ = out.write_str(line.as_str());
            }
            if out.len() > 0 {
                sink(out.as_str().as_bytes())?;
            }
            Ok(())
        })?;
        self.started = true;
        self.saved = head;
        Ok(())
    }
}

/// Take the disk out of drive `d` from under the computer. The Mac only notices at its next
/// request, so record that it hasn't seen the drive empty yet; core 1 sets
/// [`drive::MAC_SAW_EMPTY`] once it has. (Dual mode's drive 1, a Disk II, has no such signal.)
fn eject(d: usize) {
    if d == 0 && drive::DISK_IN.load(Relaxed) {
        drive::MAC_SAW_EMPTY.store(false, Relaxed);
    }
    disk_in_of(d).store(false, Relaxed);
}

/// Drive `d`'s disk-in flag for core 1: dual mode's drive 1 has its own.
fn disk_in_of(d: usize) -> &'static core::sync::atomic::AtomicBool {
    if d == 1 { &drive::B_DISK_IN } else { &drive::DISK_IN }
}

/// Drive `d`'s writable flag for core 1.
fn writable_of(d: usize) -> &'static core::sync::atomic::AtomicBool {
    if d == 1 { &drive::B_WRITABLE } else { &drive::WRITABLE }
}

/// Which drive image `c` goes in: in dual mode its kind says (`image::dual_drive`), in HD20 +
/// floppy mode a hard disk image is the HD20's (1), else the only one.
fn drive_of<D>(dual: bool, chain: bool, card: &D, dir: &Dir, c: &Candidate) -> Result<usize, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: core::fmt::Debug,
{
    match (dual, chain) {
        (true, _) => image::dual_drive(card, dir, c),
        // HD20 + floppy mode: drive 1 is the HD20.
        (_, true) => Ok(c.is_hard_disk() as usize),
        _ => Ok(0),
    }
}

/// HD20 + floppy mode: serve hard disk image `file` from `dir` on the HD20 (`hd20::insert`),
/// noting it in `hd`, `current` and `source`. Like HD20 mode, a Mac that has used the image it
/// replaces isn't served the new one until it restarts.
#[allow(clippy::too_many_arguments)]
fn load_chain_hd<D, SPI, CS, DC, RST>(
    card: &D,
    dir: &Dir,
    file: &Candidate,
    map: &mut image::BlockMap,
    server: &mut hd20::Server,
    hd: &mut Option<image::HdLoaded>,
    current: &mut Option<(Dir, Name)>,
    source: &mut Line,
    display: &mut Display<SPI, CS, DC, RST>,
) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: core::fmt::Debug,
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    let result = hd20::insert(card, dir, file, map, server.commands > 0, display);
    *source = Line::new();
    match result {
        Ok(d) => {
            let size = hd20::size_text(u64::from(d.volume.blocks) * dcd::BLOCK as u64);
            let _ = write!(source, "HD20: {} {} {} {}frag", d.volume.kind, size.as_str(), d.filesystem, d.fragments);
            *hd = Some(d);
            *current = Some((dir.clone(), file.name.clone()));
            server.reset();
            Ok(())
        }
        Err(e) => {
            *hd = None;
            *current = None;
            let _ = source.write_str("HD20: no disk");
            info!("Can't load {}: {}", file.name.as_str(), e.as_str());
            Err(e)
        }
    }
}

/// The position in `catalog` of either drive's image, for the menu to open on.
fn here(catalog: &Catalog, current: &[Option<(Dir, Name)>; 2]) -> Option<usize> {
    loaded_index(catalog, &current[0]).or(loaded_index(catalog, &current[1]))
}

/// Feed the knob's pins to its decoder, every millisecond.
#[exception]
fn SysTick() {
    let inp = unsafe { (*pac::SIO::ptr()).gpio_in().read().bits() };
    let high = |pin: u8| inp >> pin & 1 != 0;
    // Safety: only this handler touches KNOB, and it doesn't nest.
    let knob = unsafe { &mut *KNOB.0.get() };
    let event = knob.sample(high(knob::PIN_CLK), high(knob::PIN_DT), high(knob::PIN_SW));
    if event.detent != 0 {
        KNOB_TURNS.fetch_add(i32::from(event.detent), Relaxed);
    }
    if event.pressed {
        KNOB_PRESSES.fetch_add(1, Relaxed);
    }
}

/// Hand a built cylinder to core 1. `tag` is [`drive::CYL_DRIVE2`] for dual mode's second drive.
/// A Disk II's track also gives the quarter tracks that hear it (`span`) and its cells and
/// rotation in cells (`Cylinder::a2_geometry`), and `rate` its PIO clock ([`a2_rate`]).
#[allow(clippy::too_many_arguments)]
fn publish(published: &mut u32, set: usize, track: u8, tag: u32, sides: [(u32, u32, u32); 2], span: u32, geometry: (u32, u32), rate: (u32, u32)) {
    for (side, (ptr, len, rot)) in sides.into_iter().enumerate() {
        drive::CYL_PTR[set][side].store(ptr, Relaxed);
        drive::CYL_LEN[set][side].store(len, Relaxed);
        drive::CYL_ROT[set][side].store(rot, Relaxed);
    }
    drive::CYL_SPAN[set].store(span, Relaxed);
    let (bits, rotation) = geometry;
    drive::CYL_BITS[set].store(bits, Relaxed);
    drive::CYL_ROTC[set].store(rotation, Relaxed);
    drive::CYL_INV[set].store(if bits > 1 { ((1u64 << 32) / u64::from(bits)) as u32 } else { 0 }, Relaxed);
    drive::CYL_CLKDIV[set].store(rate.0, Relaxed);
    drive::CYL_Q32[set].store(rate.1, Relaxed);
    *published = drive::cylinder(set, track) | tag;
    drive::PUBLISHED.store(*published, Release);
}

/// A Disk II track's PIO clock `hz` as [`drive::CYL_CLKDIV`] and [`drive::CYL_Q32`] want it.
fn a2_rate(sys_hz: u32, hz: u32) -> (u32, u32) {
    (drive::clkdiv_word(sys_hz, hz), drive::a2_cells_q32(sys_hz, hz))
}

/// The quarter tracks around `head` that hear what it does, packed for [`drive::CYL_SPAN`]: a
/// drive reads the same track across them, so core 1 plays on while the head moves among them.
fn a2_span(disk: &image::Disk, head: u8) -> u32 {
    let key = disk.a2_track(head);
    let hears = |q: u8| disk.a2_track(q) == key;
    let (mut lo, mut hi) = (head, head);
    while lo > 0 && hears(lo - 1) {
        lo -= 1;
    }
    while hi < drive::A2_LAST_QUARTER && hears(hi + 1) {
        hi += 1;
    }
    drive::span(lo, hi)
}

/// Draw up to ROWS lines of text, with row `highlight` (if any) inverted.
fn show<SPI, CS, DC, RST>(display: &mut Display<SPI, CS, DC, RST>, lines: &[&str], highlight: Option<usize>)
where
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    draw(display, lines, highlight);
    let _ = display.present();
}

/// Draw [`show`]'s screen into the framebuffer only; the main loop sends it a few rows a pass.
fn draw<SPI, CS, DC, RST>(display: &mut Display<SPI, CS, DC, RST>, lines: &[&str], highlight: Option<usize>)
where
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    let style = MonoTextStyle::new(&FONT_5X8, Gray4::new(15));
    let inverse = MonoTextStyle::new(&FONT_5X8, Gray4::BLACK);
    display.clear_buffer(0);
    for (row, text) in lines.iter().take(ROWS).enumerate() {
        let at = Point::new(0, row as i32 * LINE_H);
        let style = if highlight == Some(row) {
            let _ = Rectangle::new(at, Size::new(256, LINE_H as u32))
                .into_styled(PrimitiveStyle::with_fill(Gray4::new(15)))
                .draw(display);
            inverse
        } else {
            style
        };
        let _ = Text::with_baseline(text, at, style, Baseline::Top).draw(display);
    }
}

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"Macintosh floppy drive emulator"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

// End of file
