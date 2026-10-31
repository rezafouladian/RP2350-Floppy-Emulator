//! SmartPort mode (`mode = smartport` in the settings file): core 0's side. Core 1 runs
//! `drive::run_smartport`, which receives the host's packets, checks them and does the handshake;
//! this answers each transaction (`smartport::command`) against the units' images, and runs the
//! knob menu and status screen.
//!
//! There are [`UNITS`] units, there from power-on whether they hold a disk or not: the host looks
//! for SmartPort devices when it starts up, and gives each an ID. A unit's disk is a block image
//! (`image::load_blocks`): 800K or less is a 3.5" disk, bigger a hard disk. Pressing an image in
//! the menu puts it in the first empty unit, or takes it out of the unit it's in; the host sees the
//! disk switched, as with a removable disk. The host's eject (CONTROL 4) empties the unit too.
//! Writes are saved as they're answered.

use core::fmt::{Debug, Write};
use core::sync::atomic::Ordering::{Acquire, Relaxed, Release};

use defmt::info;
use embedded_sdmmc::{Block, BlockDevice};

use crate::config::{self, Autoload, Mode};
use crate::drive;
use crate::image::{self, BlockImage, BlockMap, Catalog, Dir, Filter, Name};
use crate::smartport::{self, BLOCK, MAX_PACKET, MAX_DATA, UNITS, Units};
use crate::ssd1322::Display;
use crate::{DISPLAY_EVERY_US, KNOB_PRESSES, KNOB_TURNS, Line, MENU_TIMEOUT_US, Menu, ROWS, Ram, SET_ITEMS, SettingsPress};
use crate::{cylinder, dc42, hd20, reboot, remember, remembered, root_image, settings_press, settings_rows, show};

/// Runs of consecutive card sectors a unit's image can be in.
const RUNS: usize = 128;

/// Where each unit's image is on the card.
static MAPS: Ram<[BlockMap<RUNS>; UNITS]> = Ram(core::cell::UnsafeCell::new([const { BlockMap::new() }; UNITS]));

/// A unit's disk, and where it came from.
struct Unit {
    image: Option<(BlockImage, Dir, Name)>,
    /// The disk changed and the host hasn't been told (`Units::take_switched`).
    switched: bool,
}

/// The units as `smartport::command` sees them.
struct Disks<'a, D> {
    card: &'a D,
    maps: &'a [BlockMap<RUNS>; UNITS],
    units: &'a mut [Unit; UNITS],
    /// The host ejected a disk.
    ejected: bool,
}

/// The DIBs' names.
const NAMES: [&[u8]; UNITS] = [b"RP2350 1", b"RP2350 2", b"RP2350 3", b"RP2350 4"];

impl<D: BlockDevice> Disks<'_, D> {
    /// Unit `u`'s block `b` is file blocks `first` (and `first + 1` if `shift` isn't 0, the
    /// block starting that many bytes into the first).
    fn place(&self, u: usize, b: u32) -> Option<(usize, usize)> {
        let (img, _, _) = self.units[u].image.as_ref()?;
        Some(((img.offset / BLOCK as u32 + b) as usize, img.offset as usize % BLOCK))
    }
}

impl<D: BlockDevice> Units for Disks<'_, D> {
    fn blocks(&self, u: usize) -> Option<u32> {
        self.units[u].image.as_ref().map(|(img, _, _)| img.blocks)
    }

    fn writable(&self, u: usize) -> bool {
        self.units[u].image.as_ref().is_some_and(|(img, _, _)| !img.locked)
    }

    fn read(&mut self, u: usize, block: u32, out: &mut [u8; BLOCK]) -> Result<(), ()> {
        let (first, shift) = self.place(u, block).ok_or(())?;
        let mut b = [Block::new(), Block::new()];
        let n = if shift == 0 { 1 } else { 2 };
        cylinder::read_blocks(self.card, &self.maps[u], first, &mut b[..n]).map_err(|_| ())?;
        if shift != 0 {
            let (this, next) = b.split_at_mut(1);
            dc42::shift_sector(&mut this[0].contents, &next[0].contents, shift);
        }
        out.copy_from_slice(&b[0].contents);
        Ok(())
    }

    fn write(&mut self, u: usize, block: u32, data: &[u8; BLOCK]) -> Result<(), ()> {
        let (first, shift) = self.place(u, block).ok_or(())?;
        let mut b = [Block::new(), Block::new()];
        if shift == 0 {
            b[0].contents.copy_from_slice(data);
            return cylinder::write_blocks(self.card, &self.maps[u], first, &b[..1]).map_err(|_| ());
        }
        // The block spans two file blocks, which also hold its neighbours' bytes.
        cylinder::read_blocks(self.card, &self.maps[u], first, &mut b).map_err(|_| ())?;
        let (this, next) = b.split_at_mut(1);
        dc42::place_sector(&mut this[0].contents, &mut next[0].contents, data, shift);
        cylinder::write_blocks(self.card, &self.maps[u], first, &b).map_err(|_| ())
    }

    fn eject(&mut self, u: usize) {
        if self.units[u].image.take().is_some() {
            self.units[u].switched = true;
            self.ejected = true;
        }
    }

    fn take_switched(&mut self, u: usize) -> bool {
        core::mem::take(&mut self.units[u].switched)
    }

    fn name(&self, u: usize) -> &[u8] {
        NAMES[u]
    }
}

fn last_name(last: smartport::Last) -> &'static str {
    match last {
        smartport::Last::None => "-",
        smartport::Last::Init => "INIT",
        smartport::Last::Status => "STATUS",
        smartport::Last::Read => "READ",
        smartport::Last::Write => "WRITE",
        smartport::Last::Control => "CONTROL",
        smartport::Last::Other(smartport::FORMAT) => "FORMAT",
        smartport::Last::Other(_) => "OTHER",
    }
}

/// The unit holding image `name` of `dir`, if one does.
fn unit_holding(units: &[Unit; UNITS], dir: &Dir, name: &str) -> Option<usize> {
    units.iter().position(|u| u.image.as_ref().is_some_and(|(_, d, n)| d == dir && n.as_str() == name))
}

/// Core 0's loop in SmartPort mode. It lists the root's block images itself (in this frame, not
/// `main`'s); `autoload` says what goes in unit 1 at power-on, and `note` is a settings problem to show. `settings` is
/// the settings file, for the menu's settings page.
#[allow(clippy::too_many_arguments)]
pub fn run<D, SPI, CS, DC, RST>(
    card: &D,
    autoload: Autoload,
    note: Option<Line>,
    settings: &mut config::Text,
    display: &mut Display<SPI, CS, DC, RST>,
    now_us: impl Fn() -> u64,
) -> !
where
    D: BlockDevice,
    D::Error: Debug,
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    // Safety: the only reference to MAPS.
    let maps = unsafe { &mut *MAPS.0.get() };
    let mut units: [Unit; UNITS] = core::array::from_fn(|_| Unit { image: None, switched: false });
    let mut catalog = match image::scan(card, None, Filter::SmartPort) {
        Ok(c) => c,
        Err(e) => {
            show(display, &["Apple II SmartPort", "SD card error:", e.as_str()], None);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    // Item 0 is the settings page, then the catalog.
    let mut menu = Menu { open: false, settings: false, cursor: 0, top: 0, used_at: 0, note: None, fixed: 1 };
    let mut handled = drive::SP_SEQ.load(Relaxed);
    let (mut commands, mut reads, mut writes, mut errors) = (0u32, 0u32, 0u32, 0u32);
    let mut last = smartport::Last::None;
    let mut last_unit: Option<usize> = None;
    let mut sd_us = 0u64;
    let mut next_display = 0u64;
    let mut last_loops = (0u32, 0u64);
    let mut loop_ns = 0u64;
    let mut cmd = [0u8; 48];
    let mut data_pkt = [0u8; MAX_PACKET];
    let mut data = [0u8; MAX_DATA];
    // Where the last image loaded came from, for the status screen.
    let mut source = Line::new();

    // ---- Power-on: what the settings say goes in unit 1 ----
    // With `remember` on, what the knob last put in each unit goes back, in place of autoload.
    const { assert!(UNITS <= config::LAST.len()) };
    let mut remembering = false;
    for u in 0..UNITS {
        let restored = remembered(card, settings, u, Filter::SmartPort, &mut catalog);
        remembering |= restored.is_some();
        let result = match restored {
            None | Some(Ok(None)) => continue,
            Some(Ok(Some((dir, file)))) => image::load_blocks(card, &dir, &file, &mut maps[u]).map(|img| (img, dir, file.name)),
            Some(Err(e)) => Err(e),
        };
        match result {
            Ok((img, dir, name)) => {
                info!("Unit {}: {} ({} blocks)", u + 1, img.name.as_str(), img.blocks);
                source = Line::new();
                let _ = write!(source, "Unit {}: {} {}frag", u + 1, img.filesystem, img.fragments);
                units[u].image = Some((img, dir, name));
            }
            Err(e) => {
                menu.show(None, now_us());
                let mut l = Line::new();
                let _ = write!(l, "Unit {}: {}", u + 1, e.as_str());
                menu.note = Some(l);
            }
        }
    }
    let autoload = if remembering { Autoload::Off } else { autoload };
    let dir = catalog.dir().clone();
    let first: Option<Result<(BlockImage, Name), Line>> = match autoload {
        Autoload::Off => None,
        Autoload::First => Some(
            (0..catalog.len())
                .filter_map(|i| catalog.get(i))
                .filter(|c| c.is_image())
                .find_map(|c| image::load_blocks(card, &dir, c, &mut maps[0]).ok().map(|img| (img, c.name.clone())))
                .ok_or(Line::from("no block image in root that loads")),
        ),
        Autoload::Named(name) => Some(
            match (0..catalog.len()).filter_map(|i| catalog.get(i)).find(|c| c.is_image() && c.name.as_str().eq_ignore_ascii_case(name)) {
                Some(c) => image::load_blocks(card, &dir, c, &mut maps[0]).map(|img| (img, c.name.clone())),
                None => {
                    let mut l = Line::new();
                    let _ = write!(l, "{} isn't in the root", name);
                    Err(l)
                }
            },
        ),
    };
    match first {
        Some(Ok((img, name))) => {
            info!("Unit 1: {} ({} blocks)", img.name.as_str(), img.blocks);
            let _ = write!(source, "Unit 1: {} {}frag", img.filesystem, img.fragments);
            units[0].image = Some((img, dir.clone(), name));
        }
        Some(Err(e)) => {
            menu.show(None, now_us());
            let mut l = Line::new();
            let _ = write!(l, "Can't load: {}", e.as_str());
            menu.note = Some(l);
        }
        None if !remembering => menu.show(None, now_us()),
        None => {}
    }
    if let Some(n) = note
        && menu.note.is_none()
    {
        menu.show(None, now_us());
        menu.note = Some(n);
    }
    KNOB_TURNS.store(0, Relaxed);
    KNOB_PRESSES.store(0, Relaxed);

    loop {
        // ---- A transaction from the host ----
        let seq = drive::SP_SEQ.load(Acquire);
        if seq != handled {
            handled = seq;
            // Copy the packets, then make sure core 1 didn't start on another meanwhile (the
            // host gave up on this one): their checksums say whether they came through whole.
            let (cl, dl) = (drive::SP_CMD_LEN.load(Relaxed) as usize, drive::SP_DATA_LEN.load(Relaxed) as usize);
            let (cl, dl) = (cl.min(cmd.len()), dl.min(data_pkt.len()));
            let msg = drive::SP_MSG.get();
            unsafe {
                cmd[..cl].copy_from_slice(&(&(*msg).cmd)[..cl]);
                data_pkt[..dl].copy_from_slice(&(&(*msg).data)[..dl]);
            }
            let whole = drive::SP_SEQ.load(Acquire) == seq
                && smartport::checksum_ok(&cmd[..cl], false)
                && (dl == 0 || smartport::checksum_ok(&data_pkt[..dl], true));
            let mut len = 0;
            if whole {
                let n = if dl > 0 { smartport::decode(&data_pkt[..dl], &mut data) } else { 0 };
                let t0 = now_us();
                let mut disks = Disks { card, maps: &*maps, units: &mut units, ejected: false };
                // Safety: core 1 reads the reply only once SP_REPLIED says so.
                let tx = unsafe { &mut (*msg).tx };
                let answer = smartport::command(&drive::SP_IDS, &cmd[..cl], &data[..n], &mut disks, tx);
                let ejected = disks.ejected;
                len = answer.len;
                if len > 0 {
                    commands += 1;
                    last = answer.last;
                    last_unit = answer.unit;
                    match answer.last {
                        smartport::Last::Read if answer.error == 0 => reads += 1,
                        smartport::Last::Write if answer.error == 0 => writes += 1,
                        _ => {}
                    }
                    if answer.error != 0 && answer.error != smartport::END_OF_CHAIN {
                        errors += 1;
                    }
                    if matches!(answer.last, smartport::Last::Read | smartport::Last::Write) {
                        sd_us = now_us() - t0;
                    }
                }
                if ejected {
                    // The host ejected a disk: offer the menu to put one back.
                    menu.show(None, now_us());
                    next_display = 0;
                }
            }
            drive::SP_TX_LEN.store(len as u32, Relaxed);
            drive::SP_REPLIED.store(seq, Release);
        }

        // ---- Knob ----
        let now = now_us();
        let turns = KNOB_TURNS.swap(0, Relaxed);
        let presses = KNOB_PRESSES.swap(0, Relaxed);
        if turns != 0 || presses != 0 {
            next_display = now;
            if presses != 0 {
                menu.note = None;
            }
            let here = |catalog: &Catalog, units: &[Unit; UNITS]| {
                (0..catalog.len()).find(|&i| catalog.get(i).is_some_and(|c| unit_holding(units, catalog.dir(), c.name.as_str()).is_some()))
            };
            if !menu.open {
                menu.show(here(&catalog, &units), now);
            } else if menu.settings {
                menu.turn(turns, SET_ITEMS, now);
                if presses != 0 {
                    let unit1 = units[0].image.as_ref().map(|(_, d, n)| (d.clone(), n.clone()));
                    match settings_press(menu.cursor, settings, root_image(&unit1), card, catalog.dir(), &mut menu.note) {
                        SettingsPress::Stay => {
                            // Remembering, if just turned on, starts from what's in the units.
                            for (u, unit) in units.iter().enumerate() {
                                let image = unit.image.as_ref().map(|(_, d, n)| (d, n.as_str()));
                                if let Some(note) = remember(card, settings, u, image, catalog.dir()) {
                                    menu.note = Some(note);
                                }
                            }
                        }
                        SettingsPress::Back => menu.leave_settings(0, now),
                        // Each write is saved as it's answered, so there's nothing to wait for.
                        SettingsPress::Restart => reboot(display),
                    }
                }
            } else {
                menu.turn(turns, catalog.len() + menu.fixed, now);
                let item = menu.cursor.saturating_sub(menu.fixed);
                if presses == 0 {
                    // Just turning.
                } else if menu.cursor == 0 {
                    menu.enter_settings(now);
                } else if let Some(next) = catalog.folder(item) {
                    let up = catalog.get(item).is_some_and(|c| c.is_up());
                    let left = Name::from(catalog.dir().path().rsplit('/').next().unwrap_or(""));
                    match next.and_then(|dir| image::scan(card, Some(&dir), Filter::SmartPort)) {
                        Ok(c) => {
                            catalog = c;
                            let at = if up { catalog.find(left.as_str()) } else { None };
                            menu.show(at.or(here(&catalog, &units)), now);
                        }
                        Err(e) => menu.note = Some(e),
                    }
                } else if let Some(file) = catalog.get(item).cloned() {
                    let dir = catalog.dir().clone();
                    match unit_holding(&units, &dir, file.name.as_str()) {
                        Some(u) => {
                            // In a unit: take it out.
                            units[u].image = None;
                            units[u].switched = true;
                            let mut l = Line::new();
                            let _ = write!(l, "Unit {}: {} out", u + 1, file.name.as_str());
                            menu.note = Some(remember(card, settings, u, None, &dir).unwrap_or(l));
                        }
                        None => match units.iter().position(|u| u.image.is_none()) {
                            None => menu.note = Some(Line::from("All units are full: press one's image to take it out")),
                            Some(u) => {
                                show(display, &["Apple II SmartPort", "Loading:", dir.path(), file.name.as_str()], None);
                                match image::load_blocks(card, &dir, &file, &mut maps[u]) {
                                    Ok(img) => {
                                        info!("Unit {}: {} ({} blocks)", u + 1, img.name.as_str(), img.blocks);
                                        source = Line::new();
                                        let _ = write!(source, "Unit {}: {} {}frag", u + 1, img.filesystem, img.fragments);
                                        menu.open = false;
                                        if let Some(note) = remember(card, settings, u, Some((&dir, file.name.as_str())), &dir) {
                                            menu.show(None, now);
                                            menu.note = Some(note);
                                        }
                                        units[u].image = Some((img, dir, file.name.clone()));
                                        units[u].switched = true;
                                    }
                                    Err(e) => {
                                        let mut l = Line::new();
                                        let _ = write!(l, "Can't load: {}", e.as_str());
                                        menu.note = Some(l);
                                    }
                                }
                                KNOB_TURNS.store(0, Relaxed);
                                KNOB_PRESSES.store(0, Relaxed);
                            }
                        },
                    }
                }
            }
        }
        if menu.open && now - menu.used_at >= MENU_TIMEOUT_US {
            menu.open = false;
            next_display = now;
        }

        if now < next_display {
            continue;
        }
        next_display = now + DISPLAY_EVERY_US;

        if menu.open && menu.settings {
            let unit1 = units[0].image.as_ref().map(|(_, d, n)| (d.clone(), n.clone()));
            let rows = settings_rows(settings, Mode::SmartPort, false, root_image(&unit1), &menu.note);
            let mut lines = [""; ROWS];
            for (line, row) in lines.iter_mut().zip(rows.iter()) {
                *line = row.as_str();
            }
            show(display, &lines, Some(1 + menu.cursor));
            continue;
        }
        if menu.open {
            let mut rows: [Line; ROWS] = core::array::from_fn(|_| Line::new());
            match &menu.note {
                Some(note) => {
                    let _ = rows[0].write_str(note.as_str());
                }
                None => {
                    let _ = write!(rows[0], "{:<44.44} {}/{}", catalog.dir().path(), menu.cursor, catalog.len());
                }
            }
            if catalog.len() == 0 {
                let _ = rows[2].write_str("  (no block images here)");
            }
            let shown = menu.top..(menu.top + ROWS - 1).min(catalog.len() + menu.fixed);
            for (row, item) in rows[1..].iter_mut().zip(shown) {
                let Some(i) = item.checked_sub(menu.fixed) else {
                    let _ = row.write_str("  [Settings...]");
                    continue;
                };
                if let Some(entry) = catalog.get(i) {
                    // The unit it's in, if any.
                    let mark = unit_holding(&units, catalog.dir(), entry.name.as_str()).map_or(' ', |u| (b'1' + u as u8) as char);
                    let size = if entry.is_image() { hd20::size_text(entry.size()) } else { Line::from(entry.describe()) };
                    let _ = write!(row, "{} {:<42.42} {:>5}", mark, entry.name.as_str(), size.as_str());
                }
            }
            let mut lines = [""; ROWS];
            for (line, row) in lines.iter_mut().zip(rows.iter()) {
                *line = row.as_str();
            }
            show(display, &lines, Some(1 + menu.cursor - menu.top));
            continue;
        }

        let loops = drive::LOOPS.load(Relaxed);
        let d_loops = u64::from(loops.wrapping_sub(last_loops.0) & 0xFF_FFFF) * 256;
        if d_loops > 0 {
            loop_ns = (now - last_loops.1) * 1000 / d_loops;
        }
        last_loops = (loops, now);
        let selected = drive::SELECTED.swap(false, Relaxed);

        let mut l0 = Line::new();
        let _ = write!(
            l0,
            "SP cmd:{} rd:{} wr:{} err:{} bad:{} drop:{}",
            commands,
            reads,
            writes,
            errors,
            drive::SP_BAD.load(Relaxed),
            drive::SP_ABORTS.load(Relaxed)
        );
        let mut unit_rows: [Line; UNITS] = core::array::from_fn(|_| Line::new());
        for (u, row) in unit_rows.iter_mut().enumerate() {
            let id = drive::SP_IDS[u].load(Relaxed);
            let _ = match id {
                0 => write!(row, "{} --", u + 1),
                id => write!(row, "{} {:02X}", u + 1, id),
            };
            let _ = match &units[u].image {
                Some((img, _, _)) => {
                    let size = hd20::size_text(u64::from(img.blocks) * BLOCK as u64);
                    let kind = if img.blocks <= smartport::FLOPPY_BLOCKS { "3.5" } else { "HD" };
                    let lock = if img.locked { " RO" } else { "" };
                    write!(row, " {:<5} {:>5}{} {}", kind, size.as_str(), lock, img.name.as_str())
                }
                None => row.write_str(" (empty)"),
            };
        }
        let mut l5 = Line::new();
        let _ = write!(l5, "Host:{} Last:{}", if selected { "ACTIVE" } else { "idle" }, last_name(last));
        if let Some(u) = last_unit {
            let _ = write!(l5, " u{}", u + 1);
        }
        let _ = write!(l5, " sd:{}us resets:{}", sd_us, drive::SP_RESETS.load(Relaxed));
        let inp = unsafe { (*rp235x_hal::pac::SIO::ptr()).gpio_in().read().bits() };
        let bit = |pin: u8| (inp >> pin & 1) as u8;
        let mut l6 = Line::new();
        let _ = write!(
            l6,
            "PH3-0:{}{}{}{} /35:{} WR:{} ACK:{} Core1:{}ns",
            bit(drive::PIN_LSTRB),
            bit(drive::PIN_CA2),
            bit(drive::PIN_CA1),
            bit(drive::PIN_CA0),
            bit(drive::PIN_EN35),
            bit(drive::PIN_WR),
            bit(drive::PIN_WRPROT),
            loop_ns
        );
        let mut lines = [l0.as_str(), "", "", "", "", l5.as_str(), l6.as_str(), source.as_str()];
        for (u, row) in unit_rows.iter().enumerate() {
            lines[1 + u] = row.as_str();
        }
        show(display, &lines, None);
    }
}
