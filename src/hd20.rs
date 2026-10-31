//! HD20 mode (`mode = hd20` in the settings file): core 0's side. Core 1 runs
//! `drive::run_dcd`, which captures the Mac's commands and sends the replies; this decodes the
//! commands (`dcd.rs`), reads and writes the image's blocks, and runs the knob menu and status
//! screen.
//!
//! The image is an HFS volume or a disk image with an Apple partition map (its first `Apple_HFS`
//! partition is served), of any size: [`image::load_hd`] maps it through its cluster chain.
//!
//! The Mac only looks for a DCD when it starts up, and keeps what it learned about the volume
//! until it restarts. So a disk chosen with the knob while the Mac is using another one isn't
//! served under it: the drive stops answering commands until the Mac restarts (core 1 sees its
//! probe and sets `drive::DCD_LIVE`).

use core::fmt::{Debug, Write};
use core::sync::atomic::Ordering::{Acquire, Relaxed, Release};

use defmt::info;
use embedded_sdmmc::{Block, BlockDevice};

use crate::config::{self, Autoload, Mode};
use crate::dcd::{self, MAX_ENCODED, MAX_PAYLOAD};
use crate::drive;
use crate::image::{self, BlockMap, Candidate, Catalog, Dir, Filter, HdLoaded, Name};
use crate::ssd1322::Display;
use crate::{BLOCK_MAP, DISPLAY_EVERY_US, KNOB_PRESSES, KNOB_TURNS, Line, MENU_TIMEOUT_US, Menu, ROWS, SET_ITEMS, SettingsPress};
use crate::{cylinder, loaded_index, reboot, remember, remembered, root_image, settings_press, settings_rows, show};

/// The mapped image, as `dcd::Session` sees it: the volume's blocks.
struct Volume<'a, D> {
    card: &'a D,
    map: &'a BlockMap,
    volume: dcd::Volume,
}

impl<D: BlockDevice> dcd::Storage for Volume<'_, D> {
    fn blocks(&self) -> u32 {
        self.volume.blocks
    }

    fn writable(&self) -> bool {
        true
    }

    fn read(&mut self, block: u32, out: &mut [u8; dcd::BLOCK]) -> Result<(), ()> {
        let mut b = [Block::new()];
        cylinder::read_blocks(self.card, self.map, (self.volume.first + block) as usize, &mut b).map_err(|_| ())?;
        out.copy_from_slice(&b[0].contents);
        Ok(())
    }

    fn write(&mut self, block: u32, data: &[u8; dcd::BLOCK]) -> Result<(), ()> {
        let mut b = [Block::new()];
        b[0].contents.copy_from_slice(data);
        cylinder::write_blocks(self.card, self.map, (self.volume.first + block) as usize, &b).map_err(|_| ())
    }
}

/// Size for the menu and status screen (also SmartPort mode's).
pub(crate) fn size_text(bytes: u64) -> Line {
    let mut l = Line::new();
    let _ = match bytes {
        0..1_048_576 => write!(l, "{}K", bytes / 1024),
        _ => write!(l, "{}M", bytes / 1_048_576),
    };
    l
}

/// What the Mac's DCD commands get: each is decoded from the capture queue, carried out on the
/// image and answered through `drive::dcd_queue`. HD20 mode's loop below and, in HD20 + floppy
/// mode, the floppy loop (`main`) both serve the HD20 with one.
pub(crate) struct Server {
    pub session: dcd::Session,
    /// Commands since the image went in, and how many came through garbled.
    pub commands: u32,
    pub garbled: u32,
    /// The last SD access a command or reply took.
    pub sd_us: u64,
    payload: [u8; MAX_PAYLOAD],
    /// A reply built and waiting for a free slot, and the epoch of the command it answers.
    reply: [u8; MAX_ENCODED],
    pending: Option<usize>,
    epoch: u32,
}

impl Server {
    pub fn new() -> Self {
        Self {
            session: dcd::Session::new(),
            commands: 0,
            garbled: 0,
            sd_us: 0,
            payload: [0; MAX_PAYLOAD],
            reply: [0; MAX_ENCODED],
            pending: None,
            epoch: 0,
        }
    }

    /// Another image went in: start over.
    pub fn reset(&mut self) {
        self.session = dcd::Session::new();
        self.commands = 0;
        self.garbled = 0;
        self.pending = None;
    }

    /// Take the capture at the queue's tail (there must be one) as a command, and answer it.
    pub fn command<D: BlockDevice>(&mut self, card: &D, map: &BlockMap, disk: Option<&HdLoaded>, now_us: &impl Fn() -> u64) {
        let tail = drive::CAP_TAIL.load(Relaxed);
        let k = tail as usize % drive::CAPTURE_QUEUE;
        let start = drive::CAP_START[k].load(Relaxed) as usize;
        let words = drive::CAP_WORDS[k].load(Relaxed) as usize;
        let seq = drive::CAP_SEQ[k].load(Relaxed);
        self.epoch = drive::DCD_EPOCH.load(Acquire);
        let cells = |i: usize| drive::RING.word(start + i / 32) >> (31 - i % 32) & 1 != 0;
        let rx = dcd::receive(&cells, words * 32, &mut self.payload);
        let intact = drive::CAP_TOTAL.load(Acquire).wrapping_sub(seq) <= drive::RING_WORDS as u32;
        // Core 1 may take the next command now; it drops this one's replies if it does.
        drive::CAP_TAIL.store(tail.wrapping_add(1), Release);
        self.commands += 1;
        self.pending = None;
        match (rx, disk) {
            (Ok(rx), Some(d)) => {
                let rx = if intact { rx } else { dcd::Received { ok: false, ..rx } };
                if !rx.ok {
                    self.garbled += 1;
                }
                let t0 = now_us();
                let mut volume = Volume { card, map, volume: d.volume };
                let n = self.session.command(&rx, &self.payload[..rx.len], &mut volume, &mut self.reply);
                if n > 0 {
                    self.pending = Some(n);
                    self.sd_us = now_us() - t0;
                }
            }
            (Err(e), _) => {
                // Not even the lengths came through: no reply, and the Mac tries again.
                self.garbled += 1;
                info!("DCD: {}", e);
            }
            (Ok(_), None) => {}
        }
    }

    /// Queue the reply owed (a read's next block once there is room for it).
    pub fn serve<D: BlockDevice>(&mut self, card: &D, map: &BlockMap, disk: Option<&HdLoaded>, now_us: &impl Fn() -> u64) {
        if drive::DCD_EPOCH.load(Acquire) != self.epoch {
            // The Mac moved on (or deselected the drive): whatever is owed is moot.
            self.pending = None;
            self.session.abandon();
        }
        if self.pending.is_none()
            && self.session.reading()
            && let Some(d) = disk
        {
            let t0 = now_us();
            let mut volume = Volume { card, map, volume: d.volume };
            self.pending = self.session.next_reply(&mut volume, &mut self.reply);
            self.sd_us = now_us() - t0;
        }
        if let Some(n) = self.pending
            && drive::dcd_slot_free()
        {
            let reply = &self.reply;
            drive::dcd_queue(self.epoch, |slot| {
                slot[..n].copy_from_slice(&reply[..n]);
                n
            });
            self.pending = None;
        }
    }
}

fn last_name(last: dcd::Last) -> &'static str {
    match last {
        dcd::Last::None => "-",
        dcd::Last::Read => "READ",
        dcd::Last::Write => "WRITE",
        dcd::Last::Status => "STATUS",
        dcd::Last::Other(0x19) => "FORMAT",
        dcd::Last::Other(0x1A) => "VERIFY",
        dcd::Last::Other(_) => "OTHER",
        dcd::Last::Garbled => "GARBLED",
    }
}

/// Core 0's loop in HD20 mode. It lists the root's hard disk images itself (in this frame, not
/// `main`'s); `autoload` says what is served at power-on, and `note` is a settings problem to show. `settings` is the
/// settings file, for the menu's settings page.
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
    // Safety: the only reference to BLOCK_MAP; floppy mode isn't running.
    let map = unsafe { &mut *BLOCK_MAP.0.get() };
    let mut catalog = match image::scan(card, None, Filter::HardDisk) {
        Ok(c) => c,
        Err(e) => {
            show(display, &["Mac floppy emulator: HD20", "SD card error:", e.as_str()], None);
            loop {
                cortex_m::asm::wfi();
            }
        }
    };
    // Item 0 is the settings page, then the catalog.
    let mut menu = Menu { open: false, settings: false, cursor: 0, top: 0, used_at: 0, note: None, fixed: 1 };
    let mut disk: Option<HdLoaded> = None;
    let mut current: Option<(Dir, Name)> = None;
    let mut source = Line::new();
    let mut server = Server::new();
    let mut next_display = 0u64;
    let mut last_loops = (0u32, 0u64);
    let mut loop_ns = 0u64;

    // ---- Power-on: what the settings say ----
    // With `remember` on, what the knob last served goes back, in place of autoload.
    let restored = remembered(card, settings, 0, Filter::HardDisk, &mut catalog);
    let autoload = if restored.is_some() { Autoload::Off } else { autoload };
    let first = match autoload {
        Autoload::Off => None,
        Autoload::First => Some(Err(Line::from("no disk image in root"))),
        Autoload::Named(name) => Some(
            (0..catalog.len())
                .filter_map(|i| catalog.get(i))
                .find(|c| c.is_image() && c.name.as_str().eq_ignore_ascii_case(name))
                .cloned()
                .ok_or_else(|| {
                    let mut l = Line::new();
                    let _ = write!(l, "{} isn't in the root", name);
                    l
                }),
        ),
    };
    let dir = catalog.dir().clone();
    match first {
        None => match restored {
            Some(Ok(Some((dir, file)))) => {
                let result = insert(card, &dir, &file, map, false, display);
                disk = settle(result, &dir, &file, &catalog, &mut current, &mut source, &mut menu, now_us());
            }
            Some(Err(e)) => {
                menu.show(None, now_us());
                menu.note = Some(e);
            }
            _ => menu.show(None, now_us()),
        },
        Some(Ok(file)) => {
            let result = insert(card, &dir, &file, map, false, display);
            disk = settle(result, &dir, &file, &catalog, &mut current, &mut source, &mut menu, now_us());
        }
        Some(Err(e)) if autoload == Autoload::First => {
            // The first image in the root that loads.
            let mut tried = None;
            for i in 0..catalog.len() {
                let Some(file) = catalog.get(i).filter(|c| c.is_image()) else { continue };
                let result = insert(card, &dir, file, map, false, display);
                let ok = result.is_ok();
                tried = Some((file.clone(), result));
                if ok {
                    break;
                }
            }
            match tried {
                Some((file, result)) => disk = settle(result, &dir, &file, &catalog, &mut current, &mut source, &mut menu, now_us()),
                None => {
                    menu.show(None, now_us());
                    menu.note = Some(e);
                }
            }
        }
        Some(Err(e)) => {
            menu.show(None, now_us());
            menu.note = Some(e);
        }
    }
    if let Some(n) = note
        && menu.note.is_none()
    {
        menu.show(loaded_index(&catalog, &current), now_us());
        menu.note = Some(n);
    }
    let mut probes_at_load = drive::DCD_PROBES.load(Relaxed);
    KNOB_TURNS.store(0, Relaxed);
    KNOB_PRESSES.store(0, Relaxed);

    loop {
        // ---- A command from the Mac, and the replies it's owed ----
        let tail = drive::CAP_TAIL.load(Relaxed);
        if tail != drive::CAP_HEAD.load(Acquire) {
            server.command(card, map, disk.as_ref(), &now_us);
        }
        server.serve(card, map, disk.as_ref(), &now_us);

        // ---- Knob ----
        let now = now_us();
        let turns = KNOB_TURNS.swap(0, Relaxed);
        let presses = KNOB_PRESSES.swap(0, Relaxed);
        if turns != 0 || presses != 0 {
            next_display = now;
            if presses != 0 {
                menu.note = None;
            }
            if !menu.open {
                menu.show(loaded_index(&catalog, &current), now);
            } else if menu.settings {
                menu.turn(turns, SET_ITEMS, now);
                if presses != 0 {
                    let here = root_image(&current);
                    match settings_press(menu.cursor, settings, here, card, catalog.dir(), &mut menu.note) {
                        SettingsPress::Stay => {
                            // Remembering, if just turned on, starts from what's served.
                            let image = current.as_ref().filter(|_| disk.is_some()).map(|(d, n)| (d, n.as_str()));
                            if let Some(note) = remember(card, settings, 0, image, catalog.dir()) {
                                menu.note = Some(note);
                            }
                        }
                        SettingsPress::Back => menu.leave_settings(0, now),
                        // Each command is saved as it's answered, so there's nothing to wait for.
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
                    match next.and_then(|dir| image::scan(card, Some(&dir), Filter::HardDisk)) {
                        Ok(c) => {
                            catalog = c;
                            let at = if up { catalog.find(left.as_str()) } else { None };
                            menu.show(at.or(loaded_index(&catalog, &current)), now);
                        }
                        Err(e) => menu.note = Some(e),
                    }
                } else if Some(item) == loaded_index(&catalog, &current) {
                    menu.open = false;
                } else if let Some(file) = catalog.get(item).cloned() {
                    menu.open = false;
                    let dir = catalog.dir().clone();
                    let result = insert(card, &dir, &file, map, server.commands > 0, display);
                    disk = settle(result, &dir, &file, &catalog, &mut current, &mut source, &mut menu, now);
                    if disk.is_some()
                        && let Some(note) = remember(card, settings, 0, Some((&dir, file.name.as_str())), &dir)
                    {
                        menu.show(loaded_index(&catalog, &current), now);
                        menu.note = Some(note);
                    }
                    server.reset();
                    probes_at_load = drive::DCD_PROBES.load(Relaxed);
                    KNOB_TURNS.store(0, Relaxed);
                    KNOB_PRESSES.store(0, Relaxed);
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
            let rows = settings_rows(settings, Mode::Hd20, false, root_image(&current), &menu.note);
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
            let here = loaded_index(&catalog, &current);
            if catalog.len() == 0 {
                let _ = rows[2].write_str("  (no hard disk images here)");
            }
            let shown = menu.top..(menu.top + ROWS - 1).min(catalog.len() + menu.fixed);
            for (row, item) in rows[1..].iter_mut().zip(shown) {
                let Some(i) = item.checked_sub(menu.fixed) else {
                    let _ = row.write_str("  [Settings...]");
                    continue;
                };
                if let Some(entry) = catalog.get(i) {
                    let mark = if Some(i) == here { '*' } else { ' ' };
                    let size = if entry.is_image() { size_text(entry.size()) } else { Line::from(entry.describe()) };
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
        let live = drive::DCD_LIVE.load(Relaxed);
        let probes = drive::DCD_PROBES.load(Relaxed).wrapping_sub(probes_at_load);
        let selected = drive::SELECTED.swap(false, Relaxed);

        let mut l0 = Line::new();
        let _ = write!(
            l0,
            "HD20 RW cmd:{} rd:{} wr:{} err:{} bad:{}",
            server.commands, server.session.blocks_read, server.session.blocks_written, server.session.errors, server.garbled
        );
        let mut l1 = Line::new();
        match (&disk, live) {
            (Some(d), false) => {
                let _ = write!(l1, "Restart the Mac to use {}", d.name.as_str());
            }
            (Some(_), true) if server.commands == 0 && probes == 0 && (now / 2_000_000) % 2 == 1 => {
                let _ = l1.write_str("Waiting for the Mac: restart it if it's on");
            }
            (Some(d), true) => {
                let _ = l1.write_str(d.name.as_str());
            }
            (None, _) => {
                let _ = l1.write_str("No disk: press the knob to choose one");
            }
        }
        let mut l3 = Line::new();
        let _ = write!(
            l3,
            "Mac:{} Last:{} Probes:{} sd:{}us",
            if selected { "ACTIVE" } else { "idle" },
            last_name(server.session.last),
            probes,
            server.sd_us
        );
        let mut l5 = Line::new();
        let _ = write!(l5, "Core1 loop:{}ns", loop_ns);
        let inp = unsafe { (*rp235x_hal::pac::SIO::ptr()).gpio_in().read().bits() };
        let bit = |pin: u8| (inp >> pin & 1) as u8;
        let mut l6 = Line::new();
        let _ = write!(
            l6,
            "In CA2-0:{}{}{} LSTRB:{} /EN:{} /WRQ:{} WR:{}",
            bit(drive::PIN_CA2),
            bit(drive::PIN_CA1),
            bit(drive::PIN_CA0),
            bit(drive::PIN_LSTRB),
            bit(drive::PIN_ENBL),
            bit(drive::PIN_WRREQ),
            bit(drive::PIN_WR)
        );
        show(display, &[l0.as_str(), l1.as_str(), source.as_str(), l3.as_str(), "", l5.as_str(), l6.as_str()], None);
    }
}

/// Load hard disk image `file` from `dir` and serve it. `used`: the Mac has used the image it
/// replaces, so the drive stops answering until the Mac restarts.
pub(crate) fn insert<D, SPI, CS, DC, RST>(
    card: &D,
    dir: &Dir,
    file: &Candidate,
    map: &mut BlockMap,
    used: bool,
    display: &mut Display<SPI, CS, DC, RST>,
) -> Result<HdLoaded, Line>
where
    D: BlockDevice,
    D::Error: Debug,
    SPI: embedded_hal::spi::SpiBus,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
    RST: embedded_hal::digital::OutputPin,
{
    if used {
        drive::DCD_LIVE.store(false, Relaxed);
    }
    // Nothing is served while the map is rebuilt.
    drive::DCD_PRESENT.store(false, Relaxed);
    show(display, &["Mac floppy emulator: HD20", "Loading:", dir.path(), file.name.as_str()], None);
    let loaded = image::load_hd(card, dir, file, map)?;
    info!(
        "Loaded {} ({} {}, {} blocks from {})",
        loaded.name.as_str(),
        loaded.volume.kind,
        loaded.filesystem,
        loaded.volume.blocks,
        loaded.volume.first
    );
    drive::DCD_PRESENT.store(true, Relaxed);
    if !used {
        drive::DCD_LIVE.store(true, Relaxed);
    }
    Ok(loaded)
}

/// After [`insert`]: note what is in (`current`, the status screen's `source` line), or open the
/// menu on the image that failed, with the reason.
#[allow(clippy::too_many_arguments)]
fn settle(
    result: Result<HdLoaded, Line>,
    dir: &Dir,
    file: &Candidate,
    catalog: &Catalog,
    current: &mut Option<(Dir, Name)>,
    source: &mut Line,
    menu: &mut Menu,
    now: u64,
) -> Option<HdLoaded> {
    *source = Line::new();
    match result {
        Ok(d) => {
            *current = Some((dir.clone(), file.name.clone()));
            let size = size_text(u64::from(d.volume.blocks) * dcd::BLOCK as u64);
            let _ = write!(source, "{} {} {} {}frag", d.volume.kind, size.as_str(), d.filesystem, d.fragments);
            Some(d)
        }
        Err(e) => {
            *current = None;
            let _ = source.write_str("No disk");
            info!("Can't load {}: {}", file.name.as_str(), e.as_str());
            menu.show(catalog.find(file.name.as_str()).filter(|_| dir == catalog.dir()), now);
            let mut note = Line::new();
            let _ = write!(note, "Can't load: {}", e.as_str());
            menu.note = Some(note);
            None
        }
    }
}
