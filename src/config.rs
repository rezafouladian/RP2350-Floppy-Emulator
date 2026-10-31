//! Settings from [`FILE_NAME`] in the card's root, read once at power-on. Without the file every
//! setting has its default.
//!
//! The file is plain text, one `key = value` per line. Keys are case-insensitive; blank lines and
//! lines starting with `#` or `;` are ignored. Known keys:
//!
//! - `autoload`: what goes in the drive at power-on.
//!   - `off` (also `no`, `false`, `none`; the default): nothing, and the menu opens.
//!   - `first` (also `on`, `yes`, `true`): the first image in the root (by name) that loads.
//!   - anything else: the image of that name in the root (case-insensitive).
//! - `mode`: what the emulator is to the computer.
//!   - `floppy` (also `sony`; the default): a SuperDrive.
//!   - `hd20` (also `hd`, `dcd`): a Hard Disk 20, serving an HFS volume or disk image of any
//!     size. The Mac only looks for one when it starts up, and needs HD20 support in ROM (512Ke,
//!     Plus, SE, Classic, Portable, IIci, LC…) or the Hard Disk 20 file (512K).
//!   - `hd20+floppy` (also `hd20 floppy`, `hd20floppy`, `chain`): both, as an HD20 with a floppy
//!     drive (a SuperDrive) on its own port: each image goes to its drive.
//!   - `apple2` (also `a2`, `appleii`, `disk2`, `5.25`): a Disk II 5.25" drive for an Apple II,
//!     serving DOS 3.3/ProDOS sector images, `.nib`, `.2mg` and WOZ images.
//!   - `apple35` (also `a35`, `apple 3.5`, `iigs`, `3.5`): an Apple 3.5 Drive for an Apple IIgs
//!     or IIc+, serving 400K/800K block images, DiskCopy 4.2, `.2mg`, MOOF and 3.5" WOZ images.
//!   - `apple2dual` (also `dual`, `a2dual`, `apple2 dual`, `5.25+3.5`): both of those at once, on
//!     one port: each image goes to its drive.
//!   - `smartport` (also `sp`, `smart port`, `unidisk`): SmartPort units for an Apple IIc, IIgs
//!     or IIc+ (or a IIe with a SmartPort card): up to 4 block images, 800K ones as 3.5" drives
//!     (as a UniDisk 3.5), bigger ones as hard disks.
//! - `keep_rotation`: where a Mac or Apple 3.5 drive's track starts after a step or side switch.
//!   - `on` (also `yes`, `true`, `keep`; the default): at the same angle as the one before, as
//!     on a real drive.
//!   - `off` (also `no`, `false`): at its first sector.
//! - `remember`: whether what the knob puts in a drive (or SmartPort unit) is put back at the next
//!   power-on, in place of `autoload`.
//!   - `off` (also `no`, `false`; the default).
//!   - `on` (also `yes`, `true`): the emulator keeps it in [`LAST`]'s lines as it changes.
//! - `last`, `last2`, `last3`, `last4` ([`LAST`]): with `remember` on, the image in drive (or
//!   unit) 1–4 as a path from the root (`/Games/Disk.dsk`), empty for none. Written by the
//!   emulator.
//!
//! Without a file, [`DEFAULT_FILE`] is written at power-on: every setting at its default, with
//! help. The knob's settings menu changes values through [`Text::set`], which keeps the rest of
//! the file as it was.
//!
//! Hardware-free, so the host tests compile it directly.

/// The settings file, looked for in the root only (case-insensitive).
pub const FILE_NAME: &str = "settings.txt";
/// Bytes of the file read; the rest is ignored.
pub const MAX_BYTES: usize = 4_096;

pub const MODE: &str = "mode";
pub const AUTOLOAD: &str = "autoload";
pub const KEEP_ROTATION: &str = "keep_rotation";
pub const REMEMBER: &str = "remember";
/// The remembered image of each drive (dual mode: 0 the 3.5" drive, 1 the 5.25") or SmartPort
/// unit.
pub const LAST: [&str; 4] = ["last", "last2", "last3", "last4"];

/// What is written when the card has no settings file: the defaults, and what they mean.
pub const DEFAULT_FILE: &str = "\
# rp2350-floppy-emulator settings, read once at power-on.
# One \"key = value\" per line; case doesn't matter. Lines starting with
# # or ; are comments. The knob's Settings menu changes these values too,
# and leaves the rest of this file as it is.

# mode: what the emulator is to the computer. A change takes effect at the
# next power-on (or the menu's Restart).
#   floppy  a Macintosh floppy drive (a SuperDrive): 400K, 800K, 720K and
#           1.44 MB disks as raw, DiskCopy 4.2 or MOOF images
#   hd20    a Hard Disk 20 on the Mac's floppy port, serving an HFS volume
#           or disk image of any size. The Mac only looks for it when it
#           starts up, and needs HD20 support in ROM (512Ke, Plus, SE,
#           Classic, Portable, IIci, LC...) or the Hard Disk 20 file (512K)
#   hd20+floppy  both: an HD20 with a floppy drive behind it, on the Mac's
#           external port. Each image you choose goes to its own drive
#   apple2  a Disk II 5.25\" drive for an Apple II: .dsk, .do, .po, .nib,
#           .2mg and .woz images
#   apple35 an Apple 3.5 Drive for an Apple IIgs or IIc+: 400K and 800K
#           .po, .dsk, .2mg, DiskCopy 4.2, .moof and .woz images
#   apple2dual  both: an Apple 3.5 Drive and a Disk II on one port (IIgs,
#           IIc+). Each image you choose goes to its own drive
#   smartport  SmartPort units for an Apple IIc, IIgs or IIc+: up to 4
#           .po, .hdv or .2mg images, 800K ones as 3.5\" drives, bigger
#           ones as hard disks (up to 32 MB)
mode = floppy

# autoload: what goes in the drive at power-on.
#   off     nothing: the menu opens to choose an image
#   first   the first image in the root folder (by name) that loads
#   a name  that image in the root folder, e.g. autoload = System Tools.dsk
autoload = off

# keep_rotation: where a 3.5\" drive's track starts after a step or side
# switch (floppy, apple35, apple2dual; a Disk II always keeps it).
#   on      where the disk has turned to, as on a real drive
#   off     at its first sector
# Protected MOOFs imaged with cross-track sync keep it either way.
keep_rotation = on

# remember: put back at power-on what the knob put in the drive (or the
# SmartPort units), in place of autoload. The emulator keeps it in lines
# below this one: last (drive or unit 1), last2, last3, last4.
#   off     no
#   on      yes; a disk the computer ejects is still put back
remember = off
";

/// What goes in the drive at power-on.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Autoload<'a> {
    Off,
    /// The first image in the root that loads.
    First,
    /// The image with this name in the root.
    Named(&'a str),
}

impl Autoload<'_> {
    /// The value that says this in the file.
    pub fn value(&self) -> &str {
        match self {
            Autoload::Off => "off",
            Autoload::First => "first",
            Autoload::Named(name) => name,
        }
    }
}

/// What the emulator presents to the computer.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    Floppy,
    Hd20,
    /// An HD20 with a floppy drive chained behind it.
    Hd20Floppy,
    Apple2,
    /// An Apple 3.5 Drive (Apple II 3.5").
    Apple35,
    /// An Apple 3.5 Drive and a Disk II on one port.
    Dual,
    /// SmartPort units (3.5" disks and hard disks).
    SmartPort,
}

impl Mode {
    /// The value that says this in the file.
    pub fn value(self) -> &'static str {
        match self {
            Mode::Floppy => "floppy",
            Mode::Hd20 => "hd20",
            Mode::Hd20Floppy => "hd20+floppy",
            Mode::Apple2 => "apple2",
            Mode::Apple35 => "apple35",
            Mode::Dual => "apple2dual",
            Mode::SmartPort => "smartport",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Config<'a> {
    pub autoload: Autoload<'a>,
    pub mode: Mode,
    /// A Mac or Apple 3.5 drive's tracks start at the angle the disk has turned to, not at their
    /// first sector (`drive::KEEP_ROTATION`).
    pub keep_rotation: bool,
    /// Put back at power-on what was last chosen for each drive or unit ([`LAST`]).
    pub remember: bool,
    /// What was, per [`LAST`] key: `Some("")` for nothing, `None` if the file doesn't say.
    pub last: [Option<&'a str>; 4],
    /// The first line (from 1) that wasn't understood, if any. The lines that were still apply.
    pub bad_line: Option<usize>,
}

impl Default for Config<'_> {
    fn default() -> Self {
        Self { autoload: Autoload::Off, mode: Mode::Floppy, keep_rotation: true, remember: false, last: [None; 4], bad_line: None }
    }
}

/// Parse the file's contents.
pub fn parse(text: &[u8]) -> Config<'_> {
    let mut config = Config::default();
    // Editors on Windows (and sometimes macOS) start the file with a UTF-8 byte order mark.
    let text = text.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(text);
    for (n, line) in text.split(|&b| b == b'\n').enumerate() {
        let ok = match core::str::from_utf8(line).map(str::trim) {
            Ok(line) if line.is_empty() || line.starts_with(['#', ';']) => true,
            Ok(line) => set(&mut config, line),
            Err(_) => false,
        };
        if !ok && config.bad_line.is_none() {
            config.bad_line = Some(n + 1);
        }
    }
    config
}

/// Apply one `key = value` line; false if it isn't one this understands.
fn set<'a>(config: &mut Config<'a>, line: &'a str) -> bool {
    let Some((key, value)) = line.split_once('=') else { return false };
    let (key, value) = (key.trim(), value.trim());
    let is = |words: &[&str]| words.iter().any(|w| value.eq_ignore_ascii_case(w));
    if key.eq_ignore_ascii_case(AUTOLOAD) {
        config.autoload = if value.is_empty() || is(&["off", "no", "false", "none"]) {
            Autoload::Off
        } else if is(&["first", "on", "yes", "true"]) {
            Autoload::First
        } else {
            Autoload::Named(value)
        };
        return true;
    }
    if key.eq_ignore_ascii_case(KEEP_ROTATION) {
        config.keep_rotation = if is(&["on", "yes", "true", "keep"]) {
            true
        } else if is(&["off", "no", "false"]) {
            false
        } else {
            return false;
        };
        return true;
    }
    if key.eq_ignore_ascii_case(REMEMBER) {
        config.remember = if is(&["on", "yes", "true"]) {
            true
        } else if is(&["off", "no", "false"]) {
            false
        } else {
            return false;
        };
        return true;
    }
    if let Some(slot) = LAST.iter().position(|k| key.eq_ignore_ascii_case(k)) {
        config.last[slot] = Some(value);
        return true;
    }
    if key.eq_ignore_ascii_case(MODE) {
        config.mode = if is(&["floppy", "sony"]) {
            Mode::Floppy
        } else if is(&["hd20", "hd", "dcd"]) {
            Mode::Hd20
        } else if is(&["hd20+floppy", "hd20 floppy", "hd20floppy", "hd20 + floppy", "chain"]) {
            Mode::Hd20Floppy
        } else if is(&["apple2", "a2", "appleii", "apple ii", "disk2", "diskii", "disk ii", "5.25"]) {
            Mode::Apple2
        } else if is(&["apple35", "a35", "apple3.5", "apple 3.5", "apple ii 3.5", "appleii 3.5", "iigs", "3.5"]) {
            Mode::Apple35
        } else if is(&["apple2dual", "dual", "a2dual", "apple2 dual", "apple ii dual", "5.25+3.5", "3.5+5.25"]) {
            Mode::Dual
        } else if is(&["smartport", "sp", "smart port", "smartport hd", "unidisk", "unidisk 3.5"]) {
            Mode::SmartPort
        } else {
            return false;
        };
        return true;
    }
    false
}

/// The settings file's text, kept so the knob's settings menu can change a value and save the
/// file with everything else in it as it was.
pub struct Text {
    buf: [u8; MAX_BYTES],
    len: usize,
    /// Only part of the file is here (it's longer than [`MAX_BYTES`], or couldn't be read), so
    /// saving this would lose the rest.
    pub partial: bool,
}

impl Text {
    pub const fn new() -> Self {
        Self { buf: [0; MAX_BYTES], len: 0, partial: false }
    }

    /// For reading the file into; then [`Text::read`] says how much came.
    pub fn buf_mut(&mut self) -> &mut [u8; MAX_BYTES] {
        &mut self.buf
    }

    /// `len` bytes were read, of a file of `size`.
    pub fn read(&mut self, len: usize, size: u64) {
        self.len = len.min(MAX_BYTES);
        self.partial = size > self.len as u64;
    }

    /// [`DEFAULT_FILE`], for a card that has none.
    pub fn defaults(&mut self) {
        const { assert!(DEFAULT_FILE.len() <= MAX_BYTES) };
        self.buf[..DEFAULT_FILE.len()].copy_from_slice(DEFAULT_FILE.as_bytes());
        self.len = DEFAULT_FILE.len();
        self.partial = false;
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn config(&self) -> Config<'_> {
        parse(self.as_bytes())
    }

    /// Make `key` say `value`: the last line that sets `key` (the one that counts, whatever its
    /// value was) becomes `key = value`; with none, that line is added at the end. False, and
    /// nothing changed, if the result wouldn't fit.
    pub fn set(&mut self, key: &str, value: &str) -> bool {
        let text = &self.buf[..self.len];
        let body = if text.starts_with(b"\xEF\xBB\xBF") { 3 } else { 0 };
        // The span of the line to replace, without its line ending.
        let mut found = None;
        let mut at = body;
        for line in text[body..].split(|&b| b == b'\n') {
            let content = line.strip_suffix(b"\r").unwrap_or(line);
            let sets_key = core::str::from_utf8(content).is_ok_and(|l| {
                let l = l.trim();
                !l.starts_with(['#', ';']) && l.split_once('=').is_some_and(|(k, _)| k.trim().eq_ignore_ascii_case(key))
            });
            if sets_key {
                found = Some((at, at + content.len()));
            }
            at += line.len() + 1;
        }
        let crlf = text.windows(2).any(|w| w == b"\r\n");
        let newline: &[u8] = if crlf { b"\r\n" } else { b"\n" };
        let (start, end, before, after): (usize, usize, &[u8], &[u8]) = match found {
            Some((start, end)) => (start, end, b"", b""),
            // A last line without a line ending gets one first.
            None if self.len > body && !text.ends_with(b"\n") => (self.len, self.len, newline, newline),
            None => (self.len, self.len, b"", newline),
        };
        let line = [before, key.as_bytes(), b" = ", value.as_bytes(), after];
        let new: usize = line.iter().map(|p| p.len()).sum();
        let len = self.len - (end - start) + new;
        if len > MAX_BYTES {
            return false;
        }
        self.buf.copy_within(end..self.len, start + new);
        let mut at = start;
        for part in line {
            self.buf[at..at + part.len()].copy_from_slice(part);
            at += part.len();
        }
        self.len = len;
        true
    }
}
