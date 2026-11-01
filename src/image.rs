//! Lists the disk images and folders in a directory of the SD card, and maps an image to card
//! sectors. The card may be exFAT or FAT16/FAT32.
//!
//! Supported in floppy mode: raw 400K/800K GCR and 720K/1.44 MB MFM sector images (`.img`/`.dsk`/`.image`),
//! DiskCopy 4.2 images of those (`.dc42`/`.diskcopy`/`.image`/`.img`/`.dsk`; see `dc42.rs`), and 400K/800K GCR or 1.44 MB MFM MOOF images (`.moof`, bitstream or with FLUX
//! tracks, the latter read-only). [`scan`] lists a directory's folders and the files whose name
//! and size fit; whether an image is usable (e.g. not a Twiggy MOOF) is only known once [`load`]
//! has read it.
//!
//! In HD20 mode ([`Filter::HardDisk`]) the images are HFS volumes or Apple-partitioned disk
//! images of any size (`.hda`/`.hfs`/`.img`/`.dsk`/`.image`), served by [`load_hd`]; see `dcd.rs`.
//!
//! In Apple II mode ([`Filter::Apple2`]) they are 5.25" disks: 140K (35-track) or 160K (40-track)
//! sector images in DOS 3.3 or ProDOS order (`.dsk`/`.do`/`.po`), 116K/130K 13-sector (DOS 3.2)
//! images (`.d13`, or `.dsk`/`.do` of those sizes), `.nib` nibble images, `.2mg` wrapping a
//! 16-sector or `.nib` image, and WOZ 1/2 bitstreams (`.woz`); see `a2.rs` and `woz.rs`.
//!
//! In Apple II 3.5" mode ([`Filter::Apple35`]) they are 400K/800K disks for an Apple 3.5 Drive:
//! raw block images (`.po`/`.dsk`/`.img`/`.image`/`.hdv`, ProDOS order, which is the Mac's),
//! DiskCopy 4.2, `.2mg`, GCR MOOFs, and WOZ 2 of 3.5" disks (read as MOOFs, see `moof.rs`).
//! Sector images get the Apple II's format byte (4:1 interleave, `gcr::format`).
//!
//! In dual mode ([`Filter::Dual`]) they are either: each goes to its own drive ([`dual_drive`]).
//!
//! In HD20 + floppy mode ([`Filter::Hd20Floppy`]) they are the Mac floppy images and, of other
//! sizes, the hard disk images: [`Candidate::is_hard_disk`] says which drive one is for.
//!
//! In SmartPort mode ([`Filter::SmartPort`]) they are block images for the units: `.po`, `.hdv`,
//! `.2mg` (ProDOS order) and `.dsk`/`.img`/`.image`/`.hda` of whole blocks, up to 32 MB (a ProDOS
//! volume's limit), mapped through their cluster chains by [`load_blocks`].
//!
//! A [`Dir`] says where a directory is: its path (exFAT opens by path) and, for FAT, the 8.3 name
//! of each folder on the way down, since `embedded-sdmmc` opens by short name.
//!
//! Neither filesystem driver can seek, so the image is read through once when loaded while a
//! [`Recorder`] notes which card sector each 512-byte block came from. After that the main loop
//! reads tracks straight from the card by sector number, the same way for both filesystems. A
//! hard disk image is too big for that: [`load_hd`] follows its cluster chain instead
//! (`clusters.rs`).

use core::cell::{Cell, RefCell};
use core::fmt::{Debug, Write};
use core::ops::ControlFlow;

use embedded_sdmmc::{
    Block, BlockCount, BlockIdx, LfnBuffer, Mode, RawDirectory, RawVolume, ShortFileName, TimeSource,
    Timestamp, VolumeIdx, VolumeManager,
};
use exfat_embedded::{FileSystem as ExfatFs, Scratch};

use crate::{FixedStr, Line};
use crate::a2r::{self, A2r};
use crate::{a2, clusters, config, dc42, dcd, gcr, mfm};
use crate::moof::{self, Crc32, Moof};
use crate::woz::{self, Woz};
use crate::track::{BLOCK, MAX_TRACK_BLOCKS};

pub const IMAGE_400K: usize = 409_600;
pub const IMAGE_800K: usize = 819_200;
pub const IMAGE_720K: usize = 737_280;
pub const IMAGE_1440K: usize = 1_474_560;
/// Most 512-byte blocks a file can have to be mapped: 32 MB, as far as a MOOF's 16-bit block
/// numbers reach. A raw 1.44 MB image is 2,880 blocks, a bitstream MFM MOOF 7,683 (3.9 MB), and a
/// MOOF that is all flux up to ~28,000 (flux tracks take up to ~100 blocks for GCR, ~175 for MFM).
pub const MAX_BLOCKS: usize = 65_536;
/// Runs of consecutive card sectors a mapped file can be in. Files on FAT and exFAT are rarely
/// fragmented; a more fragmented one is refused.
pub const MAX_RUNS: usize = 1_024;

/// Where each 512-byte block of the loaded image is on the card: runs of consecutive card
/// sectors, in file order, up to `N` of them (SmartPort units have smaller maps).
pub struct BlockMap<const N: usize = MAX_RUNS> {
    /// Run `k` starts at file block `start[k]`, on card sector `lba[k]`, and lasts until the next
    /// run (or `blocks`).
    start: [u32; N],
    lba: [u32; N],
    runs: usize,
    blocks: usize,
}

impl<const N: usize> BlockMap<N> {
    pub const fn new() -> Self {
        Self { start: [0; N], lba: [0; N], runs: 0, blocks: 0 }
    }

    fn clear(&mut self) {
        self.runs = 0;
        self.blocks = 0;
    }

    /// Runs the file is stored in.
    pub fn fragments(&self) -> u32 {
        self.runs as u32
    }

    /// The run holding file block `block`.
    fn run_of(&self, block: usize) -> usize {
        debug_assert!(block < self.blocks);
        self.start[..self.runs].partition_point(|&s| s as usize <= block) - 1
    }

    /// The card sector holding file block `block`, and how many blocks from there on are on
    /// consecutive sectors.
    pub fn sectors(&self, block: usize) -> (u32, usize) {
        let k = self.run_of(block);
        let end = if k + 1 < self.runs { self.start[k + 1] as usize } else { self.blocks };
        (self.lba[k] + (block - self.start[k] as usize) as u32, end - block)
    }

    /// Append the next `count` file blocks, found on consecutive card sectors from `lba`.
    fn push_run(&mut self, lba: u32, count: u32) -> Result<(), &'static str> {
        let k = self.runs;
        let extends = k > 0 && lba == self.lba[k - 1].wrapping_add((self.blocks - self.start[k - 1] as usize) as u32);
        if !extends {
            if k == N {
                return Err("image too fragmented");
            }
            self.start[k] = self.blocks as u32;
            self.lba[k] = lba;
            self.runs += 1;
        }
        self.blocks += count as usize;
        Ok(())
    }

    /// Append the next file block, found on card sector `lba`.
    fn push(&mut self, lba: u32) -> Result<(), Line> {
        let i = self.blocks;
        let k = self.runs;
        let extends = k > 0 && lba == self.lba[k - 1].wrapping_add((i - self.start[k - 1] as usize) as u32);
        if !extends {
            if k == N {
                return Err(Line::from("image too fragmented"));
            }
            self.start[k] = i as u32;
            self.lba[k] = lba;
            self.runs += 1;
        }
        self.blocks += 1;
        Ok(())
    }
}

/// Entries (folders and images) listed per directory; any more are left out.
pub const MAX_IMAGES: usize = 64;
/// Folder levels below the root the menu can go.
pub const MAX_DEPTH: usize = 8;

/// A file or folder name: exFAT and FAT long names are at most 255 characters (ASCII here; FAT
/// long names have anything else replaced by `?`).
pub type Name = FixedStr<256>;
/// A path from the root, at most `MAX_DEPTH` folders and a file.
type Path = FixedStr<1024>;

/// The disk an image describes. For sector images, `offset` is the bytes before the sector data
/// in the file: 0 for a raw image, [`dc42::HEADER_BYTES`] for DiskCopy 4.2, a `.2mg`'s data
/// offset.
pub enum Disk {
    /// GCR sectors, 1 = 400K or 2 = 800K sides; encoded on the fly with `format` in the address
    /// fields (`gcr::format`: the Mac's 2:1 interleave, or the Apple II's 4:1). `diskcopy`: a
    /// DiskCopy 4.2 image, whose checksum is redone after saving. `locked`: a `.2mg` marked
    /// write-protected.
    Gcr { sides: u8, offset: u16, format: u8, diskcopy: bool, locked: bool },
    /// 720K or 1.44 MB MFM sectors; encoded on the fly.
    Mfm { format: mfm::Format, offset: u16 },
    /// Bitstream, played as stored.
    Moof(Moof),
    /// Apple II 5.25" sectors, 16 per track (13 in `a2::Order::Dos32`): `tracks` (35 or 40) of
    /// them in `order`, from byte `offset` (a `.2mg` header). `volume` goes in the address fields. `locked`: a `.2mg` marked
    /// write-protected.
    A2 { order: a2::Order, tracks: u8, volume: u8, offset: u16, locked: bool },
    /// Apple II nibbles, 6,656 per track, from byte `offset`. Played read-only.
    Nib { tracks: u8, offset: u16 },
    /// Apple II bitstream, played as stored.
    Woz(Woz),
    /// Apple II flux captures, a revolution of each quarter track found as it's read. Read-only.
    A2r(A2r),
}

impl Disk {
    /// A MOOF imaged with cross-track sync: its tracks keep their angle across steps (see
    /// `drive::KEEP_ROTATION`).
    pub fn synchronized(&self) -> bool {
        matches!(self, Disk::Moof(m) if m.synchronized)
            || matches!(self, Disk::Woz(w) if w.synchronized)
            || matches!(self, Disk::A2r(r) if r.synchronized)
    }

    /// A DiskCopy 4.2 image: sectors after its header, and a data checksum to redo after saving.
    pub fn diskcopy(&self) -> bool {
        matches!(self, Disk::Gcr { diskcopy: true, .. }) || matches!(self, Disk::Mfm { offset, .. } if *offset != 0)
    }

    /// An Apple II 5.25" disk, for `drive::run_a2`.
    pub fn is_apple2(&self) -> bool {
        matches!(self, Disk::A2 { .. } | Disk::Nib { .. } | Disk::Woz(_) | Disk::A2r(_))
    }

    /// For an Apple II disk: what is heard with the head at quarter track `quarter`, as a
    /// `Cylinder` track key: a track of a sector or `.nib` image, a WOZ TRK record, an A2R's
    /// capture (each quarter track its own), or [`a2::BLANK`].
    pub fn a2_track(&self, quarter: u8) -> u8 {
        match self {
            Disk::A2 { tracks, .. } | Disk::Nib { tracks, .. } => a2::quarter_track(quarter, *tracks),
            Disk::Woz(w) => w.track(quarter),
            Disk::A2r(r) => r.capture(quarter).map_or(a2::BLANK, |_| quarter),
            _ => a2::BLANK,
        }
    }

    /// The PIO clock for this disk's cells, 16 PIO cycles per cell: the Mac's 2.0425 µs GCR cell,
    /// a 1 µs MFM half-cell, the Apple II's 4 µs for `.nib` and an A2R (whose tracks then play at
    /// their own), a WOZ's from its tracks' length (`Woz::cell_hz`), and a sector image's the
    /// Apple II's write cell, 3.911 µs.
    pub fn cell_hz(&self) -> u32 {
        match self {
            Disk::Woz(w) => w.cell_hz(),
            Disk::A2 { .. } => crate::drive::A2_SECTOR_PIO_HZ,
            Disk::Nib { .. } | Disk::A2r(_) => crate::drive::A2_PIO_HZ,
            _ if self.is_mfm() => crate::drive::MFM_PIO_HZ,
            _ => crate::drive::GCR_PIO_HZ,
        }
    }

    /// HD media (R15 = 0): 1.44 MB, raw or MOOF. 720K MFM is DD media, like GCR.
    pub fn is_hd(&self) -> bool {
        match self {
            Disk::Mfm { format, .. } => *format == mfm::Format::Hd,
            Disk::Moof(m) => m.mfm,
            _ => false,
        }
    }

    /// MFM (720K or 1.44 MB), played at 1 µs half-cells and written through the MFM separator.
    pub fn is_mfm(&self) -> bool {
        match self {
            Disk::Mfm { .. } => true,
            Disk::Moof(m) => m.mfm,
            _ => false,
        }
    }

    /// Accepts writes: sector images always (DiskCopy too: its checksum is redone after saving,
    /// see `Cylinder::checksum_step`), MOOFs unless INFO says write-protected or some tracks are
    /// flux.
    pub fn writable(&self) -> bool {
        match self {
            Disk::Moof(m) => !m.write_protected && !m.has_flux,
            Disk::Gcr { locked, .. } => !locked,
            Disk::Mfm { .. } => true,
            Disk::A2 { locked, .. } => !locked,
            Disk::Nib { .. } | Disk::A2r(_) => false,
            Disk::Woz(w) => w.writable(),
        }
    }

    pub fn describe(&self) -> &'static str {
        match self {
            Disk::Gcr { sides: 1, offset: 0, .. } => "400K GCR",
            Disk::Gcr { offset: 0, .. } => "800K GCR",
            Disk::Mfm { format: mfm::Format::Dd, offset: 0 } => "720K MFM",
            Disk::Mfm { offset: 0, .. } => "1.44M MFM",
            Disk::Gcr { sides: 1, diskcopy: true, .. } => "DC42 400K",
            Disk::Gcr { diskcopy: true, .. } => "DC42 800K",
            Disk::Gcr { sides: 1, .. } => "2MG 400K",
            Disk::Gcr { .. } => "2MG 800K",
            Disk::Mfm { format: mfm::Format::Dd, .. } => "DC42 720K",
            Disk::Mfm { .. } => "DC42 1.44M",
            Disk::Moof(m) if m.woz => match (m.sides, m.has_flux) {
                (1, false) => "WOZ 400K",
                (1, true) => "WOZ 400K flux",
                (_, false) => "WOZ 800K",
                (_, true) => "WOZ 800K flux",
            },
            Disk::Moof(m) => match (m.mfm, m.sides, m.has_flux) {
                (true, _, false) => "MOOF 1.44M",
                (true, _, true) => "MOOF 1.44M flux",
                (false, 1, false) => "MOOF 400K",
                (false, 1, true) => "MOOF 400K flux",
                (false, _, false) => "MOOF 800K",
                (false, _, true) => "MOOF 800K flux",
            },
            Disk::A2 { order: a2::Order::Dos32, tracks: 35, .. } => "116K DOS 3.2",
            Disk::A2 { order: a2::Order::Dos32, .. } => "130K DOS 3.2",
            Disk::A2 { order: a2::Order::Dos, tracks: 35, .. } => "140K DOS order",
            Disk::A2 { order: a2::Order::Dos, .. } => "160K DOS order",
            Disk::A2 { tracks: 35, .. } => "140K ProDOS order",
            Disk::A2 { .. } => "160K ProDOS order",
            Disk::Nib { .. } => "NIB",
            Disk::Woz(w) => match (w.version, w.has_flux) {
                (1, _) => "WOZ1",
                (_, false) => "WOZ2",
                (_, true) => "WOZ2.1 flux",
            },
            Disk::A2r(r) => match r.version {
                1 => "A2R1 flux",
                2 => "A2R2 flux",
                _ => "A2R3 flux",
            },
        }
    }
}

/// Which images a catalog lists: the emulator's mode.
#[derive(Clone, Copy, PartialEq)]
pub enum Filter {
    Floppy,
    HardDisk,
    /// Apple II 5.25" disks.
    Apple2,
    /// Apple II 3.5" disks (400K/800K GCR).
    Apple35,
    /// Both of the Apple II's, for its two drives.
    Dual,
    /// Block images for SmartPort units.
    SmartPort,
    /// Both of HD20 + floppy mode's: Mac floppy images, and hard disk images of other sizes.
    Hd20Floppy,
}

/// What a directory entry is, from its name and size, before it is read.
#[derive(Clone, Copy, PartialEq)]
enum Kind {
    /// `..`: back to the parent directory.
    Up,
    Folder,
    /// Sectors: `data` bytes of them, after a DiskCopy 4.2 header if `diskcopy`.
    Sectors { data: usize, diskcopy: bool },
    Moof,
    /// Apple II sectors: 35 or 40 tracks, in ProDOS order (`Some(true)`), DOS order
    /// (`Some(false)`), or to be worked out from the contents (`.dsk`).
    A2 { tracks: u8, prodos: Option<bool> },
    /// Apple II 13-sector (DOS 3.2) sectors, 35 or 40 tracks.
    D13 { tracks: u8 },
    /// Apple II nibbles, 35 or 40 tracks.
    Nib { tracks: u8 },
    Woz,
    /// A WOZ 2 of a 3.5" disk, read as a MOOF.
    Woz35,
    /// Applesauce flux captures (`a2r.rs`).
    A2r,
    /// A `.2mg`: sectors or nibbles after a header that says which.
    TwoImg,
    /// A hard disk image or volume for HD20 mode, whole blocks.
    HardDisk,
    /// A block image for a SmartPort unit: whole blocks, or a `.2mg`'s.
    Blocks,
    /// The settings file ([`config::FILE_NAME`]).
    Settings,
}

/// Most blocks a SmartPort unit serves: 65,535, a ProDOS volume's limit, which is also as many as
/// ProDOS reads from a unit's status.
const MAX_SP_BLOCKS: u32 = 65_535;
/// Most bytes of a SmartPort unit's file: exactly 32 MB, a block more than it serves. 32 MB images
/// are often made that size (GS/OS 6.0.2's `smart0.po`), their volume 65,535 blocks of it.
const MAX_SP_BYTES: u64 = (MAX_SP_BLOCKS as u64 + 1) * BLOCK as u64;
/// Most bytes a hard disk image can have on FAT: FAT32's file size limit (4 GB), in whole blocks.
const MAX_FAT_HD_BYTES: u64 = 0xFFFF_FE00;
/// On exFAT: as far as the DCD's 3-byte block numbers reach (8 GB). An Apple-partitioned image
/// can only be that big if its HFS partition is smaller; System 6 and 7.1 stop at 2 GB a volume.
const MAX_EXFAT_HD_BYTES: u64 = dcd::MAX_BLOCKS as u64 * BLOCK as u64;

impl Kind {
    fn of(name: &str, size: u64, filter: Filter, exfat: bool) -> Option<Self> {
        if name.starts_with('.') {
            return None; // macOS `._` sidecars, hidden files
        }
        let ext = name.rsplit_once('.').map_or("", |(_, e)| e);
        if filter == Filter::Hd20Floppy {
            // A floppy's size and name say it's one; hard disks are the other whole-block images.
            return Kind::of(name, size, Filter::Floppy, exfat).or_else(|| Kind::of(name, size, Filter::HardDisk, exfat));
        }
        if filter == Filter::HardDisk {
            // Whether it holds a volume is only known once read (`load_hd`).
            let hd = ["hda", "hfs", "img", "dsk", "image"].iter().any(|e| ext.eq_ignore_ascii_case(e));
            let max = if exfat { MAX_EXFAT_HD_BYTES } else { MAX_FAT_HD_BYTES };
            let whole = size % BLOCK as u64 == 0 && (3 * BLOCK as u64..=max).contains(&size);
            return (hd && whole).then_some(Kind::HardDisk);
        }
        if filter == Filter::SmartPort {
            let is = |e: &[&str]| e.iter().any(|x| ext.eq_ignore_ascii_case(x));
            let whole = size % BLOCK as u64 == 0 && (3 * BLOCK as u64..=MAX_SP_BYTES).contains(&size);
            // 140K and 160K `.dsk`s are mostly DOS 3.3 order: not blocks.
            let dos = size == a2::TRACK_BYTES as u64 * 35 || size == a2::TRACK_BYTES as u64 * 40;
            return if is(&["2mg", "2img"]) {
                // Its header says where the blocks are; a little more for its comment chunks.
                (TWO_IMG_HEADER as u64 + 3 * BLOCK as u64..=MAX_SP_BYTES + 65_536).contains(&size).then_some(Kind::Blocks)
            } else if (is(&["po", "hdv"]) || (is(&["dsk", "img", "image", "hda"]) && !dos)) && whole {
                Some(Kind::Blocks)
            } else {
                None
            };
        }
        if filter == Filter::Dual {
            // A `.woz` says in its header which drive it's for (`dual_drive`); the rest by name and
            // size, which don't overlap.
            if ext.eq_ignore_ascii_case("woz") {
                return (woz::HEADER_BYTES as u64..=(MAX_BLOCKS * BLOCK) as u64).contains(&size).then_some(Kind::Woz);
            }
            return Kind::of(name, size, Filter::Apple35, exfat).or_else(|| Kind::of(name, size, Filter::Apple2, exfat));
        }
        if filter == Filter::Apple2 {
            let is = |e: &[&str]| e.iter().any(|x| ext.eq_ignore_ascii_case(x));
            let tracks = |track_bytes: usize| match size as usize / track_bytes {
                35 | 40 if size as usize % track_bytes == 0 => Some((size as usize / track_bytes) as u8),
                _ => None,
            };
            // A 13-sector image's size is no 16-sector one's.
            let d13 = tracks(a2::TRACK_BYTES_13);
            return if is(&["d13"]) || (is(&["dsk", "do"]) && d13.is_some()) {
                d13.map(|tracks| Kind::D13 { tracks })
            } else if is(&["dsk", "do", "po"]) {
                let prodos = if is(&["po"]) { Some(true) } else if is(&["do"]) { Some(false) } else { None };
                tracks(a2::TRACK_BYTES).map(|tracks| Kind::A2 { tracks, prodos })
            } else if is(&["nib"]) {
                tracks(a2::NIB_TRACK_BYTES).map(|tracks| Kind::Nib { tracks })
            } else if is(&["woz"]) {
                (woz::HEADER_BYTES as u64..=(MAX_BLOCKS * BLOCK) as u64).contains(&size).then_some(Kind::Woz)
            } else if is(&["a2r"]) {
                (1_024..=(MAX_BLOCKS * BLOCK) as u64).contains(&size).then_some(Kind::A2r)
            } else if is(&["2mg", "2img"]) {
                // The header's lengths are checked once it's read.
                (TWO_IMG_HEADER as u64 + a2::TRACK_BYTES as u64 * 35..=(MAX_BLOCKS * BLOCK) as u64).contains(&size).then_some(Kind::TwoImg)
            } else {
                None
            };
        }
        let fits = (moof::HEADER_BYTES as u64..=(MAX_BLOCKS * BLOCK) as u64).contains(&size);
        if filter == Filter::Apple35 {
            let is = |e: &[&str]| e.iter().any(|x| ext.eq_ignore_ascii_case(x));
            let gcr = |data: usize| data == IMAGE_400K || data == IMAGE_800K;
            let diskcopy_only = is(&["dc42", "diskcopy"]);
            return if is(&["po", "dsk", "img", "image", "hdv"]) && gcr(size as usize) {
                // Blocks in ProDOS order, which is the order a raw Mac image has.
                Some(Kind::Sectors { data: size as usize, diskcopy: false })
            } else if is(&["dsk", "img", "image"]) || diskcopy_only {
                dc42::data_size_for(size).filter(|&d| gcr(d)).map(|data| Kind::Sectors { data, diskcopy: true })
            } else if is(&["2mg", "2img"]) {
                (TWO_IMG_HEADER as u64 + IMAGE_400K as u64..=(MAX_BLOCKS * BLOCK) as u64).contains(&size).then_some(Kind::TwoImg)
            } else if is(&["moof"]) {
                fits.then_some(Kind::Moof)
            } else if is(&["woz"]) {
                fits.then_some(Kind::Woz35)
            } else {
                None
            };
        }
        if ["img", "dsk", "image", "dc42", "diskcopy"].iter().any(|e| ext.eq_ignore_ascii_case(e)) {
            // Raw images are exactly the sector data; DiskCopy adds a header and maybe tags, so
            // the sizes never overlap. `.image` is used for both; `.dc42` and `.diskcopy`
            // (FluxEngine's) only for DiskCopy.
            let raw = [IMAGE_400K, IMAGE_800K, IMAGE_720K, IMAGE_1440K].contains(&(size as usize));
            let diskcopy_only = ext.eq_ignore_ascii_case("dc42") || ext.eq_ignore_ascii_case("diskcopy");
            if raw && !diskcopy_only {
                return Some(Kind::Sectors { data: size as usize, diskcopy: false });
            }
            return dc42::data_size_for(size).map(|data| Kind::Sectors { data, diskcopy: true });
        }
        (ext.eq_ignore_ascii_case("moof") && fits).then_some(Kind::Moof)
    }
}

/// A folder or image file in a directory, judged by its name and size only.
#[derive(Clone)]
pub struct Candidate {
    pub name: Name,
    /// FAT only: the 8.3 name to open it by.
    short: Option<ShortFileName>,
    kind: Kind,
    size: u64,
    /// The mode it was listed for.
    filter: Filter,
}

impl Candidate {
    /// Size class for the menu.
    pub fn describe(&self) -> &'static str {
        match self.kind {
            Kind::Up | Kind::Settings => "",
            Kind::Folder => "<DIR>",
            Kind::Sectors { data: IMAGE_400K, .. } => "400K",
            Kind::Sectors { data: IMAGE_800K, .. } => "800K",
            Kind::Sectors { data: IMAGE_720K, .. } => "720K",
            Kind::Sectors { .. } => "1.44M",
            Kind::Moof => "MOOF",
            Kind::HardDisk => "HD",
            Kind::Blocks => "BLK",
            Kind::A2 { tracks: 35, .. } => "140K",
            Kind::A2 { .. } => "160K",
            Kind::D13 { tracks: 35 } => "116K",
            Kind::D13 { .. } => "130K",
            Kind::Nib { .. } => "NIB",
            Kind::Woz | Kind::Woz35 => "WOZ",
            Kind::A2r => "A2R",
            Kind::TwoImg => "2MG",
        }
    }

    /// The file's size in bytes.
    pub fn size(&self) -> u64 {
        self.size
    }

    pub fn is_up(&self) -> bool {
        self.kind == Kind::Up
    }

    /// A hard disk image (for HD20 mode's [`load_hd`]), not a floppy's.
    pub fn is_hard_disk(&self) -> bool {
        self.kind == Kind::HardDisk
    }

    pub fn is_image(&self) -> bool {
        !matches!(self.kind, Kind::Up | Kind::Folder | Kind::Settings)
    }
}

/// Where a directory is on the card.
#[derive(Clone)]
pub struct Dir {
    /// From the root, `/`-separated (`/Games/Mac`); empty for the root.
    path: Path,
    /// FAT: the 8.3 name of each folder on the way down, `depth` of them.
    fat: [Option<ShortFileName>; MAX_DEPTH],
    depth: usize,
    exfat: bool,
}

impl PartialEq for Dir {
    fn eq(&self, other: &Self) -> bool {
        self.path.as_str() == other.path.as_str()
    }
}

impl Dir {
    /// For the display.
    pub fn path(&self) -> &str {
        if self.depth == 0 { "/" } else { self.path.as_str() }
    }

    pub fn is_root(&self) -> bool {
        self.depth == 0
    }

    pub fn filesystem(&self) -> &'static str {
        if self.exfat { "exFAT" } else { "FAT" }
    }

    /// Path of `name` in this directory, or an error if it won't fit in a [`Path`].
    fn join(&self, name: &str) -> Result<Path, Line> {
        if self.path.len() + 1 + name.len() > Path::CAPACITY {
            return Err(Line::from("path too long"));
        }
        let mut path = self.path.clone();
        let _ = write!(path, "/{}", name);
        Ok(path)
    }

    /// The folder `c` in this directory, or the parent for `..`.
    fn enter(&self, c: &Candidate) -> Result<Dir, Line> {
        let mut dir = self.clone();
        match c.kind {
            Kind::Up => {
                dir.depth = dir.depth.saturating_sub(1);
                dir.fat[dir.depth] = None;
                let parent = dir.path.as_str().rfind('/').unwrap_or(0);
                dir.path.truncate(parent);
            }
            _ => {
                if dir.depth == MAX_DEPTH {
                    return Err(Line::from("folders nested too deep"));
                }
                dir.path = self.join(c.name.as_str())?;
                dir.fat[dir.depth] = c.short.clone();
                dir.depth += 1;
            }
        }
        Ok(dir)
    }
}

/// A directory's folders and image files: `..` first (below the root), then folders, then
/// images, each sorted by name ignoring case.
pub struct Catalog {
    entries: [Option<Candidate>; MAX_IMAGES],
    len: usize,
    dir: Dir,
    /// The root's settings file, if there is one (not one of the entries).
    settings: Option<Candidate>,
    filter: Filter,
}

impl Catalog {
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn get(&self, i: usize) -> Option<&Candidate> {
        self.entries.get(i)?.as_ref()
    }

    pub fn dir(&self) -> &Dir {
        &self.dir
    }

    /// Entry `i`'s position, by name.
    pub fn find(&self, name: &str) -> Option<usize> {
        (0..self.len).find(|&i| self.get(i).is_some_and(|c| c.name.as_str() == name))
    }

    /// Where entry `i` leads, if it is a folder or `..`.
    pub fn folder(&self, i: usize) -> Option<Result<Dir, Line>> {
        let c = self.get(i).filter(|c| !c.is_image())?;
        Some(self.dir.enter(c))
    }

    /// Add a folder, or a file if it looks like an image, or note the settings file. Returns
    /// false once there is nothing more to look for: the catalog is full (and, in the root, the
    /// settings file has been seen).
    fn push(&mut self, name: Name, short: Option<ShortFileName>, folder: bool, size: u64) -> bool {
        let root = self.dir.depth == 0;
        let kind = match folder {
            true if name.as_str().starts_with('.') => None,
            true if name.as_str().eq_ignore_ascii_case("System Volume Information") => None,
            true => Some(Kind::Folder),
            false if root && name.as_str().eq_ignore_ascii_case(config::FILE_NAME) => Some(Kind::Settings),
            false => Kind::of(name.as_str(), size, self.filter, self.dir.exfat),
        };
        match kind {
            Some(Kind::Settings) => self.settings = Some(Candidate { name, short, kind: Kind::Settings, size, filter: self.filter }),
            Some(kind) if self.len < MAX_IMAGES => {
                self.entries[self.len] = Some(Candidate { name, short, kind, size, filter: self.filter });
                self.len += 1;
            }
            _ => {}
        }
        self.len < MAX_IMAGES || (root && self.settings.is_none())
    }

    fn sort(&mut self) {
        fn key(c: &Option<Candidate>) -> (u8, impl Iterator<Item = u8> + '_) {
            let rank = c.as_ref().map_or(2, |c| match c.kind {
                Kind::Up => 0,
                Kind::Folder => 1,
                _ => 2,
            });
            (rank, c.as_ref().map_or("", |c| c.name.as_str()).bytes().map(|b| b.to_ascii_lowercase()))
        }
        self.entries[..self.len].sort_unstable_by(|a, b| {
            let (ra, na) = key(a);
            let (rb, nb) = key(b);
            ra.cmp(&rb).then_with(|| na.cmp(nb))
        });
    }
}

/// What was loaded, for the display and the track builder.
pub struct Loaded {
    pub name: Line,
    pub filesystem: &'static str,
    pub disk: Disk,
    /// Runs of consecutive card sectors the image is stored in.
    pub fragments: u32,
    /// Extra status for the display (MOOF CRC).
    pub note: &'static str,
}

/// No RTC on the board; the card is only read.
struct NoClock;

impl TimeSource for NoClock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Remembers the last card sector read and its contents.
struct Recorder {
    reads: Cell<u32>,
    lba: Cell<u32>,
    data: RefCell<[u8; BLOCK]>,
}

impl Recorder {
    fn new() -> Self {
        Self { reads: Cell::new(0), lba: Cell::new(0), data: RefCell::new([0; BLOCK]) }
    }

    fn note(&self, lba: u32, data: &[u8]) {
        self.reads.set(self.reads.get().wrapping_add(1));
        self.lba.set(lba);
        self.data.borrow_mut().copy_from_slice(data);
    }
}

/// Shares one SD card between the FAT driver (`embedded-sdmmc`) and the exFAT driver
/// (`exfat-embedded`), so a card can be probed as exFAT and fall back to FAT. Every read goes
/// through the [`Recorder`].
struct Shared<'a, D> {
    card: &'a D,
    rec: &'a Recorder,
}

impl<D: embedded_sdmmc::BlockDevice> embedded_sdmmc::BlockDevice for Shared<'_, D> {
    type Error = D::Error;

    fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.card.read(blocks, start)?;
        if let Some(last) = blocks.last() {
            self.rec.note(start.0 + blocks.len() as u32 - 1, &last.contents);
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Self::Error> {
        self.card.write(blocks, start)
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        self.card.num_blocks()
    }
}

impl<D: embedded_sdmmc::BlockDevice> exfat_embedded::BlockDevice for Shared<'_, D> {
    type Error = D::Error;

    fn sector_size(&self) -> usize {
        Block::LEN
    }

    fn sector_count(&self) -> u64 {
        self.card.num_blocks().map_or(0, |b| u64::from(b.0))
    }

    fn read_sector(&mut self, lba: u64, out: &mut [u8]) -> Result<(), Self::Error> {
        let mut block = [Block::new()];
        self.card.read(&mut block, BlockIdx(lba as u32))?;
        out.copy_from_slice(&block[0].contents);
        self.rec.note(lba as u32, out);
        Ok(())
    }

    fn write_sector(&mut self, lba: u64, data: &[u8]) -> Result<(), Self::Error> {
        let mut block = Block::new();
        block.contents.copy_from_slice(data);
        self.card.write(&[block], BlockIdx(lba as u32))
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn error(e: impl Debug) -> Line {
    let mut l = Line::new();
    let _ = write!(l, "{:?}", e);
    l
}

/// Read the file once, block by block, and record in `map` the card sector each block came from.
/// `read_block` reads the next `buf.len()` bytes of the file (512, or less for a partial last
/// block); `on_block` sees each block's data. Each block must have come from exactly the sector
/// the recorder saw last, with the same contents, or mapping fails.
fn map_blocks(
    rec: &Recorder,
    size: u64,
    map: &mut BlockMap,
    mut read_block: impl FnMut(&mut [u8]) -> Result<(), Line>,
    mut on_block: impl FnMut(usize, &[u8]),
) -> Result<u32, Line> {
    let mut buf = [0u8; BLOCK];
    map.clear();
    let blocks = (size as usize).div_ceil(BLOCK);
    for i in 0..blocks {
        let len = (size as usize - i * BLOCK).min(BLOCK);
        let before = rec.reads.get();
        read_block(&mut buf[..len])?;
        if rec.reads.get() == before || rec.data.borrow()[..len] != buf[..len] {
            let mut l = Line::new();
            let _ = write!(l, "can't map image block {}", i);
            return Err(l);
        }
        map.push(rec.lba.get())?;
        on_block(i, &buf[..len]);
    }
    Ok(map.fragments())
}

/// Map one candidate and work out its disk. For a MOOF this parses the header (and the FLUX
/// chunk's block, if any) and checks the CRC, all from the data seen while mapping;
/// `on_found(true)` is called as soon as the header says MFM, so the HD media flag (R15) is
/// right early.
fn load_candidate(
    rec: &Recorder,
    c: &Candidate,
    map: &mut BlockMap,
    on_found: &mut impl FnMut(bool),
    read_block: impl FnMut(&mut [u8]) -> Result<(), Line>,
) -> Result<(Disk, u32, &'static str), Line> {
    let mut header = [0u8; moof::HEADER_BYTES];
    let mut flux_chunk: Option<[u8; BLOCK]> = None;
    let mut crc = Crc32::new();
    let mut dc42_header = [0u8; dc42::HEADER_BYTES];
    let mut dc42_sum = dc42::Checksum::new();
    // WOZ 1 keeps each track's bit count in its record, past the header.
    let mut woz1_bits = [0u16; woz::QUARTERS];
    // Dual mode: a `.woz` that may be a 3.5" disk, read as a MOOF if its header says so.
    let woz_dual = c.kind == Kind::Woz && c.filter == Filter::Dual;
    // An A2R's captures, found as its blocks go by.
    let mut a2r_index = a2r::Indexer::new();
    let fragments = map_blocks(rec, c.size, map, read_block, |i, data| {
        if let Kind::Sectors { data: size, diskcopy: true } = c.kind {
            let at = i * BLOCK;
            if i == 0 {
                dc42_header.copy_from_slice(&data[..dc42::HEADER_BYTES]);
            }
            // The checksum covers the sector data only (even offsets and lengths: whole words).
            let start = at.max(dc42::HEADER_BYTES);
            let end = (at + data.len()).min(dc42::HEADER_BYTES + size);
            if start < end {
                dc42_sum.update(&data[start - at..end - at]);
            }
        }
        // The first blocks: a MOOF, WOZ or 2MG header, or for a `.dsk` where a ProDOS volume
        // directory would be (block 2).
        let at = i * BLOCK;
        if at < header.len() {
            header[at..at + data.len()].copy_from_slice(data);
        }
        if c.kind == Kind::A2r {
            a2r_index.feed(data);
        }
        if c.kind == Kind::Woz {
            if let Some((j, k)) = Woz::v1_bit_count_at(i).filter(|&(_, k)| k + 2 <= data.len()) {
                woz1_bits[j] = u16::from_le_bytes([data[k], data[k + 1]]);
            }
            crc.update(if i == 0 { &data[12..] } else { data });
        }
        // The FLUX chunk (MOOF, WOZ 2.1) has a block of its own, named in INFO (block 0). It may
        // be the file's last, partial block (Oids): ID, size and the 160-byte map are enough.
        if matches!(c.kind, Kind::Moof | Kind::Woz35 | Kind::Woz)
            && i > 0
            && Moof::flux_block(&header) == Some(i)
            && data.len() >= 8 + moof::TRK_COUNT
        {
            let mut chunk = [0u8; BLOCK];
            chunk[..data.len()].copy_from_slice(data);
            flux_chunk = Some(chunk);
        }
        if c.kind == Kind::Moof || c.kind == Kind::Woz35 || woz_dual {
            if at < header.len() && at + data.len() > 21 && header[..4] == *b"MOOF" && header[21] == 3 {
                on_found(true); // INFO disk type 3: 1.44 MB MFM
            }
            // The CRC covers bytes 12 to the end of the file (a WOZ's is above).
            if !woz_dual {
                crc.update(if i == 0 { &data[12..] } else { data });
            }
        }
    })?;
    // INFO's disk type 2: a 3.5" WOZ.
    let kind = if woz_dual && header[21] == 2 { Kind::Woz35 } else { c.kind };
    let (disk, note) = match kind {
        Kind::Sectors { data, diskcopy } => {
            let (offset, note) = if diskcopy {
                let h = dc42::Header::parse(&dc42_header, c.size).map_err(Line::from)?;
                if h.data_size != data {
                    return Err(Line::from("DC42: size doesn't match header"));
                }
                // A bad checksum is reported, not refused: the data is usually still good.
                (dc42::HEADER_BYTES as u16, if h.data_checksum == dc42_sum.finish() { "sum ok" } else { "sum BAD" })
            } else {
                (0, "")
            };
            let diskcopy = offset != 0;
            let apple2 = matches!(c.filter, Filter::Apple35 | Filter::Dual);
            let disk = match data {
                IMAGE_400K => Disk::Gcr { sides: 1, offset, format: gcr::format(1, apple2), diskcopy, locked: false },
                IMAGE_800K => Disk::Gcr { sides: 2, offset, format: gcr::format(2, apple2), diskcopy, locked: false },
                IMAGE_720K => Disk::Mfm { format: mfm::Format::Dd, offset },
                _ => Disk::Mfm { format: mfm::Format::Hd, offset },
            };
            (disk, note)
        }
        Kind::Up | Kind::Folder | Kind::Settings | Kind::HardDisk | Kind::Blocks => return Err(Line::from("not a floppy image")),
        Kind::A2 { tracks, prodos } => {
            let found = prodos_volume(&header[2 * BLOCK..3 * BLOCK]);
            let (order, note) = match prodos {
                Some(true) => (a2::Order::Prodos, ""),
                Some(false) => (a2::Order::Dos, ""),
                // A `.dsk` is DOS order unless it has a ProDOS volume directory where ProDOS
                // order puts it.
                None if found => (a2::Order::Prodos, "ProDOS order found"),
                None => (a2::Order::Dos, ""),
            };
            (Disk::A2 { order, tracks, volume: a2::VOLUME, offset: 0, locked: false }, note)
        }
        Kind::D13 { tracks } => (Disk::A2 { order: a2::Order::Dos32, tracks, volume: a2::VOLUME, offset: 0, locked: false }, "13-sector"),
        Kind::Nib { tracks } => (Disk::Nib { tracks, offset: 0 }, "read-only"),
        Kind::TwoImg => (two_img(&header[..TWO_IMG_HEADER], c.size, c.filter)?, "2MG"),
        Kind::Woz => {
            let blocks = (c.size as usize).div_ceil(BLOCK);
            let w = Woz::parse(&header, &woz1_bits, flux_chunk.as_ref(), blocks, MAX_TRACK_BLOCKS).map_err(Line::from)?;
            let note = match w.stored_crc {
                0 => "no CRC",
                stored if stored == crc.finish() => "CRC ok",
                _ => return Err(Line::from("WOZ: CRC mismatch")),
            };
            (Disk::Woz(w), note)
        }
        Kind::A2r => (Disk::A2r(a2r_index.finish().map_err(Line::from)?), "read-only"),
        Kind::Moof | Kind::Woz35 => {
            let blocks = (c.size as usize).div_ceil(BLOCK);
            let m = Moof::parse(&header, flux_chunk.as_ref(), blocks, MAX_TRACK_BLOCKS).map_err(Line::from)?;
            if kind == Kind::Moof && m.woz {
                return Err(Line::from("MOOF: a WOZ file"));
            }
            // An Apple 3.5 Drive is an 800K drive.
            if m.mfm && matches!(c.filter, Filter::Apple35 | Filter::Dual) {
                return Err(Line::from("MOOF: 1.44 MB (an Apple 3.5 Drive is 800K)"));
            }
            let note = match m.stored_crc {
                0 => "no CRC",
                stored if stored == crc.finish() => "CRC ok",
                _ if m.woz => return Err(Line::from("WOZ: CRC mismatch")),
                _ => return Err(Line::from("MOOF: CRC mismatch")),
            };
            (Disk::Moof(m), note)
        }
    };
    Ok((disk, fragments, note))
}

/// Bytes of a `.2mg` header.
const TWO_IMG_HEADER: usize = 64;

/// A ProDOS volume directory's key block: no previous block, storage type `F`, a name, and
/// 13 entries of 39 bytes.
fn prodos_volume(block: &[u8]) -> bool {
    block[..2] == [0, 0] && block[4] >> 4 == 0xF && block[4] & 0xF != 0 && block[0x23] == 0x27 && block[0x24] == 0x0D
}

/// The disk in a `.2mg` of `size` bytes, from its header: 5.25" sectors (DOS or ProDOS order)
/// or nibbles, or in Apple II 3.5" mode ([`Filter::Apple35`]) 400K/800K blocks.
fn two_img(h: &[u8], size: u64, filter: Filter) -> Result<Disk, Line> {
    let TwoImg { format, flags, offset, len, locked } = two_img_header(h, size)?;
    let (offset, len) = (offset as u16, len as usize);
    let gcr_sides = match len {
        IMAGE_400K => Some(1),
        IMAGE_800K => Some(2),
        _ => None,
    };
    // Dual mode: whichever drive's disk it is.
    let filter = match filter {
        Filter::Dual if gcr_sides.is_some() => Filter::Apple35,
        Filter::Dual => Filter::Apple2,
        f => f,
    };
    if filter == Filter::Apple35 {
        // Blocks in order. A 3.5" disk has no DOS 3.3 order: writers that say so mean blocks too.
        return match (format, gcr_sides) {
            (0 | 1, Some(sides)) => Ok(Disk::Gcr { sides, offset, format: gcr::format(sides, true), diskcopy: false, locked }),
            (0..=2, None) => Err(Line::from("2MG: not a 3.5\" disk (use mode = apple2)")),
            _ => Err(Line::from("2MG: unknown image format")),
        };
    }
    if gcr_sides.is_some() && format != 2 {
        return Err(Line::from("2MG: a 3.5\" disk (use mode = apple35)"));
    }
    let tracks = |track_bytes: usize| match len / track_bytes {
        35 | 40 if len % track_bytes == 0 => Ok((len / track_bytes) as u8),
        _ => Err(Line::from("2MG: not a 5.25\" disk")),
    };
    // Flag bit 8: the low byte is the DOS volume number.
    let volume = if flags & 0x100 != 0 { flags as u8 } else { a2::VOLUME };
    match format {
        0 => Ok(Disk::A2 { order: a2::Order::Dos, tracks: tracks(a2::TRACK_BYTES)?, volume, offset, locked }),
        1 => Ok(Disk::A2 { order: a2::Order::Prodos, tracks: tracks(a2::TRACK_BYTES)?, volume, offset, locked }),
        2 => Ok(Disk::Nib { tracks: tracks(a2::NIB_TRACK_BYTES)?, offset }),
        _ => Err(Line::from("2MG: unknown image format")),
    }
}

/// What a `.2mg`'s 64-byte header says.
struct TwoImg {
    /// 0 DOS 3.3 order, 1 ProDOS order, 2 nibbles.
    format: u32,
    flags: u32,
    /// Where the disk's data is in the file, and its length.
    offset: u32,
    len: u32,
    /// Flag bit 31: write-protected.
    locked: bool,
}

/// Read a `.2mg` header `h` from a file of `size` bytes, and check the data is in the file.
fn two_img_header(h: &[u8], size: u64) -> Result<TwoImg, Line> {
    let u32_at = |at: usize| u32::from_le_bytes([h[at], h[at + 1], h[at + 2], h[at + 3]]);
    if &h[..4] != b"2IMG" {
        return Err(Line::from("2MG: bad signature"));
    }
    let (format, flags, offset) = (u32_at(12), u32_at(16), u32_at(24));
    // Some writers leave the data length 0 and give ProDOS blocks instead.
    let len = match u32_at(28) {
        0 if format == 1 => u32_at(20) * BLOCK as u32,
        len => len,
    };
    if offset < TWO_IMG_HEADER as u32 || offset > u32::from(u16::MAX) || u64::from(offset) + u64::from(len) > size {
        return Err(Line::from("2MG: bad data offset or length"));
    }
    Ok(TwoImg { format, flags, offset, len, locked: flags & 0x8000_0000 != 0 })
}

/// List directory `dir`, or the root when `None` (which also works out whether the card is
/// exFAT or FAT), with the images `filter` wants. Only names and sizes are checked here; [`load`]
/// or [`load_hd`] finds out whether an image is usable.
pub fn scan<D>(card: &D, dir: Option<&Dir>, filter: Filter) -> Result<Catalog, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let root = Dir { path: Path::new(), fat: [const { None }; MAX_DEPTH], depth: 0, exfat: true };
    let mut catalog = Catalog { entries: [const { None }; MAX_IMAGES], len: 0, dir: root, settings: None, filter };
    scan_into(card, dir, filter, &mut catalog)?;
    Ok(catalog)
}

/// [`scan`] into `catalog`, in place (a catalog is ~21 KB: walking folders with `scan` keeps a
/// copy or two on the caller's stack). On an error `catalog` holds what was listed so far.
pub fn scan_into<D>(card: &D, dir: Option<&Dir>, filter: Filter, catalog: &mut Catalog) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let root = Dir { path: Path::new(), fat: [const { None }; MAX_DEPTH], depth: 0, exfat: true };
    let dir = dir.unwrap_or(&root);
    catalog.entries.iter_mut().for_each(|e| *e = None);
    catalog.len = 0;
    catalog.dir = dir.clone();
    catalog.settings = None;
    catalog.filter = filter;
    if dir.depth > 0 {
        let up = Candidate { name: Name::from(".."), short: None, kind: Kind::Up, size: 0, filter };
        catalog.entries[0] = Some(up);
        catalog.len = 1;
    }
    let rec = Recorder::new();
    let mut sector = [0u8; Block::LEN];
    let mut scratch = Scratch::new(&mut sector);
    let mounted = match ExfatFs::mount(Shared { card, rec: &rec }, &mut scratch) {
        Ok(fs) if dir.exfat => Some(fs),
        Err(exfat_embedded::Error::PartitionNotFound | exfat_embedded::Error::InvalidBootSector)
            if dir.depth == 0 =>
        {
            None
        }
        Ok(_) => return Err(Line::from("card changed")),
        Err(e) => return Err(error(e)),
    };
    let Some(mut fs) = mounted else {
        catalog.dir.exfat = false;
        scan_fat(card, &rec, catalog)?;
        catalog.sort();
        return Ok(());
    };
    let visit = |entry: &exfat_embedded::DirectoryEntry| {
        let mut name = Name::new();
        for c in char::decode_utf16(entry.name_utf16().iter().copied()) {
            match c {
                Ok(c) if c.is_ascii() => {
                    let _ = name.write_char(c);
                }
                _ => return true, // opening by path needs the exact name
            }
        }
        catalog.push(name, None, entry.is_directory, entry.valid_length)
    };
    if dir.depth == 0 {
        fs.read_root(&mut scratch, visit).map_err(error)?;
    } else {
        let entry = fs.lookup(dir.path.as_str(), &mut scratch).map_err(error)?;
        let folder = entry.directory().ok_or(Line::from("not a folder"))?;
        fs.read_directory(folder, &mut scratch, visit).map_err(error)?;
    }
    catalog.sort();
    Ok(())
}

/// Open `dir` on a FAT volume, one folder at a time (the volume manager has room for 4 open
/// directories, so each parent is closed on the way).
fn open_fat_dir<V: embedded_sdmmc::BlockDevice>(
    vm: &VolumeManager<V, NoClock>,
    volume: RawVolume,
    dir: &Dir,
) -> Result<RawDirectory, Line>
where
    V::Error: Debug,
{
    let mut open = vm.open_root_dir(volume).map_err(error)?;
    for short in dir.fat[..dir.depth].iter() {
        let short = short.as_ref().ok_or(Line::from("no short name"))?;
        let child = vm.open_dir(open, short);
        let _ = vm.close_dir(open);
        open = child.map_err(error)?;
    }
    Ok(open)
}

fn scan_fat<D>(card: &D, rec: &Recorder, catalog: &mut Catalog) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let vm = VolumeManager::new(Shared { card, rec }, NoClock);
    let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
    let dir = catalog.dir.clone();
    let result = open_fat_dir(&vm, volume, &dir).and_then(|open| {
        let mut lfn_storage = [0u8; 256];
        let mut lfn = LfnBuffer::new(&mut lfn_storage);
        let result = vm.iterate_dir_lfn(open, &mut lfn, |entry, long_name| {
            let a = entry.attributes;
            if a.is_volume() || a.is_hidden() || a.is_system() {
                return ControlFlow::Continue(());
            }
            let mut name = Name::new();
            match long_name {
                Some(n) => {
                    let _ = name.write_str(n);
                }
                None => {
                    let _ = write!(name, "{}", entry.name); // also `.` and `..`, skipped by `push`
                }
            }
            match catalog.push(name, Some(entry.name.clone()), a.is_directory(), u64::from(entry.size)) {
                true => ControlFlow::Continue(()),
                false => ControlFlow::Break(()),
            }
        });
        let _ = vm.close_dir(open);
        result.map_err(error)
    });
    let _ = vm.close_volume(volume);
    result
}

/// Read the root's settings file into `text`, as far as it fits. Returns false if `root` (the
/// root's catalog) has no settings file.
pub fn read_settings<D>(card: &D, root: &Catalog, text: &mut config::Text) -> Result<bool, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let Some(c) = &root.settings else { return Ok(false) };
    let buf = text.buf_mut();
    let len = (c.size as usize).min(buf.len());
    let buf = &mut buf[..len];
    let rec = Recorder::new();
    let mut done = 0;
    if root.dir.exfat {
        let mut sector = [0u8; Block::LEN];
        let mut scratch = Scratch::new(&mut sector);
        let mut fs = ExfatFs::mount(Shared { card, rec: &rec }, &mut scratch).map_err(error)?;
        let path = root.dir.join(c.name.as_str())?;
        let mut file = fs.open(path.as_str(), &mut scratch).map_err(error)?;
        while done < len {
            match fs.read(&mut file, &mut buf[done..], &mut scratch).map_err(error)? {
                0 => break,
                n => done += n,
            }
        }
    } else {
        let Some(short) = &c.short else { return Err(Line::from("no short name")) };
        let vm = VolumeManager::new(Shared { card, rec: &rec }, NoClock);
        let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
        let result = vm.open_root_dir(volume).map_err(error).and_then(|dir| {
            let result = vm.open_file_in_dir(dir, short, Mode::ReadOnly).map_err(error).and_then(|file| {
                let result = loop {
                    if done == len {
                        break Ok(());
                    }
                    match vm.read(file, &mut buf[done..]) {
                        Ok(0) => break Ok(()),
                        Ok(n) => done += n,
                        Err(e) => break Err(error(e)),
                    }
                };
                let _ = vm.close_file(file);
                result
            });
            let _ = vm.close_dir(dir);
            result
        });
        let _ = vm.close_volume(volume);
        result?;
    }
    text.read(done, c.size);
    Ok(true)
}

/// Save `text` as the root's settings file, replacing it or creating it. `dir` is any directory
/// of the card (only its filesystem matters).
pub fn write_settings<D>(card: &D, dir: &Dir, text: &[u8]) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    write_root_file(card, dir, config::FILE_NAME, false, &mut |sink| sink(text))
}

/// Write the root's file `name` (at most 13 ASCII characters, matched ignoring case), created if
/// it isn't there: emptied first, or with `append` added to. `produce` hands its bytes to the
/// sink it's given, in as many pieces as it likes. `dir` is any directory of the card.
pub fn write_root_file<D>(
    card: &D,
    dir: &Dir,
    name: &str,
    append: bool,
    produce: &mut dyn FnMut(&mut dyn FnMut(&[u8]) -> Result<(), Line>) -> Result<(), Line>,
) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let rec = Recorder::new();
    if dir.exfat {
        let mut sector = [0u8; Block::LEN];
        let mut scratch = Scratch::new(&mut sector);
        let mut fs = ExfatFs::mount(Shared { card, rec: &rec }, &mut scratch).map_err(error)?;
        let mut path = Path::new();
        let _ = write!(path, "/{}", name);
        let opened = if append { fs.open(path.as_str(), &mut scratch).ok() } else { None };
        // Emptied if it's there (names match ignoring case), created if not.
        let mut file = match opened {
            Some(file) => file,
            None => fs.create(name, &mut scratch).map_err(error)?,
        };
        produce(&mut |data| fs.append(&mut file, data, &mut scratch).map(|_| ()).map_err(error))?;
        return fs.flush(&mut scratch).map_err(error);
    }
    // FAT: find the file's short name; `embedded-sdmmc` can only open it by that.
    let found = {
        let vm = VolumeManager::new(Shared { card, rec: &rec }, NoClock);
        let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
        let result = vm.open_root_dir(volume).map_err(error).and_then(|root| {
            let mut lfn_storage = [0u8; 256];
            let mut lfn = LfnBuffer::new(&mut lfn_storage);
            let mut found = None;
            let result = vm.iterate_dir_lfn(root, &mut lfn, |entry, long_name| {
                let a = entry.attributes;
                if !a.is_directory() && !a.is_volume() && long_name.is_some_and(|n| n.eq_ignore_ascii_case(name)) {
                    found = Some(entry.name.clone());
                    return ControlFlow::Break(());
                }
                ControlFlow::Continue(())
            });
            let _ = vm.close_dir(root);
            result.map_err(error).map(|()| found)
        });
        let _ = vm.close_volume(volume);
        result?
    };
    let short = match found {
        Some(short) => short,
        None => {
            // Its long name has to be written by hand, before the volume manager reads the root
            // (its block cache would miss it otherwise).
            let read = |lba: u32, out: &mut [u8; BLOCK]| read_sector(card, lba, out);
            let write = |lba: u32, data: &[u8; BLOCK]| {
                card.write(&[Block { contents: *data }], BlockIdx(lba)).map_err(|_| "SD write error")
            };
            let raw = clusters::add_root_file(read, write, name).map_err(Line::from)?;
            let mut name = FixedStr::<12>::new();
            let part = |b| core::str::from_utf8(b).map_or("", str::trim_end);
            let _ = write!(name, "{}.{}", part(&raw[..8]), part(&raw[8..]));
            ShortFileName::create_from_str(name.as_str()).map_err(error)?
        }
    };
    let vm = VolumeManager::new(Shared { card, rec: &rec }, NoClock);
    let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
    let result = vm.open_root_dir(volume).map_err(error).and_then(|root| {
        let mode = if append { Mode::ReadWriteAppend } else { Mode::ReadWriteTruncate };
        let result = vm.open_file_in_dir(root, &short, mode).map_err(error).and_then(|file| {
            let written = produce(&mut |data| vm.write(file, data).map_err(error));
            // Closing writes the new size into the directory entry.
            written.and(vm.close_file(file).map_err(error))
        });
        let _ = vm.close_dir(root);
        result
    });
    let _ = vm.close_volume(volume);
    result
}

/// Map image `c` of directory `dir` into `map` and work out its disk. `on_found(hd)` is called as
/// soon as HD media is known (from the name and size, or a MOOF's first block), before the slow
/// mapping pass, so the drive can present the right identity early.
pub fn load<D>(
    card: &D,
    dir: &Dir,
    c: &Candidate,
    map: &mut BlockMap,
    mut on_found: impl FnMut(bool),
) -> Result<Loaded, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    if !c.is_image() {
        return Err(Line::from("not an image"));
    }
    on_found(matches!(c.kind, Kind::Sectors { data: IMAGE_1440K, .. }));
    let rec = Recorder::new();
    let (disk, fragments, note) = read_file(card, dir, c, &rec, |read| load_candidate(&rec, c, map, &mut on_found, read))?;
    Ok(Loaded { name: Line::from(c.name.as_str()), filesystem: dir.filesystem(), disk, fragments, note })
}

/// Dual mode: which drive image `c` is for, 0 the 3.5" one or 1 the 5.25". A `.woz` says in its
/// header (read here); the rest go by their kind and size.
pub fn dual_drive<D>(card: &D, dir: &Dir, c: &Candidate) -> Result<usize, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    match c.kind {
        Kind::A2 { .. } | Kind::D13 { .. } | Kind::Nib { .. } | Kind::A2r => Ok(1),
        Kind::TwoImg => Ok((c.size < (TWO_IMG_HEADER + IMAGE_400K) as u64) as usize),
        Kind::Woz => {
            // INFO's disk type: 1 = 5.25", 2 = 3.5".
            let mut head = [0u8; 22];
            read_file(card, dir, c, &Recorder::new(), |read| read(&mut head))?;
            Ok((head[21] != 2) as usize)
        }
        _ => Ok(0),
    }
}

/// Open image `c` of directory `dir` and hand `f` a reader that fills each buffer it's given from
/// the file in order, through `rec`'s device (exFAT or FAT, as `dir` is).
fn read_file<D, R>(
    card: &D,
    dir: &Dir,
    c: &Candidate,
    rec: &Recorder,
    f: impl FnOnce(&mut dyn FnMut(&mut [u8]) -> Result<(), Line>) -> Result<R, Line>,
) -> Result<R, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    if dir.exfat {
        let mut sector = [0u8; Block::LEN];
        let mut scratch = Scratch::new(&mut sector);
        let mut fs = ExfatFs::mount(Shared { card, rec }, &mut scratch).map_err(error)?;
        let path = dir.join(c.name.as_str())?;
        let mut file = fs.open(path.as_str(), &mut scratch).map_err(error)?;
        f(&mut |buf| match fs.read(&mut file, buf, &mut scratch).map_err(error)? {
            n if n == buf.len() => Ok(()),
            _ => Err(Line::from("short read")),
        })
    } else {
        let Some(short) = &c.short else { return Err(Line::from("no short name")) };
        let vm = VolumeManager::new(Shared { card, rec }, NoClock);
        let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
        let root = match open_fat_dir(&vm, volume, dir) {
            Ok(open) => open,
            Err(e) => {
                let _ = vm.close_volume(volume);
                return Err(e);
            }
        };
        let result = match vm.open_file_in_dir(root, short, Mode::ReadOnly) {
            Ok(file) => {
                let result = f(&mut |buf| {
                    let mut done = 0;
                    while done < buf.len() {
                        match vm.read(file, &mut buf[done..]).map_err(error)? {
                            0 => return Err(Line::from("short read")),
                            n => done += n,
                        }
                    }
                    Ok(())
                });
                let _ = vm.close_file(file);
                result
            }
            Err(e) => Err(error(e)),
        };
        let _ = vm.close_dir(root);
        let _ = vm.close_volume(volume);
        result
    }
}

/// A hard disk image, mapped for HD20 mode.
pub struct HdLoaded {
    pub name: Line,
    pub filesystem: &'static str,
    pub volume: dcd::Volume,
    /// Runs of consecutive card sectors the image is stored in.
    pub fragments: u32,
}

/// Read card sector `lba`.
fn read_sector<D: embedded_sdmmc::BlockDevice>(card: &D, lba: u32, out: &mut [u8; BLOCK]) -> Result<(), &'static str> {
    let mut block = [Block::new()];
    card.read(&mut block, BlockIdx(lba)).map_err(|_| "SD read error")?;
    out.copy_from_slice(&block[0].contents);
    Ok(())
}

/// Map hard disk image `c` of directory `dir` into `map` and find its volume
/// (`dcd::find_volume`). The cluster chain comes from the FAT (`clusters.rs`) rather than from
/// reading the file through, which for an image of a gigabyte would take minutes.
pub fn load_hd<D>(card: &D, dir: &Dir, c: &Candidate, map: &mut BlockMap) -> Result<HdLoaded, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    if c.kind != Kind::HardDisk {
        return Err(Line::from("not a hard disk image"));
    }
    let blocks = (c.size / BLOCK as u64) as u32;
    map_clusters(card, dir, c, blocks, map)?;
    let volume = dcd::find_volume(blocks, |b, out| read_sector(card, map.sectors(b as usize).0, out)).map_err(Line::from)?;
    Ok(HdLoaded { name: Line::from(c.name.as_str()), filesystem: dir.filesystem(), volume, fragments: map.fragments() })
}

/// A block image mapped for a SmartPort unit.
pub struct BlockImage {
    pub name: Line,
    pub filesystem: &'static str,
    /// Bytes before block 0 in the file: a `.2mg`'s header (and anything after it), else 0.
    pub offset: u32,
    pub blocks: u32,
    /// A `.2mg` marked write-protected.
    pub locked: bool,
    pub fragments: u32,
}

/// Map block image `c` of `dir` into `map` for a SmartPort unit, through its cluster chain.
pub fn load_blocks<D, const N: usize>(card: &D, dir: &Dir, c: &Candidate, map: &mut BlockMap<N>) -> Result<BlockImage, Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    if c.kind != Kind::Blocks {
        return Err(Line::from("not a block image"));
    }
    map_clusters(card, dir, c, c.size.div_ceil(BLOCK as u64) as u32, map)?;
    let ext = c.name.as_str().rsplit_once('.').map_or("", |(_, e)| e);
    let (offset, len, locked) = if ext.eq_ignore_ascii_case("2mg") || ext.eq_ignore_ascii_case("2img") {
        let mut head = [0u8; BLOCK];
        read_sector(card, map.sectors(0).0, &mut head).map_err(Line::from)?;
        let h = two_img_header(&head[..TWO_IMG_HEADER], c.size)?;
        if h.format == 2 {
            return Err(Line::from("2MG: nibbles, not blocks"));
        }
        // DOS 3.3 order is a 5.25" disk's; on a 3.5" or bigger one, writers that say so mean blocks.
        if h.format == 0 && h.len as usize <= a2::TRACK_BYTES * 40 {
            return Err(Line::from("2MG: DOS 3.3 order, not blocks"));
        }
        (h.offset, h.len, h.locked)
    } else {
        (0, c.size as u32, false)
    };
    if len as usize % BLOCK != 0 || len < 3 * BLOCK as u32 || u64::from(len) > MAX_SP_BYTES {
        return Err(Line::from("not whole blocks, or over 32 MB"));
    }
    Ok(BlockImage {
        name: Line::from(c.name.as_str()),
        filesystem: dir.filesystem(),
        offset,
        // A 32 MB file's last block is past what ProDOS can ask for.
        blocks: (len / BLOCK as u32).min(MAX_SP_BLOCKS),
        locked,
        fragments: map.fragments(),
    })
}

/// Map the first `blocks` blocks of file `c` of `dir` into `map` from its cluster chain, without
/// reading the file through.
fn map_clusters<D, const N: usize>(card: &D, dir: &Dir, c: &Candidate, blocks: u32, map: &mut BlockMap<N>) -> Result<(), Line>
where
    D: embedded_sdmmc::BlockDevice,
    D::Error: Debug,
{
    let read = |lba: u32, out: &mut [u8; BLOCK]| read_sector(card, lba, out);
    map.clear();
    let rec = Recorder::new();
    if dir.exfat {
        let mut sector = [0u8; Block::LEN];
        let mut scratch = Scratch::new(&mut sector);
        let mut fs = ExfatFs::mount(Shared { card, rec: &rec }, &mut scratch).map_err(error)?;
        let path = dir.join(c.name.as_str())?;
        let entry = fs.lookup(path.as_str(), &mut scratch).map_err(error)?;
        if entry.is_directory || entry.valid_length != c.size {
            return Err(Line::from("file changed"));
        }
        let g = fs.geometry();
        let layout = clusters::Layout::exfat(
            g.partition.first_lba as u32,
            g.fat_offset,
            g.cluster_heap_offset,
            g.sectors_per_cluster,
            g.cluster_count,
        );
        clusters::walk(&layout, entry.first_cluster, blocks, entry.no_fat_chain, read, |lba, n| map.push_run(lba, n))
            .map_err(Line::from)?;
    } else {
        // embedded-sdmmc doesn't say where a file starts, so read its first block and see which
        // sector that came from; the rest of the chain comes from the FAT.
        let Some(short) = &c.short else { return Err(Line::from("no short name")) };
        let vm = VolumeManager::new(Shared { card, rec: &rec }, NoClock);
        let volume = vm.open_raw_volume(VolumeIdx(0)).map_err(error)?;
        let first = open_fat_dir(&vm, volume, dir).and_then(|open| {
            let first = vm.open_file_in_dir(open, short, Mode::ReadOnly).map_err(error).and_then(|file| {
                let mut buf = [0u8; BLOCK];
                let before = rec.reads.get();
                let result = match vm.read(file, &mut buf) {
                    Ok(n) if n == BLOCK && rec.reads.get() != before && *rec.data.borrow() == buf => Ok(rec.lba.get()),
                    Ok(_) => Err(Line::from("can't map image block 0")),
                    Err(e) => Err(error(e)),
                };
                let _ = vm.close_file(file);
                result
            });
            let _ = vm.close_dir(open);
            first
        });
        let _ = vm.close_volume(volume);
        let first = first?;
        let layout = clusters::Layout::fat(read).map_err(Line::from)?;
        let cluster = layout.cluster_at(first).ok_or(Line::from("FAT: image isn't where its directory says"))?;
        clusters::walk(&layout, cluster, blocks, false, read, |lba, n| map.push_run(lba, n)).map_err(Line::from)?;
    }
    if map.blocks != blocks as usize {
        return Err(Line::from("can't map image"));
    }
    Ok(())
}
