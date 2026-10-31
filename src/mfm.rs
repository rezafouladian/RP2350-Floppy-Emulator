//! MFM track encoder for 720K (DD media) and 1.44 MB (HD media) disks on a SuperDrive.
//!
//! Builds the cell stream for one track side: one bit per 1 µs half-cell (clock and data bits
//! alternate), `1` = flux reversal (see `track.rs`). Layout follows the ROM formatter
//! at the length of one revolution, so the track loops
//! seamlessly in gap 4.
//!
//! Both formats play at the same 1 µs half-cells: the ROM programs the ISM with the same 2/3/4 µs
//! windows for 720K and 1.44 MB, so the Mac expects DD media at
//! the HD data rate. A 720K track's 6,250 bytes then take 100 ms, as if the disk spun at 600 RPM.

use crate::track::{BLOCK, Bits, MAX_TRACK_WORDS, TrackBuf};

/// Which MFM disk: the sector count, gap 3 and revolution length differ.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// 720K on DD media: 9 sectors per track side.
    Dd,
    /// 1.44 MB on HD media: 18 sectors per track side.
    Hd,
}

impl Format {
    /// Sectors per track side, numbered from 1.
    pub const fn sectors(self) -> u8 {
        match self {
            Format::Dd => 9,
            Format::Hd => 18,
        }
    }

    const fn gap3(self) -> usize {
        match self {
            Format::Dd => 80,
            Format::Hd => 101,
        }
    }

    /// Data bytes in one revolution: 300 RPM at 250 kbit/s (DD) or 500 kbit/s (HD).
    const fn track_bytes(self) -> usize {
        match self {
            Format::Dd => 6_250,
            Format::Hd => 12_500,
        }
    }

    const fn sector_bytes(self) -> usize {
        SECTOR_BYTES_NO_GAP3 + self.gap3()
    }

    /// The formatted fields, without gap 4b.
    const fn layout_bytes(self) -> usize {
        FIRST_SECTOR + self.sectors() as usize * self.sector_bytes()
    }

    /// The track fits a buffer, and the formatted fields a revolution.
    const fn fits(self) -> bool {
        self.track_bytes() * 16 / 32 <= MAX_TRACK_WORDS && self.layout_bytes() <= self.track_bytes()
    }

    /// First block of a track side: images run cylinder, side, sector.
    pub const fn first_block(self, track: u8, side: u8) -> usize {
        (track as usize * 2 + side as usize) * self.sectors() as usize
    }
}

/// The most sectors a track side has (HD).
pub const MAX_SECTORS: u8 = Format::Hd.sectors();

const GAP4A: usize = 80;
const GAP1: usize = 50;
const GAP2: usize = 22;
const SYNC: usize = 12;

/// Bytes from the start of the track to the first sector's sync, and from one sector's sync to
/// the next.
const FIRST_SECTOR: usize = GAP4A + SYNC + 4 + GAP1;
const SECTOR_BYTES_NO_GAP3: usize = SYNC + 4 + 4 + 2 + GAP2 + SYNC + 4 + BLOCK + 2;
/// From a sector's sync to the end of its ID field CRC.
const ID_END: usize = SYNC + 4 + 4 + 2;

/// The sector whose ID field last passed before cell `pos` of a track from [`build_track`]:
/// the sector a write starting at `pos` is for. A write starts about 22 bytes after the ID,
/// so a few bytes of error in `pos` don't matter.
pub fn sector_at(format: Format, pos: usize) -> u8 {
    let byte = pos / 16 + 4; // allow for `pos` being a little early
    let sectors = format.sectors();
    match byte.checked_sub(FIRST_SECTOR + ID_END) {
        Some(b) => (b / format.sector_bytes()).min(sectors as usize - 1) as u8 + 1,
        None => sectors, // before the first ID: still the last sector of the previous turn
    }
}

/// CRC-16/CCITT as the ISM computes it: polynomial `$1021`, MSB first.
pub fn crc16(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { crc << 1 ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// MFM encoder on top of the bit packer; remembers the last data bit for the clock rule.
struct Mfm<'a> {
    bits: Bits<'a>,
    prev: bool,
}

impl Mfm<'_> {
    /// Encode one byte as 16 half-cells: clock = NOR(previous data bit, this data bit).
    /// `missing_clock` clears one raw bit to make a mark.
    fn put(&mut self, byte: u8, missing_clock: u16) {
        let mut raw = 0u16;
        for i in (0..8).rev() {
            let d = byte >> i & 1 != 0;
            let c = !(self.prev || d);
            raw = raw << 2 | (c as u16) << 1 | d as u16;
            self.prev = d;
        }
        self.bits.push((raw & !missing_clock).into(), 16);
    }

    fn byte(&mut self, b: u8) {
        self.put(b, 0);
    }

    fn fill(&mut self, b: u8, count: usize) {
        for _ in 0..count {
            self.byte(b);
        }
    }

    /// `$A1` with the clock before data bit 2 dropped: raw `$4489`.
    fn mark_a1(&mut self) {
        self.put(0xA1, 0x0020);
    }

    /// `$C2` with the clock before data bit 3 dropped: raw `$5224`.
    fn mark_c2(&mut self) {
        self.put(0xC2, 0x0080);
    }

    /// Sync run, three `$A1` marks, then the field's mark byte. Returns the CRC so far.
    fn address_mark(&mut self, mark: u8) -> u16 {
        self.fill(0x00, SYNC);
        for _ in 0..3 {
            self.mark_a1();
        }
        self.byte(mark);
        crc16(0xFFFF, &[0xA1, 0xA1, 0xA1, mark])
    }

    fn bytes_written(&self) -> usize {
        self.bits.len() / 16
    }
}

/// Encode one side of `track` into `out`. `sector_data(s)` returns the 512 bytes of sector
/// `s` (1-based). Returns the number of words to stream; always a whole revolution.
pub fn build_track<'a>(
    format: Format,
    track: u8,
    side: u8,
    sector_data: impl Fn(u8) -> &'a [u8; BLOCK],
    out: &mut TrackBuf,
) -> usize {
    // Every byte is 16 half-cells, and the formatted fields must fit in a revolution.
    const { assert!(Format::Dd.fits() && Format::Hd.fits()) };
    build_track_cells(format, track, side, sector_data, format.track_bytes() * 16, out).unwrap_or(0)
}

/// Like [`build_track`], for a track that must be exactly `cells` half-cells long (a MOOF track
/// being reformatted, whose length is fixed in the file). Gap 3 shrinks to fit, down to 20
/// bytes. Returns the number of words, or `None` if it can't fit.
pub fn build_track_cells<'a>(
    format: Format,
    track: u8,
    side: u8,
    sector_data: impl Fn(u8) -> &'a [u8; BLOCK],
    cells: usize,
    out: &mut TrackBuf,
) -> Option<usize> {
    let bytes = cells / 16;
    let per_sector = bytes.checked_sub(FIRST_SECTOR)? / format.sectors() as usize;
    let gap3 = per_sector.checked_sub(SECTOR_BYTES_NO_GAP3)?.min(format.gap3());
    if gap3 < 20 || cells > MAX_TRACK_WORDS * 32 {
        return None;
    }
    let mut m = Mfm { bits: Bits::new(out), prev: false };

    // Gap 4a, then the index mark.
    m.fill(0x4E, GAP4A);
    m.fill(0x00, SYNC);
    for _ in 0..3 {
        m.mark_c2();
    }
    m.byte(0xFC);
    m.fill(0x4E, GAP1);

    for sector in 1..=format.sectors() {
        let id = [track, side, sector, 2]; // size code 2 = 512 bytes
        let crc = crc16(m.address_mark(0xFE), &id);
        for b in id.into_iter().chain(crc.to_be_bytes()) {
            m.byte(b);
        }
        m.fill(0x4E, GAP2);

        let data = sector_data(sector);
        let crc = crc16(m.address_mark(0xFB), data);
        for &b in data.iter().chain(&crc.to_be_bytes()) {
            m.byte(b);
        }
        m.fill(0x4E, gap3);
    }

    // Gap 4b runs up to the end of the revolution, which then wraps into gap 4a. Odd cells at
    // the end are the start of one more `4E`.
    let rest = bytes - m.bytes_written();
    m.fill(0x4E, rest);
    let odd = (cells % 16) as u32;
    if odd > 0 {
        m.bits.push(0x9254 >> (16 - odd), odd);
    }
    Some(m.bits.finish())
}
