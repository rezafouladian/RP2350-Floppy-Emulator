//! MOOF (Applesauce) bitstream images: 400K/800K GCR and 1.44 MB MFM.
//!
//! A MOOF stores one revolution of raw cells per track side, so there is nothing to encode:
//! a track's bits are copied into a track buffer (see `track.rs`) and played as they are. GCR
//! tracks are rotated so the word-aligned loop wraps in a sync gap; MFM tracks start at the
//! index, in gap 4a, and are played from there.
//!
//! Tracks listed in the optional FLUX map hold flux timings instead of cells. [`FluxCells`]
//! turns them into cells as they are read, after which they play like any other track.
//!
//! A WOZ 2 of a 3.5" disk (an Apple II's 400K/800K GCR) is the same file but for its signature
//! and where INFO keeps its fields: the TMAP is per track side (track × 2 + side), and the TRK
//! records, FLUX map (WOZ 2.1) and cells are MOOF's. So [`Moof::parse`] reads those too
//! ([`Moof::woz`]).

use crate::track::{BLOCK, MAX_TRACK_WORDS, TRACKS, TrackBuf};

/// Blocks 0–2: file header, INFO, TMAP and the TRKS record table. Track data starts after.
pub const HEADER_BYTES: usize = 3 * BLOCK;

const SIGNATURE: [u8; 8] = *b"MOOF\xFF\x0A\x0D\x0A";
const WOZ2_SIGNATURE: [u8; 8] = *b"WOZ2\xFF\x0A\x0D\x0A";
/// INFO "optimal bit timing" in 125 ns units: GCR cells are 2 µs, MFM half-cells 1 µs.
const GCR_BIT_TIMING: u8 = 16;
const MFM_BIT_TIMING: u8 = 8;
pub const TRK_COUNT: usize = 160;
const BLANK: u8 = 0xFF;

/// One TRK record: where a track side's bits are in the file.
#[derive(Clone, Copy, Default)]
pub struct Trk {
    /// First file block.
    pub start: u16,
    pub blocks: u16,
    /// Bit cells in one revolution, or for a flux track, bytes of flux data.
    pub bits: u32,
    /// Flux timings rather than cells (the record is in the FLUX map).
    pub flux: bool,
}

pub struct Moof {
    /// A WOZ 2 of an Apple II 3.5" disk rather than a MOOF.
    pub woz: bool,
    /// 1 = 400K single-sided, 2 = 800K or 1.44 MB double-sided.
    pub sides: u8,
    /// 1.44 MB MFM (disk type 3) rather than GCR.
    pub mfm: bool,
    pub write_protected: bool,
    /// INFO "synchronized": imaged with cross-track sync, so every track starts at the same
    /// angle. Protections that time one track against another rely on it.
    pub synchronized: bool,
    /// Some tracks are flux. Such images are played read-only: a write can't be spliced into
    /// flux timings.
    pub has_flux: bool,
    /// CRC-32 stored in the header, 0 = not computed.
    pub stored_crc: u32,
    tmap: [u8; TRK_COUNT],
    /// The FLUX map, same layout as TMAP; its entries win over TMAP's.
    flux: [u8; TRK_COUNT],
    trks: [Trk; TRK_COUNT],
}

fn u16_at(h: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([h[at], h[at + 1]])
}

fn u32_at(h: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([h[at], h[at + 1], h[at + 2], h[at + 3]])
}

impl Moof {
    /// The file block holding the FLUX chunk, if the header says there is one: both FLUX Block
    /// and Largest Flux Track set, in INFO version 1 of a MOOF or version 3 of a WOZ (2.1).
    pub fn flux_block(h: &[u8]) -> Option<usize> {
        let (at, version) = if h[..8] == WOZ2_SIGNATURE { (66, 3) } else { (60, 1) };
        let block = u16_at(h, at);
        (h[20] >= version && block != 0 && u16_at(h, at + 2) != 0).then_some(block as usize)
    }

    /// Parse and check the first three blocks of a `file_blocks`-block file, and the FLUX chunk's
    /// block if [`flux_block`](Self::flux_block) names one. Every mapped track must lie inside
    /// the file. Bitstream tracks must also fit `max_track_blocks` and a track buffer; flux tracks
    /// are converted as they're read, so only their cells must fit (see [`FluxCells`]).
    pub fn parse(
        h: &[u8; HEADER_BYTES],
        flux_chunk: Option<&[u8; BLOCK]>,
        file_blocks: usize,
        max_track_blocks: usize,
    ) -> Result<Self, &'static str> {
        let woz = h[..8] == WOZ2_SIGNATURE;
        // The same complaint, about whichever it is.
        let err = |moof: &'static str, woz_text: &'static str| if woz { woz_text } else { moof };
        if h[..8] != SIGNATURE && !woz {
            return Err("MOOF: bad signature");
        }
        if &h[12..16] != b"INFO" || &h[80..84] != b"TMAP" || &h[248..252] != b"TRKS" {
            return Err(err("MOOF: unexpected chunk layout", "WOZ: unexpected chunk layout"));
        }
        // INFO data starts at byte 20.
        if h[20] < 1 {
            return Err(err("MOOF: bad INFO version", "WOZ: bad INFO version"));
        }
        let (sides, mfm, timing) = if woz {
            match h[21] {
                2 => {}
                1 => return Err("WOZ: 5.25\" disk (use mode = apple2)"),
                _ => return Err("WOZ: unsupported disk type"),
            }
            // INFO version 2 added the sides and the bit timing. Before it, a side 1 in the TMAP
            // means two.
            if h[20] >= 2 {
                (h[57], false, h[59])
            } else {
                let two = h[88..88 + TRK_COUNT].iter().skip(1).step_by(2).any(|&k| k != BLANK);
                (1 + two as u8, false, GCR_BIT_TIMING)
            }
        } else {
            match h[21] {
                1 => (1, false, h[24]),
                2 => (2, false, h[24]),
                3 => (2, true, h[24]),
                4 => return Err("MOOF: Twiggy not supported"),
                _ => return Err("MOOF: unsupported disk type"),
            }
        };
        if !(1..=2).contains(&sides) {
            return Err("WOZ: bad disk sides");
        }
        if timing != if mfm { MFM_BIT_TIMING } else { GCR_BIT_TIMING } {
            return Err(err("MOOF: unsupported bit timing", "WOZ: unsupported bit timing"));
        }
        let mut moof = Moof {
            woz,
            sides,
            mfm,
            write_protected: h[22] != 0,
            synchronized: h[23] != 0,
            has_flux: false,
            stored_crc: u32_at(h, 8),
            tmap: [BLANK; TRK_COUNT],
            flux: [BLANK; TRK_COUNT],
            trks: [Trk::default(); TRK_COUNT],
        };
        moof.tmap.copy_from_slice(&h[88..88 + TRK_COUNT]);
        for (i, trk) in moof.trks.iter_mut().enumerate() {
            let r = 256 + i * 8;
            *trk = Trk { start: u16_at(h, r), blocks: u16_at(h, r + 2), bits: u32_at(h, r + 4), flux: false };
        }
        if Self::flux_block(h).is_some() {
            // The chunk starts at its block: ID, size, then the map.
            let c = flux_chunk.ok_or(err("MOOF: FLUX chunk missing", "WOZ: FLUX chunk missing"))?;
            if &c[..4] != b"FLUX" || u32_at(c, 4) < TRK_COUNT as u32 {
                return Err(err("MOOF: bad FLUX chunk", "WOZ: bad FLUX chunk"));
            }
            moof.flux.copy_from_slice(&c[8..8 + TRK_COUNT]);
            for &index in &moof.flux {
                if index != BLANK {
                    moof.trks.get_mut(index as usize).ok_or(err("MOOF: bad FLUX entry", "WOZ: bad FLUX entry"))?.flux = true;
                    moof.has_flux = true;
                }
            }
        }
        for t in 0..TRK_COUNT as u8 / 2 {
            for side in 0..2 {
                let Some((_, trk)) = moof.track(t, side) else { continue };
                let (start, blocks) = (trk.start as usize, trk.blocks as usize);
                let bits_max = if trk.flux { blocks * BLOCK } else { blocks * BLOCK * 8 };
                if start < 3 || start + blocks > file_blocks || trk.bits == 0 || trk.bits as usize > bits_max {
                    return Err(err("MOOF: bad TRK record", "WOZ: bad TRK record"));
                }
                if !trk.flux && (blocks > max_track_blocks || (trk.bits as usize).div_ceil(32) > MAX_TRACK_WORDS) {
                    return Err(err("MOOF: track too long", "WOZ: track too long"));
                }
            }
        }
        Ok(moof)
    }

    /// The TRK record for a track side, or `None` for a blank (unformatted) side.
    pub fn track(&self, track: u8, side: u8) -> Option<(usize, Trk)> {
        if track >= TRACKS {
            return None;
        }
        let k = track as usize * 2 + side as usize;
        match (self.flux[k], self.tmap[k]) {
            (BLANK, BLANK) => None,
            (BLANK, i) | (i, _) => self.trks.get(i as usize).map(|trk| (i as usize, *trk)),
        }
    }
}

/// Turns a flux track into cells as its bytes arrive. Each byte is the ticks since the previous
/// reversal; 255 means "add 255 and keep reading". Each interval becomes `n` = round(ticks / cell)
/// cells, `n − 1` zeros then a 1. Rounding each interval on its own, rather
/// than each reversal's absolute time, keeps the cell counts right when the imaging drive's
/// speed was a little off. An interval under half a cell (noise) is merged into the next one.
///
/// An interval that lands next to a half cell (at least [`FUZZY`] off a whole count) means its
/// reversal sits on the IWM's boundary between two cell counts, so a Mac reads it either way,
/// differently on each pass (fuzzy bits, as on Oids). It's reported with the other cell. The
/// interval after such a reversal is also measured from its true place: if that puts the next
/// reversal clearly on the grid, it goes there and isn't reported (otherwise rounding the same
/// way twice would move every later cell by one). Everything else is placed by rounding alone,
/// so a run of messy intervals can't drift.
///
/// The cell is the one the Mac reads at: 2.0425 µs for GCR (16.34 ticks), 1 µs half-cells
/// for MFM.
pub struct FluxCells {
    /// Cell length in 1/100 tick.
    cell: u32,
    /// Ticks since the last reversal that made a cell.
    pending: u32,
    /// The last interval's offset from a whole count, in 1/100 cell, if it was at least
    /// [`FUZZY`]; else 0.
    last_off: i32,
    cells: usize,
    /// Ticks fed so far, a time to note the cells at ([`watch`](Self::watch)) and the cells then.
    ticks: u64,
    watch: u64,
    at_watch: usize,
}

/// Longest interval counted, in ticks (5 s), and the longest counted in 1/100 cell (50 ms):
/// `pending * 10_000` stays in 32 bits (the M33 divides those in hardware) up to that, and a longer
/// one is a stretch with no flux (part of a track blank, as on Choplifter's track 0: 84 ms),
/// counted in whole cells. Capping it at 50 ms instead made such a revolution 34 ms short.
const MAX_PENDING: u32 = 40_000_000;
const MAX_FINE: u32 = 400_000;

/// An interval at least this far from a whole number of cells, in 1/100 cell, is fuzzy. The
/// Mac's IWM (fast mode, 16 FCLK per cell) counts FCLKs between reversals and splits cell counts
/// at 1.5 and 2.5 cells, and its count depends on its clock phase by about one
/// FCLK (1/16 cell). Oids' fuzzy intervals are 25, 40 and 41 ticks (1.53, 2.45, 2.51 cells);
/// clean real flux (an A2R) has a few this close too, at most 9 on a side.
pub const FUZZY: i32 = 44;

/// Where the other cell of a fuzzy reversal is.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Other {
    None,
    Earlier,
    Later,
}

impl FluxCells {
    pub fn new(mfm: bool) -> Self {
        // In 1/100 of a 125 ns tick: 2.0425 µs = 16.34 ticks, 1 µs = 8 ticks.
        Self::with_cell(if mfm { 800 } else { 1_634 })
    }

    /// With a cell of `cell` hundredths of a tick: an Apple II 5.25" track's, as measured on it
    /// ([`measure_cell`]; nominally 4 µs, 3,200).
    pub fn with_cell(cell: u32) -> Self {
        Self { cell: cell.max(100), pending: 0, last_off: 0, cells: 0, ticks: 0, watch: 0, at_watch: 0 }
    }

    /// Note the cells made by the reversal at or after `ticks` from the start ([`at_watch`]):
    /// an A2R capture's loop point, as cells. (Not the loop point over the measured cell: a 5.25"
    /// track's sectors may have been written at speeds a few percent apart.)
    ///
    /// [`at_watch`]: Self::at_watch
    pub fn watch(&mut self, ticks: u64) {
        self.watch = ticks;
    }

    /// The cells at the time [`watch`](Self::watch) named, or 0 if not reached.
    pub fn at_watch(&self) -> usize {
        if self.at_watch == usize::MAX { 0 } else { self.at_watch }
    }

    /// Feed the next flux bytes. `one(i, other)` is called for each cell `i` holding a reversal,
    /// in increasing order; every other cell is a 0. `other` says whether the reversal is fuzzy,
    /// and which way its other cell is.
    pub fn feed(&mut self, bytes: &[u8], mut one: impl FnMut(usize, Other)) {
        for &b in bytes {
            self.pending = self.pending.saturating_add(u32::from(b)).min(MAX_PENDING);
            self.ticks += u64::from(b);
            if b == 255 {
                continue;
            }
            if self.watch > 0 && self.at_watch == 0 && self.ticks >= self.watch {
                // This reversal's cells are counted below; `cells` then ends with it.
                self.at_watch = usize::MAX;
            }
            // The interval in 1/100 cell (a long one in whole cells: never fuzzy).
            let x = if self.pending <= MAX_FINE {
                (self.pending * 10_000 / self.cell) as i32
            } else {
                ((self.pending * 100 + self.cell / 2) / self.cell).min(1_000_000) as i32 * 100
            };
            // From the last reversal's true place, if it was off the grid: back on it?
            let from_true = x + self.last_off;
            let n_true = (from_true + 50) / 100;
            let (n, off, other) = if self.last_off != 0 && n_true >= 1 && (from_true - n_true * 100).abs() < FUZZY {
                (n_true, 0, Other::None)
            } else {
                let n = (x + 50) / 100;
                if n == 0 {
                    continue;
                }
                let off = x - n * 100;
                // An earlier cell must still come after the previous reversal.
                let other = match (off.abs() >= FUZZY, off > 0) {
                    (true, true) => Other::Later,
                    (true, false) if n >= 2 => Other::Earlier,
                    _ => Other::None,
                };
                (n, off, other)
            };
            self.last_off = if off.abs() >= FUZZY { off } else { 0 };
            self.cells += n as usize;
            if self.at_watch == usize::MAX {
                self.at_watch = self.cells;
            }
            one(self.cells - 1, other);
            self.pending = 0;
        }
    }

    /// Cells in the revolution so far.
    pub fn cells(&self) -> usize {
        self.cells
    }

    /// The cell, in 1/100 tick.
    pub fn cell(&self) -> u32 {
        self.cell
    }
}

/// The cell of a track's flux `bytes` (the start of it will do), in 1/100 tick: the mean of its
/// one-cell intervals, those within a quarter of `nominal` (1/100 tick). A 5.25" disk's tracks
/// were written at the speed of whichever drive wrote them, a few percent apart (and some, as a
/// protection, far more), so a fixed cell would put some reversals on the boundary between two
/// counts. `nominal` if there are too few to tell.
pub fn measure_cell(bytes: &[u8], nominal: u32) -> u32 {
    let (lo, hi) = (nominal * 3 / 4, nominal * 5 / 4);
    let (mut pending, mut sum, mut n) = (0u32, 0u64, 0u32);
    for &b in bytes {
        pending = pending.saturating_add(u32::from(b));
        if b == 255 {
            continue;
        }
        let t = pending * 100;
        if (lo..hi).contains(&t) {
            sum += u64::from(t);
            n += 1;
        }
        pending = 0;
    }
    if n < 200 { nominal } else { (sum / u64::from(n)) as u32 }
}

/// CRC-32 as used in the MOOF header: reflected `$EDB88320`, same as zlib `crc32`.
pub struct Crc32(u32);

const CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { c >> 1 ^ 0xEDB8_8320 } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

impl Crc32 {
    pub fn new() -> Self {
        Self(!0)
    }

    pub fn update(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = CRC_TABLE[((self.0 ^ u32::from(b)) & 0xFF) as usize] ^ self.0 >> 8;
        }
    }

    pub fn finish(&self) -> u32 {
        !self.0
    }
}

/// Track bits stored MSB first; `byte(i)` returns byte `i` of the track data.
struct Stream<F> {
    byte: F,
    bits: usize,
}

impl<F: Fn(usize) -> u8> Stream<F> {
    fn bit(&self, i: usize) -> u32 {
        u32::from((self.byte)(i / 8) >> (7 - i % 8) & 1)
    }

    /// 32 bits starting at bit `i`, which must not run past the end.
    fn word(&self, i: usize) -> u32 {
        let (at, shift) = (i / 8, i % 8);
        let mut w = 0u64;
        for k in 0..5 {
            // The fifth byte is only needed for an unaligned start.
            let b = if k < 4 || shift != 0 { (self.byte)(at + k) } else { 0 };
            w = w << 8 | u64::from(b);
        }
        (w >> (8 - shift)) as u32
    }
}

/// Where the loop should wrap: the middle of the longest gap from a data field's trailer
/// (`DE AA` after a `D5 AA AD` data mark) to the next address mark (`D5 AA 96`, or `D5 AA B5`
/// on a 13-sector Apple II track), as an IWM would frame the bits. Runs the framer over two revolutions so a gap across the wrap counts too. A
/// track with no such gap wraps in the middle of its longest run of `FF` nibbles, or at 0.
///
/// The padding at the wrap and every stream restart land here, so it must be filler. Only a data
/// trailer starts a gap: address fields end in `DE AA` too, and on Skyfox's track 1 a sector with
/// no data field has the protection's timing records after its address field. The filler itself
/// isn't checked: most disks use `FF` sync, but Dollars and Sense's gaps are `AA`.
///
/// The result is a nibble boundary (the framer is empty there), so [`Pad::Gcr`]'s whole nibbles
/// leave the framing after the wrap as it was. That matters where there are no 10-bit syncs to
/// resync on: The Ancient Art of War's track 79 has no sectors, only 8-bit `FF`s, and its
/// longest `FF` run ends 22 cells before the `AA FF FF` its protection counts nibbles to. Extra
/// 1s there shifted the framing and the mark was never seen.
pub fn find_rotation(byte: impl Fn(usize) -> u8, bits: u32) -> u32 {
    let s = Stream { byte, bits: bits as usize };
    let n = s.bits;
    let mut shifter = 0u8;
    let mut recent = 0u32;
    // Inside a data field (after its mark, before its trailer), and where its trailer ended.
    let mut in_data = false;
    let mut trailer_end: Option<usize> = None;
    // (length, first boundary, middle), unwrapped: the middle may be in the second revolution.
    let mut best_gap = (0usize, 0usize, 0usize);
    // Fallback: the current and longest run of FF nibbles.
    let mut nibble_start = 0usize;
    let mut run_start: Option<usize> = None;
    let mut best_run = (0usize, 0usize, 0usize);
    for i in 0..2 * n {
        shifter = shifter << 1 | s.bit(i % n) as u8;
        if shifter & 0x80 == 0 {
            continue;
        }
        recent = recent << 8 | u32::from(shifter);
        if recent & 0xFF_FFFF == 0xD5_AAAD {
            in_data = true;
        } else if recent & 0xFFFF == 0xDEAA && in_data {
            in_data = false;
            trailer_end = Some(i + 1);
        } else if matches!(recent & 0xFF_FFFF, 0xD5_AA96 | 0xD5_AAB5) {
            // An address mark (`D5 AA B5`: a 13-sector Apple II track's).
            in_data = false;
            let mark_start = i + 1 - 24;
            // A gap ending in the second revolution has been seen whole.
            if let Some(t) = trailer_end.take().filter(|&t| t < mark_start && i >= n) {
                let gap = mark_start - t;
                if gap > best_gap.0 {
                    best_gap = (gap, t, t + gap / 2);
                }
            }
        }
        if shifter == 0xFF {
            run_start.get_or_insert(nibble_start);
        } else if let Some(start) = run_start.take() {
            let len = nibble_start - start;
            if i >= n && len > best_run.0 {
                best_run = (len, start, start + len / 2);
            }
        }
        shifter = 0;
        nibble_start = i + 1;
    }
    let (_, from, middle) = if best_gap.0 > 0 { best_gap } else { best_run };
    // Frame on from the gap's first boundary to the first boundary at or past its middle.
    let (mut i, mut shifter) = (from, 0u8);
    while i < middle || shifter != 0 {
        shifter = shifter << 1 | s.bit(i % n) as u8;
        if shifter & 0x80 != 0 {
            shifter = 0;
        }
        i += 1;
    }
    (i % n.max(1)) as u32
}

/// How [`load_track`] fills the loop past a track's cells, up to a whole word.
#[derive(Clone, Copy, PartialEq)]
pub enum Pad {
    /// Whole `FF` nibbles, some led by one or two 0s (8, 9 or 10 cells each), plus a word if
    /// the room left can't be made of them. At a nibble boundary ([`find_rotation`]) an IWM
    /// reads them as extra `FF`s and frames what follows as before. Plain 1s did that only
    /// where 10-bit syncs follow to resync on.
    Gcr,
    /// A reversal every 2 half-cells, which keeps the clock rule in gap 4.
    Mfm,
    /// None: for a player that counts a revolution's cells (the Disk II's, `src/disk2.pio`). The
    /// cells go in one word more than their whole words: the last cells and 0s, or a word of 0s
    /// the player takes and drops.
    None,
}

/// `k` cells of whole `FF` nibbles (8 + 9a + 10b … in any mix), MSB first in the low `k` bits,
/// or `None` if `k` can't be made of them (1–7, 11–15, 21–23, 31). Fewest nibbles: as many
/// 10-cell ones as fit. Plain `FF`s, if any, go first.
fn gcr_fill(k: usize) -> Option<u64> {
    for long in (0..=k / 10).rev() {
        for mid in 0..=(k - 10 * long) / 9 {
            let rest = k - 10 * long - 9 * mid;
            if rest % 8 == 0 {
                let mut v = 0u64;
                for _ in 0..rest / 8 {
                    v = v << 8 | 0xFF;
                }
                for _ in 0..mid {
                    v = v << 9 | 0xFF;
                }
                for _ in 0..long {
                    v = v << 10 | 0xFF;
                }
                return Some(v);
            }
        }
    }
    None
}

/// Copy a track's `bits` cells into `out`, starting `rotation` bits in and wrapping, then fill
/// the rest of the last word (or two, for [`Pad::Gcr`]) per `pad`. Returns the number of words
/// to loop.
pub fn load_track(byte: impl Fn(usize) -> u8, bits: u32, rotation: u32, pad: Pad, out: &mut TrackBuf) -> usize {
    let s = Stream { byte, bits: bits as usize };
    let (n, rot) = (s.bits, rotation as usize % s.bits);
    let mut words = if pad == Pad::None { (n / 32 + 1).min(MAX_TRACK_WORDS) } else { n.div_ceil(32) };
    // The fill's cells, MSB first in the low `room` bits. A track that fills the buffer and needs
    // another word for whole nibbles gets 1s, as before.
    let (fill, room) = match pad {
        Pad::None => (0, words * 32 - n),
        Pad::Mfm => (0x5555_5555, words * 32 - n),
        Pad::Gcr => match gcr_fill(words * 32 - n) {
            Some(v) => (v, words * 32 - n),
            None if words < MAX_TRACK_WORDS => {
                words += 1;
                (gcr_fill(words * 32 - n).unwrap_or(u64::MAX), words * 32 - n)
            }
            None => (u64::MAX, words * 32 - n),
        },
    };
    for (w, slot) in out[..words].iter_mut().enumerate() {
        let k = w * 32;
        let from = (k + rot) % n;
        *slot = if k + 32 <= n && from + 32 <= n {
            s.word(from)
        } else {
            // Across the wrap or the end: bit by bit, the fill past the end.
            (0..32).fold(0, |v, j| {
                let at = k + j;
                v << 1 | if at < n { s.bit((at + rot) % n) } else { (fill >> (room - 1 - (at - n)) & 1) as u32 }
            })
        };
    }
    words
}
