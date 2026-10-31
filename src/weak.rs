//! Weak bits: places on a track that a real drive reads differently on every pass, which copy
//! protection checks by reading them more than once.
//!
//! Two kinds, found when a MOOF track is built (see `Cylinder::build`):
//! - a **no-flux run**: a stretch with no reversals, longer than any real data has. A drive's
//!   read amplifier turns up its gain there and returns noise. A bitstream MOOF records it as a
//!   run of 0 cells (Skyfox's track 1 has 85 of 16 cells). After its first few cells, the run is
//!   played as random bits.
//! - a **fuzzy reversal**: in a flux track, a reversal written between two cells, so the drive
//!   sees it in one or the other (Oids' track 1, sector 5). `moof::FluxCells` reports these; one
//!   of the two cells is picked at random. The track length doesn't change.
//!
//! Core 0 re-randomizes the playing buffers' spots every few tens of ms ([`WeakList::randomize`]).
//! Only the playing copy changes; the stored track keeps its zeros.
//!
//! No hardware dependencies, so the host tests compile it directly.

/// Spots kept per track side; any more are dropped (counted in `dropped`).
pub const MAX_SPOTS: usize = 512;

/// A no-flux run counts as weak from this long, in cells; its first `keep` cells stay 0 (the
/// drive's gain takes a while to rise). About 16 and 8 µs: 2.04 µs GCR cells, 1 µs MFM
/// half-cells. Valid data never has more than 2 (GCR) or 3 (MFM) zeros in a row; gaps on real
/// disks have up to 5. Guesses until tuned on hardware.
pub const GCR_RUN: (usize, usize) = (8, 4);
pub const MFM_RUN: (usize, usize) = (16, 8);
/// The same for a Disk II's 4 µs cells: its MC3470 read amplifier makes up bits once the last 4
/// cells have been 0 (2 zeros, 12 µs between reversals, are safe, 3 "fairly
/// safe"). Valid data has at most 2 in a row.
pub const A2_RUN: (usize, usize) = (4, 3);

/// Packing of a spot in a `u32`: a cell index in bits 0–17 (tracks are at most 204,800 cells).
const KIND_FUZZY: u32 = 1 << 31;
const CELL_MASK: u32 = (1 << 18) - 1;
/// Run: length in bits 18–30. Fuzzy: bit 18 set if the other cell is the next one, else the
/// previous one.
const LEN_SHIFT: u32 = 18;
const MAX_RUN: usize = (1 << 13) - 1;
const LATER: u32 = 1 << 18;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Spot {
    /// Cells `start..start + len` are random.
    Run { start: usize, len: usize },
    /// One reversal, at cell `at` or `other`.
    Fuzzy { at: usize, other: usize },
}

/// The weak spots of one track buffer.
pub struct WeakList {
    spots: [u32; MAX_SPOTS],
    len: usize,
    /// Spots that didn't fit.
    pub dropped: u32,
}

impl WeakList {
    pub const fn new() -> Self {
        Self { spots: [0; MAX_SPOTS], len: 0, dropped: 0 }
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.dropped = 0;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    fn push(&mut self, s: u32) {
        if self.len < MAX_SPOTS {
            self.spots[self.len] = s;
            self.len += 1;
        } else {
            self.dropped += 1;
        }
    }

    pub fn push_run(&mut self, mut start: usize, mut len: usize) {
        while len > 0 {
            let n = len.min(MAX_RUN);
            self.push(start as u32 & CELL_MASK | (n as u32) << LEN_SHIFT);
            start += n;
            len -= n;
        }
    }

    /// A reversal at cell `at` that could be in the next cell (`later`) or the previous one.
    pub fn push_fuzzy(&mut self, at: usize, later: bool) {
        self.push(KIND_FUZZY | at as u32 & CELL_MASK | if later { LATER } else { 0 });
    }

    pub fn iter(&self) -> impl Iterator<Item = Spot> + '_ {
        self.spots[..self.len].iter().map(|&s| {
            let cell = (s & CELL_MASK) as usize;
            if s & KIND_FUZZY == 0 {
                Spot::Run { start: cell, len: ((s & !KIND_FUZZY) >> LEN_SHIFT) as usize }
            } else {
                Spot::Fuzzy { at: cell, other: if s & LATER != 0 { cell + 1 } else { cell.wrapping_sub(1) } }
            }
        })
    }

    /// Add the no-flux runs in the first `cells` cells of `buf` (MSB first): every run of at
    /// least `run.0` zeros, minus its first `run.1` cells. The loop's wrap isn't followed: the
    /// padding there is never all zeros.
    pub fn find_runs(&mut self, buf: &[u32], cells: usize, run: (usize, usize)) {
        let (min, keep) = run;
        let mut last_one: Option<usize> = None;
        let check = |this: &mut Self, from: usize, to: usize| {
            // Zeros from `from` up to (not including) `to`.
            if to - from >= min {
                this.push_run(from + keep, to - from - keep);
            }
        };
        for (w, &word) in buf[..cells.div_ceil(32)].iter().enumerate() {
            let mut bits = word;
            while bits != 0 {
                let i = w * 32 + bits.leading_zeros() as usize;
                if i >= cells {
                    break;
                }
                check(self, last_one.map_or(0, |l| l + 1), i);
                last_one = Some(i);
                bits &= !(0x8000_0000 >> (i % 32));
            }
        }
        check(self, last_one.map_or(0, |l| l + 1), cells);
    }

    /// Add fuzzy reversals from `stored` (cells of the stored track, `n` long) for a buffer
    /// rotated by `rotation` (buffer cell k = stored cell k + rotation). One whose other cell
    /// holds a reversal of its own in `buf` is skipped: moving it would merge the two.
    pub fn add_fuzzy(&mut self, stored: &WeakList, n: usize, rotation: usize, buf: &[u32]) {
        let bit = |i: usize| buf[i / 32] >> (31 - i % 32) & 1 != 0;
        for spot in stored.iter() {
            if let Spot::Fuzzy { at, other } = spot {
                if other >= n {
                    continue; // across the wrap
                }
                let at = (at + n - rotation) % n;
                let other = (other + n - rotation) % n;
                if !bit(other) && other.abs_diff(at) == 1 {
                    self.push_fuzzy(at, other > at);
                }
            }
        }
    }

    /// Give every spot in `buf` a new random value. `buf` may be playing: each word is written
    /// in one store, and a read that sees half old, half new bits only sees different noise.
    pub fn randomize(&self, buf: &mut [u32], rng: &mut Rng) {
        for spot in self.iter() {
            match spot {
                Spot::Run { start, len } => {
                    let end = start + len;
                    let mut i = start;
                    while i < end {
                        let w = i / 32;
                        let first = i % 32;
                        let last = (end - w * 32).min(32); // exclusive, within this word
                        let mask = (u32::MAX >> first) & !(u32::MAX.checked_shr(last as u32).unwrap_or(0));
                        buf[w] = buf[w] & !mask | rng.noise() & mask;
                        i = w * 32 + last;
                    }
                }
                Spot::Fuzzy { at, other } => {
                    let (on, off) = if rng.next() & 1 != 0 { (at, other) } else { (other, at) };
                    buf[on / 32] |= 0x8000_0000 >> (on % 32);
                    buf[off / 32] &= !(0x8000_0000 >> (off % 32));
                }
            }
        }
    }
}

/// xorshift32.
pub struct Rng(u32);

impl Rng {
    pub const fn new(seed: u32) -> Self {
        Self(if seed == 0 { 0x2545_F491 } else { seed })
    }

    pub fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }

    /// 32 noise cells, about 31% ones (a ∧ (b ∨ (c ∧ d))), near what a drive returns from an
    /// unrecorded stretch.
    pub fn noise(&mut self) -> u32 {
        let (a, b, c, d) = (self.next(), self.next(), self.next(), self.next());
        a & (b | c & d)
    }
}
