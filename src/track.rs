//! Track buffers shared by every format: a track side is a loop of bit cells, `1` = flux
//! reversal, packed MSB-first into `u32` words so the PIO state machine can shift them straight
//! out (see `drive.rs`).

/// Bytes per disk block / SD card sector.
pub const BLOCK: usize = 512;
/// Tracks per side, for every supported format.
pub const TRACKS: u8 = 80;

/// Blocks of one MOOF track side that fit a track buffer: enough for an MFM track of up to
/// 204,800 cells (Applesauce's 1.44 MB tracks are ~193,000). GCR tracks are at most 19 blocks.
pub const MAX_TRACK_BLOCKS: usize = 50;

/// Words in one track buffer: a whole MOOF track side of `MAX_TRACK_BLOCKS`. An encoded 1.44 MB
/// MFM track needs 6,250 (200 ms at 1 µs per half-cell), a GCR track at most 2,333 encoded or
/// 2,432 from a MOOF.
pub const MAX_TRACK_WORDS: usize = MAX_TRACK_BLOCKS * BLOCK / 4;

pub type TrackBuf = [u32; MAX_TRACK_WORDS];

/// Card blocks held for the current cylinder: two MOOF track sides (side `s` at
/// `s * MAX_TRACK_BLOCKS`), or a raw cylinder (side 0's sectors, then side 1's; at most
/// 2 × 18 for 1.44 MB).
pub const MAX_CYLINDER_BLOCKS: usize = 2 * MAX_TRACK_BLOCKS;

/// MSB-first bit packer.
pub struct Bits<'a> {
    words: &'a mut [u32],
    /// Whole words written.
    len: usize,
    /// Pending bits, right-aligned; the low `pending` bits are valid.
    acc: u64,
    pending: u32,
}

impl<'a> Bits<'a> {
    pub fn new(words: &'a mut [u32]) -> Self {
        Self { words, len: 0, acc: 0, pending: 0 }
    }

    /// Append the low `count` (<= 32) bits of `value`.
    pub fn push(&mut self, value: u32, count: u32) {
        self.acc = self.acc << count | u64::from(value) & ((1 << count) - 1);
        self.pending += count;
        if self.pending >= 32 {
            self.pending -= 32;
            self.words[self.len] = (self.acc >> self.pending) as u32;
            self.len += 1;
        }
    }

    /// Bits written so far.
    pub fn len(&self) -> usize {
        self.len * 32 + self.pending as usize
    }

    /// Pad the last word with 1s and return the number of words. In a GCR sync gap the extra
    /// 1s only lengthen one sync nibble; the formats that use this end their tracks in a gap.
    pub fn finish(mut self) -> usize {
        if self.pending > 0 {
            self.push(u32::MAX, 32 - self.pending);
        }
        self.len
    }
}
