//! DiskCopy 4.2 images (`.dc42`, often `.image`): an 84-byte header, the sector data in raw
//! image order, then 12 bytes of tags per sector (or none).
//!
//! Header, all big-endian:
//! - `0x00` disk name, a Pascal string of up to 63 bytes;
//! - `0x40` data size, `0x44` tag size;
//! - `0x48` data checksum, `0x4C` tag checksum;
//! - `0x50` disk format (0 = 400K, 1 = 800K, 2 = 720K, 3 = 1.44 MB), `0x51` format byte
//!   (`$12` 400K, `$22` 800K Mac or MFM), `0x52` `$0100`.
//!
//! The data is played like a raw image whose block `i` starts at file byte `84 + 512 i`, so each
//! block straddles two file blocks; [`shift_sector`] puts it back together and [`place_sector`]
//! takes it apart again for saving. Tags are ignored (the GCR encoder writes zero tags, as for raw
//! images, and writes never touch the tag area, so the tag checksum stays right).
//!
//! The data checksum covers every data byte, and each step rotates, so a write can't patch it:
//! after saving, [`ChecksumJob`] recomputes it from the card a few blocks at a time and the new
//! value goes into the header ([`DATA_CHECKSUM`]).
//!
//! No hardware dependencies, so the host tests compile it directly.

use crate::track::BLOCK;

/// Bytes before the sector data.
pub const HEADER_BYTES: usize = 84;

/// Where the data checksum is in the header (big-endian).
pub const DATA_CHECKSUM: core::ops::Range<usize> = 0x48..0x4C;

/// Sector data sizes this emulator plays: 400K, 800K, 720K, 1.44 MB.
pub const DATA_SIZES: [usize; 4] = [409_600, 819_200, 737_280, 1_474_560];

/// Tag bytes per 512-byte sector.
const TAG_BYTES: usize = 12;

/// The sector data size of a DiskCopy 4.2 file of `file_size` bytes, if it has one we play
/// (with or without tags).
pub fn data_size_for(file_size: u64) -> Option<usize> {
    DATA_SIZES.into_iter().find(|&data| {
        let base = (HEADER_BYTES + data) as u64;
        file_size == base || file_size == base + (data / BLOCK * TAG_BYTES) as u64
    })
}

pub struct Header {
    pub data_size: usize,
    pub data_checksum: u32,
}

impl Header {
    /// Parse and check the header of a `file_size`-byte file.
    pub fn parse(h: &[u8; HEADER_BYTES], file_size: u64) -> Result<Self, &'static str> {
        let be32 = |at: usize| u32::from_be_bytes([h[at], h[at + 1], h[at + 2], h[at + 3]]);
        let data_size = be32(0x40) as usize;
        let tag_size = be32(0x44) as usize;
        if h[0] > 63 {
            return Err("DC42: bad header");
        }
        if !DATA_SIZES.contains(&data_size) {
            return Err("DC42: unsupported disk size");
        }
        if (HEADER_BYTES + data_size + tag_size) as u64 != file_size {
            return Err("DC42: size doesn't match header");
        }
        let format = match data_size {
            409_600 => 0,
            819_200 => 1,
            737_280 => 2,
            _ => 3,
        };
        if h[0x50] != format {
            return Err("DC42: format doesn't match size");
        }
        Ok(Self { data_size, data_checksum: be32(0x48) })
    }
}

/// DiskCopy's checksum: for each big-endian 16-bit word, add it, then rotate right by one.
pub struct Checksum {
    sum: u32,
}

impl Checksum {
    pub fn new() -> Self {
        Self { sum: 0 }
    }

    /// Feed data bytes in order. Each call must hold whole words (even length).
    pub fn update(&mut self, data: &[u8]) {
        for w in data.chunks_exact(2) {
            self.sum = self.sum.wrapping_add(u32::from(u16::from_be_bytes([w[0], w[1]]))).rotate_right(1);
        }
    }

    pub fn finish(&self) -> u32 {
        self.sum
    }
}

/// Turn file block `this` into the sector that starts `offset` bytes into it, the rest coming
/// from the next file block.
pub fn shift_sector(this: &mut [u8; BLOCK], next: &[u8; BLOCK], offset: usize) {
    this.copy_within(offset.., 0);
    this[BLOCK - offset..].copy_from_slice(&next[..offset]);
}

/// The reverse of [`shift_sector`]: store `sector` starting `offset` bytes into file block `this`,
/// the rest at the start of `next`. Other bytes of both blocks are kept.
pub fn place_sector(this: &mut [u8; BLOCK], next: &mut [u8; BLOCK], sector: &[u8; BLOCK], offset: usize) {
    this[offset..].copy_from_slice(&sector[..BLOCK - offset]);
    next[..offset].copy_from_slice(&sector[BLOCK - offset..]);
}

/// The data checksum of an image on the card, worked out one file block at a time.
pub struct ChecksumJob {
    sum: Checksum,
    /// The next file block to feed.
    next: usize,
    data_size: usize,
}

impl ChecksumJob {
    pub fn new(data_size: usize) -> Self {
        Self { sum: Checksum::new(), next: 0, data_size }
    }

    /// The file block to read and [`feed`](Self::feed) next, or `None` once all the data is in.
    pub fn next_block(&self) -> Option<usize> {
        (self.next * BLOCK < HEADER_BYTES + self.data_size).then_some(self.next)
    }

    /// Feed file block [`next_block`](Self::next_block): its data bytes, if any, go into the sum.
    pub fn feed(&mut self, block: &[u8; BLOCK]) {
        let at = self.next * BLOCK;
        let start = at.max(HEADER_BYTES);
        let end = (at + BLOCK).min(HEADER_BYTES + self.data_size);
        if start < end {
            // Even offsets and lengths: whole words.
            self.sum.update(&block[start - at..end - at]);
        }
        self.next += 1;
    }

    pub fn finish(&self) -> u32 {
        self.sum.finish()
    }
}
