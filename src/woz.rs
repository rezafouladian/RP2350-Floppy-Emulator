//! WOZ (Applesauce) bitstream images of Apple II 5.25" disks, versions 1 and 2.
//!
//! WOZ 2 is laid out like MOOF (which was derived from it): a 12-byte header with a CRC-32,
//! INFO at byte 12, TMAP at 80, TRKS at 248 with 160 8-byte TRK records from byte 256, and the
//! track data in whole blocks from block 3. The TMAP has one entry per quarter track (0–159),
//! naming the TRK record heard there, or `FF` for nothing. WOZ 1 has the same chunks in the same
//! places, but TRKS holds fixed 6,656-byte records instead: 6,646 bytes of cells, then the bytes
//! used, the bit count and splice information. They start 256 bytes into a block.
//!
//! Tracks are played as stored, like MOOF ones (`moof::find_rotation`, `moof::load_track`). WOZ
//! 2.1's FLUX chunk (its block named in INFO v3, captured while mapping) maps quarter tracks to TRK
//! records of flux timings (125 ns ticks, one revolution; Bit Count is the byte count), as MOOF's
//! does. Its entries win over TMAP's; such tracks are converted to cells as they're read
//! (`moof::FluxCells`, at the track's own measured cell). A WOZ with flux tracks is read-only.
//!
//! A WOZ 2 of a 3.5" disk is refused here: its TMAP is per track side, as in a MOOF, so
//! `moof::Moof::parse` reads it for the Apple 3.5" drive (`mode = apple35`).
//!
//! Hardware-free, so the host tests compile it directly.

use crate::moof::Trk;
use crate::track::{BLOCK, MAX_TRACK_WORDS};

/// Blocks 0–2: header, INFO, TMAP and (WOZ 2) the TRK records.
pub const HEADER_BYTES: usize = 3 * BLOCK;
pub const QUARTERS: usize = 160;
const BLANK: u8 = 0xFF;

/// WOZ 1 track records: bytes each, and the offset of the bit count in one.
pub const V1_RECORD: usize = 6_656;
const V1_BIT_COUNT: usize = 6_648;
/// WOZ 1 records start at byte 256 of the file, so record `j` is file blocks `13j..13j + 14`
/// read 256 bytes in.
pub const V1_OFFSET: usize = 256;
pub const V1_BLOCKS: usize = V1_RECORD / BLOCK;

pub struct Woz {
    /// 1 or 2 (WOZ 2.1's INFO version 3 is still format 2).
    pub version: u8,
    pub write_protected: bool,
    /// Imaged with cross-track sync: every track starts at the same angle.
    pub synchronized: bool,
    /// One cell, in 125 ns units: 32 = 4 µs (WOZ 1 has no field: always 32).
    pub bit_timing: u8,
    /// Some quarter tracks are flux (the FLUX map names them).
    pub has_flux: bool,
    /// CRC-32 in the header, 0 = not computed.
    pub stored_crc: u32,
    tmap: [u8; QUARTERS],
    /// The FLUX map, as TMAP: its entries win.
    flux: [u8; QUARTERS],
    trks: [Trk; QUARTERS],
}

fn u16_at(h: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([h[at], h[at + 1]])
}

fn u32_at(h: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([h[at], h[at + 1], h[at + 2], h[at + 3]])
}

/// The PIO clock (16 cycles a cell) that plays a track of `bits` cells in 200 ms, one revolution
/// at 300 RPM, as the disk turned under the head that wrote it: a track written by a drive running
/// fast or slow (or on purpose, as a protection) holds more or fewer bits in the same revolution.
/// Within 20% of `disk_hz` (the disk's average, [`Woz::cell_hz`]); else `disk_hz`, as for a track
/// whose length is no measure of its timing.
pub fn track_hz(bits: u32, disk_hz: u32) -> u32 {
    let hz = bits.saturating_mul(80);
    if hz >= disk_hz / 5 * 4 && hz <= disk_hz / 5 * 6 { hz } else { disk_hz }
}

impl Woz {
    /// The WOZ version from the signature, if it is one.
    pub fn version_of(h: &[u8]) -> Option<u8> {
        match h.get(..8)? {
            b"WOZ1\xFF\x0A\x0D\x0A" => Some(1),
            b"WOZ2\xFF\x0A\x0D\x0A" => Some(2),
            _ => None,
        }
    }

    /// For WOZ 1: which record's bit count is in file block `block`, and where. Record `j`'s is
    /// at byte 248 of block `13j + 13`.
    pub fn v1_bit_count_at(block: usize) -> Option<(usize, usize)> {
        let at = V1_OFFSET + V1_BIT_COUNT;
        let j = block.checked_sub(at / BLOCK)?;
        (j % V1_BLOCKS == 0 && j / V1_BLOCKS < QUARTERS).then_some((j / V1_BLOCKS, at % BLOCK))
    }

    /// Parse and check the first three blocks of a `file_blocks`-block file. `v1_bits[j]` is WOZ 1
    /// record `j`'s bit count, read while mapping (see [`v1_bit_count_at`](Self::v1_bit_count_at));
    /// unused for WOZ 2. `flux_chunk` is the block INFO names for the FLUX chunk
    /// (`moof::Moof::flux_block`), if it names one. Every track in the maps must lie in the file,
    /// and every bitstream track fit a track buffer and `max_track_blocks`.
    pub fn parse(
        h: &[u8; HEADER_BYTES],
        v1_bits: &[u16; QUARTERS],
        flux_chunk: Option<&[u8; BLOCK]>,
        file_blocks: usize,
        max_track_blocks: usize,
    ) -> Result<Self, &'static str> {
        let version = Self::version_of(h).ok_or("WOZ: bad signature")?;
        if &h[12..16] != b"INFO" || &h[80..84] != b"TMAP" || &h[248..252] != b"TRKS" {
            return Err("WOZ: unexpected chunk layout");
        }
        // INFO data starts at byte 20.
        if h[20] < 1 {
            return Err("WOZ: bad INFO version");
        }
        match h[21] {
            1 => {}
            2 => return Err("WOZ: 3.5\" disk (use mode = apple35)"),
            _ => return Err("WOZ: unsupported disk type"),
        }
        let bit_timing = if version >= 2 && h[20] >= 2 { h[59] } else { 32 };
        if !(16..=64).contains(&bit_timing) {
            return Err("WOZ: unsupported bit timing");
        }
        let mut woz = Woz {
            version,
            write_protected: h[22] != 0,
            synchronized: h[23] != 0,
            bit_timing,
            has_flux: false,
            stored_crc: u32_at(h, 8),
            tmap: [BLANK; QUARTERS],
            flux: [BLANK; QUARTERS],
            trks: [Trk::default(); QUARTERS],
        };
        woz.tmap.copy_from_slice(&h[88..88 + QUARTERS]);
        if version == 1 {
            let records = (u32_at(h, 252) as usize / V1_RECORD).min(QUARTERS);
            for (j, trk) in woz.trks.iter_mut().enumerate().take(records) {
                *trk = Trk { start: (j * V1_BLOCKS) as u16, blocks: V1_BLOCKS as u16, bits: u32::from(v1_bits[j]), flux: false };
            }
            for &k in woz.tmap.iter().filter(|&&k| k != BLANK) {
                let k = k as usize;
                let bits = woz.trks.get(k).map_or(0, |t| t.bits as usize);
                // The record and the block after it (it starts 256 bytes in) must be in the file.
                if k >= records || (k + 1) * V1_BLOCKS + 1 > file_blocks || bits == 0 || bits > (V1_BIT_COUNT - 2) * 8 {
                    return Err("WOZ: bad TRK record");
                }
            }
        } else {
            for (i, trk) in woz.trks.iter_mut().enumerate() {
                let r = 256 + i * 8;
                *trk = Trk { start: u16_at(h, r), blocks: u16_at(h, r + 2), bits: u32_at(h, r + 4), flux: false };
            }
            for &k in woz.tmap.iter().filter(|&&k| k != BLANK) {
                let trk = *woz.trks.get(k as usize).ok_or("WOZ: bad TMAP entry")?;
                let (start, blocks) = (trk.start as usize, trk.blocks as usize);
                if start < 3 || start + blocks > file_blocks || trk.bits == 0 || trk.bits as usize > blocks * BLOCK * 8 {
                    return Err("WOZ: bad TRK record");
                }
                if blocks > max_track_blocks || (trk.bits as usize).div_ceil(32) > MAX_TRACK_WORDS {
                    return Err("WOZ: track too long");
                }
            }
            if version >= 2 && h[20] >= 3 && u16_at(h, 66) != 0 && u16_at(h, 68) != 0 {
                // The chunk starts at its block: ID, size, then the map.
                let c = flux_chunk.ok_or("WOZ: FLUX chunk missing")?;
                if &c[..4] != b"FLUX" || u32_at(c, 4) < QUARTERS as u32 {
                    return Err("WOZ: bad FLUX chunk");
                }
                woz.flux.copy_from_slice(&c[8..8 + QUARTERS]);
                for &k in woz.flux.iter().filter(|&&k| k != BLANK) {
                    let trk = woz.trks.get_mut(k as usize).ok_or("WOZ: bad FLUX entry")?;
                    let (start, blocks) = (trk.start as usize, trk.blocks as usize);
                    if start < 3 || start + blocks > file_blocks || trk.bits == 0 || trk.bits as usize > blocks * BLOCK {
                        return Err("WOZ: bad flux TRK record");
                    }
                    trk.flux = true;
                    woz.has_flux = true;
                }
            }
        }
        Ok(woz)
    }

    /// The TRK record heard at quarter track `quarter` (flux if the FLUX map has one), or `0xFF`
    /// for none.
    pub fn track(&self, quarter: u8) -> u8 {
        match self.flux.get(quarter as usize).copied().unwrap_or(BLANK) {
            BLANK => self.tmap.get(quarter as usize).copied().unwrap_or(BLANK),
            k => k,
        }
    }

    /// TRK record `key` (a [`track`](Self::track) value).
    pub fn trk(&self, key: u8) -> Option<Trk> {
        (key != BLANK).then(|| self.trks.get(key as usize).copied()).flatten()
    }

    /// PIO clock (16 cycles a cell) that plays the tracks as a real drive turns the disk, 300 RPM:
    /// their bits, on average, in 200 ms. Each track then plays at its own ([`track_hz`]). INFO's "optimal bit timing" is for emulators' disk
    /// controllers, not the disk: Border Zone's is 28 (3.5 µs), but its tracks hold ~50,400 bits,
    /// 3.97 µs on the disk. Played at 3.5 µs a real Apple II's boot ROM can't keep up.
    pub fn cell_hz(&self) -> u32 {
        let (sum, n) = self.trks.iter().filter(|t| t.bits > 0 && !t.flux).fold((0u64, 0u64), |(s, n), t| (s + u64::from(t.bits), n + 1));
        if n == 0 {
            return 128_000_000 / u32::from(self.bit_timing);
        }
        (sum / n * 5 * 16) as u32
    }

    /// Tracks can be written back: WOZ 2 only (WOZ 1 records aren't block-aligned), not
    /// write-protected, and with no flux tracks (a write can't be spliced into flux timings).
    pub fn writable(&self) -> bool {
        self.version == 2 && !self.write_protected && !self.has_flux
    }
}
