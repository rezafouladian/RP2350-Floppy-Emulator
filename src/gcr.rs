//! Apple 3.5" GCR track encoder for 400K (single-sided) and 800K (double-sided) disks.
//!
//! Builds the cell stream for one track side: one bit per 2.0425 µs cell (see `track.rs`).
//! The address fields' format byte ([`format`]) says single- or double-sided and the interleave:
//! 2:1 as the Mac formats (`$22`), 4:1 as an Apple IIgs does (`$24`), which leaves its slower
//! CPU time to take each block before the next one comes round.

use crate::track::{BLOCK, Bits, MAX_TRACK_WORDS, TrackBuf};

const TAG: usize = 12;

/// 10-bit self-sync nibbles (`11111111 00`) before each address field. 40 of them make each
/// sector slot ~6220 cells, close to the real ~6224 at every zone's speed.
const GAP_SYNC: usize = 40;
/// Sync nibbles between the address and data fields (the ROM allows up to 48 nibbles).
const HEADER_SYNC: usize = 6;

const SYNC_BITS: u32 = 10;
/// `D5 AA 96` + 5 header nibbles + `DE AA`.
const ADDRESS_NIBBLES: usize = 10;
/// `D5 AA AD` + sector + 699 data + 4 checksum + `DE AA FF`.
const DATA_NIBBLES: usize = 3 + 1 + 699 + 4 + 3;
const SECTOR_BITS: usize =
    (GAP_SYNC + HEADER_SYNC) * SYNC_BITS as usize + (ADDRESS_NIBBLES + DATA_NIBBLES) * 8;


/// The ROM's `Nibl` table: 6-bit value -> disk nibble.
pub const ENCODE: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

/// Disk nibble -> 6-bit value, or `0xFF` for a nibble that isn't a data nibble.
const DECODE: [u8; 256] = {
    let mut t = [0xFF; 256];
    let mut i = 0;
    while i < 64 {
        t[ENCODE[i] as usize] = i as u8;
        i += 1;
    }
    t
};

/// 6-bit value of a data nibble.
pub fn decode6(nibble: u8) -> Option<u8> {
    match DECODE[nibble as usize] {
        0xFF => None,
        v => Some(v),
    }
}

/// Speed zone (0..=4) of a track.
pub const fn zone(track: u8) -> u8 {
    track / 16
}

/// Sectors on each side of a track: 12, 11, 10, 9 or 8.
pub const fn sectors_per_track(track: u8) -> u8 {
    12 - zone(track)
}

/// Spindle speed for the track's zone; also the TACH frequency in Hz.
pub const fn rpm(track: u8) -> u32 {
    [394, 429, 472, 525, 590][zone(track) as usize]
}

/// First logical block of a cylinder. Blocks run through side 0 of a track, then side 1,
/// then the next track.
pub const fn first_block(track: u8, sides: u8) -> usize {
    let mut block = 0;
    let mut t = 0;
    while t < track {
        block += sectors_per_track(t) as usize;
        t += 1;
    }
    block * sides as usize
}

/// The format byte for a disk of `sides` (1 or 2): the Mac's `$02`/`$22` (2:1 interleave), or
/// for an Apple II `$04`/`$24` (4:1), as Apple IIgs disks have.
pub const fn format(sides: u8, apple2: bool) -> u8 {
    (if sides == 2 { 0x20 } else { 0 }) | if apple2 { 4 } else { 2 }
}

/// Physical order of sectors around the track for interleave `factor`: sector `s` goes in slot
/// `s * factor` (mod the sector count), or the next free slot after it. 2:1 gives the Mac's
/// 0 6 1 7 2 8 ..., 4:1 the IIgs's 0 3 6 9 1 4 ... (as on its disks, in every zone).
fn interleave(sectors: u8, factor: u8) -> [u8; 12] {
    let mut order = [0xFF; 12];
    for s in 0..sectors {
        let mut slot = (s as usize * factor.max(1) as usize) % sectors as usize;
        while order[slot] != 0xFF {
            slot = (slot + 1) % sectors as usize;
        }
        order[slot] = s;
    }
    order
}

fn nibble(bits: &mut Bits, n: u8) {
    bits.push(n.into(), 8);
}

fn sync(bits: &mut Bits, count: usize) {
    for _ in 0..count {
        bits.push(0b11_1111_1100, SYNC_BITS);
    }
}

/// The 5 address-field nibbles after `D5 AA 96`.
pub fn address_field(track: u8, sector: u8, side: u8, format: u8) -> [u8; 5] {
    let t = track & 0x3F;
    let h = (side & 1) << 5 | (track >> 6);
    let f = format & 0x3F;
    [ENCODE[t as usize], ENCODE[sector as usize], ENCODE[h as usize], ENCODE[f as usize], ENCODE[(t ^ sector ^ h ^ f) as usize]]
}

/// 6-and-2 encode 12 tag bytes + 512 data bytes into 699 data nibbles + 4 checksum nibbles.
/// Order of operations follows the ROM in the Mac's .Sony driver.
pub fn encode_sector(tags: &[u8; TAG], data: &[u8], mut emit: impl FnMut(u8)) {
    let byte = |i: usize| if i < TAG { tags[i] } else { data[i - TAG] };
    let (mut c1, mut c2, mut c3) = (0u32, 0u32, 0u32);
    let mut i = 0;
    while i < TAG + BLOCK {
        let last = i + 2 == TAG + BLOCK;
        let x = c3 >> 7;
        c3 = (c3 << 1 | x) & 0xFF;
        c1 += byte(i) as u32 + x;
        let x = c1 >> 8;
        c1 &= 0xFF;
        let a = byte(i) ^ c3 as u8;
        c2 += byte(i + 1) as u32 + x;
        let x = c2 >> 8;
        c2 &= 0xFF;
        let b = byte(i + 1) ^ c1 as u8;
        let mut c = 0;
        if !last {
            c3 += byte(i + 2) as u32 + x;
            c3 &= 0xFF;
            c = byte(i + 2) ^ c2 as u8;
        }
        emit(ENCODE[((a >> 2 & 0x30) | (b >> 4 & 0x0C) | c >> 6) as usize]);
        emit(ENCODE[(a & 0x3F) as usize]);
        emit(ENCODE[(b & 0x3F) as usize]);
        if !last {
            emit(ENCODE[(c & 0x3F) as usize]);
        }
        i += 3;
    }
    let (c1, c2, c3) = (c1 as u8, c2 as u8, c3 as u8);
    emit(ENCODE[((c1 >> 2 & 0x30) | (c2 >> 4 & 0x0C) | c3 >> 6) as usize]);
    emit(ENCODE[(c1 & 0x3F) as usize]);
    emit(ENCODE[(c2 & 0x3F) as usize]);
    emit(ENCODE[(c3 & 0x3F) as usize]);
}

/// Reverse `encode_sector`: 699 data nibbles + 4 checksum nibbles -> 12 tag bytes + 512 data
/// bytes. `None` if a nibble isn't a data nibble or the checksum doesn't match.
pub fn decode_sector(nibbles: &[u8; 703]) -> Option<[u8; TAG + BLOCK]> {
    let mut out = [0u8; TAG + BLOCK];
    let mut v = [0u8; 703];
    for (d, &n) in v.iter_mut().zip(nibbles) {
        *d = decode6(n)?;
    }
    let (mut c1, mut c2, mut c3) = (0u32, 0u32, 0u32);
    let (mut i, mut o) = (0, 0);
    while o < TAG + BLOCK {
        let last = o + 2 == TAG + BLOCK;
        let hi = v[i];
        let a = v[i + 1] | (hi << 2 & 0xC0);
        let b = v[i + 2] | (hi << 4 & 0xC0);
        i += 3;
        let x = c3 >> 7;
        c3 = (c3 << 1 | x) & 0xFF;
        let a = a ^ c3 as u8;
        c1 += u32::from(a) + x;
        let x = c1 >> 8;
        c1 &= 0xFF;
        out[o] = a;
        let b = b ^ c1 as u8;
        c2 += u32::from(b) + x;
        let x = c2 >> 8;
        c2 &= 0xFF;
        out[o + 1] = b;
        o += 2;
        if !last {
            let c = (v[i] | (hi << 6 & 0xC0)) ^ c2 as u8;
            i += 1;
            c3 += u32::from(c) + x;
            c3 &= 0xFF;
            out[o] = c;
            o += 1;
        }
    }
    let hi = v[i];
    let k = [v[i + 1] | (hi << 2 & 0xC0), v[i + 2] | (hi << 4 & 0xC0), v[i + 3] | (hi << 6 & 0xC0)];
    (k == [c1 as u8, c2 as u8, c3 as u8]).then_some(out)
}

/// Encode one side of `track` into `out`. `sector_data(s)` returns the 512 data bytes of
/// sector `s` on that side; `format` ([`format`]) goes in the address fields and sets the
/// interleave. Returns the number of words to stream (the track repeats from word 0 after that).
pub fn build_track<'a>(
    track: u8,
    side: u8,
    format: u8,
    sector_data: impl Fn(u8) -> &'a [u8; BLOCK],
    out: &mut TrackBuf,
) -> usize {
    // The longest track (12 sectors) must fit a track buffer.
    const { assert!((12 * SECTOR_BITS).div_ceil(32) <= MAX_TRACK_WORDS) };
    let mut bits = Bits::new(out);
    encode_track(&mut bits, track, side, format, sector_data, GAP_SYNC);
    bits.finish()
}

/// Like [`build_track`], for a track that must be exactly `cells` long (a MOOF track being
/// reformatted, whose length is fixed in the file). The gap before each address field shrinks
/// to fit, down to 5 syncs, and the rest of the revolution is sync. `None` if it can't fit.
pub fn build_track_cells<'a>(
    track: u8,
    side: u8,
    format: u8,
    sector_data: impl Fn(u8) -> &'a [u8; BLOCK],
    cells: usize,
    out: &mut TrackBuf,
) -> Option<()> {
    let fixed = SECTOR_BITS - GAP_SYNC * SYNC_BITS as usize;
    let per_sector = cells / sectors_per_track(track) as usize;
    let gap = (per_sector.checked_sub(fixed)? / SYNC_BITS as usize).min(GAP_SYNC);
    if gap < 5 || cells > MAX_TRACK_WORDS * 32 {
        return None;
    }
    let mut bits = Bits::new(out);
    encode_track(&mut bits, track, side, format, sector_data, gap);
    while bits.len() + SYNC_BITS as usize <= cells {
        sync(&mut bits, 1);
    }
    bits.finish(); // the last few cells are 1s, the start of one more sync
    Some(())
}

fn encode_track<'a>(
    bits: &mut Bits,
    track: u8,
    side: u8,
    format: u8,
    sector_data: impl Fn(u8) -> &'a [u8; BLOCK],
    gap: usize,
) {
    let sectors = sectors_per_track(track);
    let order = interleave(sectors, format & 0x0F);
    let tags = [0u8; TAG];
    for &sector in &order[..sectors as usize] {
        sync(bits, gap);
        for n in [0xD5, 0xAA, 0x96] {
            nibble(bits, n);
        }
        for n in address_field(track, sector, side, format) {
            nibble(bits, n);
        }
        nibble(bits, 0xDE);
        nibble(bits, 0xAA);
        sync(bits, HEADER_SYNC);
        for n in [0xD5, 0xAA, 0xAD, ENCODE[sector as usize]] {
            nibble(bits, n);
        }
        encode_sector(&tags, sector_data(sector), |n| nibble(bits, n));
        for n in [0xDE, 0xAA, 0xFF] {
            nibble(bits, n);
        }
    }
}
