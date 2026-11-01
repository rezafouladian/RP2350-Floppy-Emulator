//! Apple II 5.25" disks, as a Disk II drive holds them: 35 (or 40) tracks of 16 sectors of 256
//! bytes, 6-and-2 GCR at ~4 µs cells, one side; or of 13 sectors, 5-and-3, as DOS 3.1–3.2.1
//! wrote them ([`Encoding`]).
//!
//! - Sector images (`.dsk`/`.do`/`.po`, and inside `.2mg`) are encoded on the fly like the Mac's
//!   ([`build_track`]): each physical sector 0–15 around the track, in order, with an address
//!   field (`D5 AA 96`, 4-and-4 volume, track, sector, checksum, `DE AA EB`) and a data field
//!   (`D5 AA AD`, 342 6-and-2 nibbles and a checksum nibble, `DE AA EB`). DOS 3.3 and ProDOS
//!   images store the sectors of a track in different orders ([`file_sector`]).
//! - 13-sector images (`.d13`) hold DOS 3.2's sectors by the number in their address field
//!   (`D5 AA B5`); the skew is on the disk ([`Encoding::sector_at`]). Their data fields are
//!   `D5 AA AD`, 410 5-and-3 nibbles and a checksum nibble ([`encode_data53`]), `DE AA EB`.
//! - `.nib` images hold 6,656 nibbles per track ([`build_nib_track`]).
//! - The head is moved by four stepper magnets, PH0–PH3, a quarter track apart in pairs: each
//!   magnet sits on a half track ([`step`]).
//!
//! The marks are the Mac's, and so is the 64-nibble table (`gcr::ENCODE`), but the 6-and-2
//! encoding of a sector is the Apple II's (DOS 3.3 `PRENIBBLE`, ProDOS the same): 86 nibbles of
//! the low two bits of every byte, then 256 of the top six, each XORed with the one before.
//!
//! Hardware-free, so the host tests compile it directly.

use crate::gcr;
use crate::track::{Bits, MAX_TRACK_WORDS, TrackBuf};

pub const SECTORS: u8 = 16;
pub const SECTOR_BYTES: usize = 256;
/// Bytes of one track in a sector image.
pub const TRACK_BYTES: usize = SECTORS as usize * SECTOR_BYTES;
/// Bytes of one track in a `.nib` image.
pub const NIB_TRACK_BYTES: usize = 6_656;
/// Head positions: quarter tracks 0–159 (track 0 to 39.75), as many as a WOZ map has.
pub const QUARTERS: u8 = 160;
/// The volume number DOS 3.3 formats with by default; sector images don't record one.
pub const VOLUME: u8 = 254;
/// Track key for a head position with nothing under it: plays noise.
pub const BLANK: u8 = 0xFF;

/// Sectors a track in DOS 3.2's format, and bytes of one in a `.d13` image.
pub const SECTORS_13: u8 = 13;
pub const TRACK_BYTES_13: usize = SECTORS_13 as usize * SECTOR_BYTES;

/// How a track's sectors are encoded.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Encoding {
    /// DOS 3.3 and ProDOS: 16 sectors, address mark `D5 AA 96`, 6-and-2 data, 10-cell syncs.
    SixAndTwo,
    /// DOS 3.1–3.2.1: 13 sectors, address mark `D5 AA B5`, 5-and-3 data, 9-cell syncs (the
    /// 13-sector P6 PROM's sequencer can't take two 0 cells in a row; a 16-sector one reads them
    /// as well).
    FiveAndThree,
}

impl Encoding {
    pub const fn sectors(self) -> u8 {
        match self {
            Encoding::SixAndTwo => SECTORS,
            Encoding::FiveAndThree => SECTORS_13,
        }
    }

    /// The address mark's third nibble, after `D5 AA`.
    pub const fn address_mark(self) -> u8 {
        match self {
            Encoding::SixAndTwo => 0x96,
            Encoding::FiveAndThree => 0xB5,
        }
    }

    /// Nibbles in a data field after its mark, checksum included.
    pub const fn data_nibbles(self) -> usize {
        match self {
            Encoding::SixAndTwo => DATA_NIBBLES,
            Encoding::FiveAndThree => DATA_NIBBLES_53,
        }
    }

    /// Bytes of one track in a sector image.
    pub const fn track_bytes(self) -> usize {
        self.sectors() as usize * SECTOR_BYTES
    }

    /// Cells of a sync nibble: `FF` and its zeros.
    const fn sync_cells(self) -> usize {
        match self {
            Encoding::SixAndTwo => 10,
            Encoding::FiveAndThree => 9,
        }
    }

    /// The sector number in the `k`th address field around a track. DOS 3.3 and ProDOS skew in
    /// software, so their tracks have sectors 0–15 in order; DOS 3.2's formatter numbers them
    /// 10 apart (modulo 13) around the track, and reads them by those numbers.
    pub const fn sector_at(self, k: u8) -> u8 {
        match self {
            Encoding::SixAndTwo => k,
            Encoding::FiveAndThree => (k as u16 * 10 % SECTORS_13 as u16) as u8,
        }
    }
}

/// The order of a track's sectors in a sector image.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Order {
    /// DOS 3.3 order (`.do`, most `.dsk`): logical sectors, which DOS maps to physical ones.
    Dos,
    /// ProDOS order (`.po`): 512-byte blocks, each two physical sectors.
    Prodos,
    /// DOS 3.2 (`.d13`): 13 sectors a track, 5-and-3, each in the slot its address field's
    /// number says (the skew is in the numbering around the track).
    Dos32,
}

impl Order {
    pub const fn encoding(self) -> Encoding {
        match self {
            Order::Dos | Order::Prodos => Encoding::SixAndTwo,
            Order::Dos32 => Encoding::FiveAndThree,
        }
    }
}

/// Where sector `phys` of a track (the number in its address field) is in a sector image's
/// track: its 256-byte slot.
pub fn file_sector(order: Order, phys: u8) -> usize {
    const DOS: [u8; 16] = [0, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 15];
    const PRODOS: [u8; 16] = [0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15];
    match order {
        Order::Dos => DOS[phys as usize & 0xF] as usize,
        Order::Prodos => PRODOS[phys as usize & 0xF] as usize,
        Order::Dos32 => phys as usize,
    }
}

/// The track a sector or `.nib` image plays at quarter track `quarter`, or [`BLANK`]. A track is
/// heard from a quarter track either side of it, as WOZ maps of standard disks have it; half
/// tracks between them are blank.
pub fn quarter_track(quarter: u8, tracks: u8) -> u8 {
    let t = (quarter as u32 + 1) / 4;
    if quarter % 4 == 2 || t >= tracks as u32 { BLANK } else { t as u8 }
}

// ---- Stepper ----------------------------------------------------------------------------------

/// Where each combination of magnets (bit `i` = PHi on) pulls the head, as a quarter track
/// modulo 8: PHi sits at `2i`, two neighbours pull to the quarter track between them, three to
/// the middle one. `F`: nowhere (none, all four, or two opposite ones).
const STEP_TARGET: u64 = {
    let t: [u64; 16] = [0xF, 0, 2, 1, 4, 0xF, 3, 2, 6, 7, 0xF, 0, 5, 6, 4, 0xF];
    let mut v = 0u64;
    let mut m = 0;
    while m < 16 {
        v |= t[m] << (4 * m);
        m += 1;
    }
    v
};

/// The head's quarter track after the magnets change to `phases` (bit `i` = PHi), from
/// `quarter`. It moves at once to the nearest place the magnets pull it to, up to 3 quarter tracks
/// either way; a magnet 4 away (opposite) can't pull it. Track 0 and 39.75 are the stops.
///
/// Packed in a constant rather than a table so core 1 can run it from RAM.
#[inline(always)]
pub const fn step(quarter: u8, phases: u32) -> u8 {
    let target = (STEP_TARGET >> ((phases & 0xF) * 4)) & 0xF;
    if target > 7 {
        return quarter;
    }
    let d = (target as u8).wrapping_sub(quarter) & 7;
    if d == 0 || d == 4 {
        quarter
    } else if d < 4 {
        let q = quarter + d;
        if q >= QUARTERS { QUARTERS - 1 } else { q }
    } else {
        quarter.saturating_sub(8 - d)
    }
}

// ---- Sector codec -------------------------------------------------------------------------

/// 4-and-4: the odd bits, then the even bits, each with every other bit set.
pub const fn encode44(b: u8) -> [u8; 2] {
    [b >> 1 | 0xAA, b | 0xAA]
}

pub const fn decode44(a: u8, b: u8) -> u8 {
    (a << 1 | 1) & b
}

/// The 8 nibbles after `D5 AA 96` (or DOS 3.2's `D5 AA B5`): volume, track, sector and their checksum, 4-and-4.
pub fn address_field(volume: u8, track: u8, sector: u8) -> [u8; 8] {
    let mut out = [0u8; 8];
    for (k, b) in [volume, track, sector, volume ^ track ^ sector].into_iter().enumerate() {
        out[2 * k..2 * k + 2].copy_from_slice(&encode44(b));
    }
    out
}

/// Nibbles in a data field after its mark: 342 data, 1 checksum.
pub const DATA_NIBBLES: usize = 343;

/// 6-and-2 encode a sector into [`DATA_NIBBLES`] nibbles.
pub fn encode_data(data: &[u8; SECTOR_BYTES], mut emit: impl FnMut(u8)) {
    // The low two bits of bytes i, i + 86, i + 172 (each pair swapped) go in value i.
    let swap = |b: u8| (b & 1) << 1 | (b >> 1 & 1);
    let mut prev = 0u8;
    for i in 0..342 {
        let v = if i < 86 {
            (0..3).filter(|k| i + 86 * k < 256).fold(0, |v, k| v | swap(data[i + 86 * k]) << (2 * k))
        } else {
            data[i - 86] >> 2
        };
        emit(gcr::ENCODE[(v ^ prev) as usize]);
        prev = v;
    }
    emit(gcr::ENCODE[prev as usize]);
}

/// Reverse [`encode_data`]. `None` if a nibble isn't a data nibble or the checksum is wrong.
pub fn decode_data(nibbles: &[u8; DATA_NIBBLES]) -> Option<[u8; SECTOR_BYTES]> {
    let mut v = [0u8; 342];
    let mut prev = 0u8;
    for (d, &n) in v.iter_mut().zip(nibbles) {
        prev ^= gcr::decode6(n)?;
        *d = prev;
    }
    if gcr::decode6(nibbles[342])? != prev {
        return None;
    }
    let mut out = [0u8; SECTOR_BYTES];
    for (i, b) in out.iter_mut().enumerate() {
        let low = v[i % 86] >> (2 * (i / 86)) & 3;
        *b = v[86 + i] << 2 | (low & 1) << 1 | low >> 1;
    }
    Some(out)
}

// ---- 5-and-3 (DOS 3.2) ----------------------------------------------------------------------

/// The 32 nibbles of 5-and-3: every byte with its top bit set and no two 0 bits in a row, but for
/// `D5` and `AA` (the marks).
pub const ENCODE53: [u8; 32] = [
    0xAB, 0xAD, 0xAE, 0xAF, 0xB5, 0xB6, 0xB7, 0xBA, 0xBB, 0xBD, 0xBE, 0xBF, 0xD6, 0xD7, 0xDA, 0xDB,
    0xDD, 0xDE, 0xDF, 0xEA, 0xEB, 0xED, 0xEE, 0xEF, 0xF5, 0xF6, 0xF7, 0xFA, 0xFB, 0xFD, 0xFE, 0xFF,
];

const DECODE53: [u8; 256] = {
    let mut t = [0xFF; 256];
    let mut i = 0;
    while i < 32 {
        t[ENCODE53[i] as usize] = i as u8;
        i += 1;
    }
    t
};

/// 5-bit value of a 5-and-3 nibble.
pub fn decode5(nibble: u8) -> Option<u8> {
    match DECODE53[nibble as usize] {
        0xFF => None,
        v => Some(v),
    }
}

/// Nibbles in a 5-and-3 data field after its mark: 410 data, 1 checksum.
pub const DATA_NIBBLES_53: usize = 411;
/// Groups of five bytes in a sector (the 256th is on its own).
const GROUPS_53: usize = 51;
/// 5-and-3 values before the 256 top fives: three per group, and byte 255's low 3 bits.
const THREES: usize = 3 * GROUPS_53 + 1;

/// Where the low 3 bits of byte `j` (0–2) of group `c` go in a sector's 410 5-and-3 values
///
/// Bytes come in groups of five, `g` = 0–50, numbered backwards (`c` = 50 − g). Byte
/// `5g + j`'s top 5 bits are value `154 + 51j + c`. Bytes 0–2 keep their low 3 bits as bits 4–2
/// of value `153 − 51j − c`, a "three", whose bits 1 and 0 are bit `2 − j` of bytes 3 and 4.
/// Byte 255's top is value 409, its low 3 bits value 0.
#[inline]
const fn three_at(j: usize, c: usize) -> usize {
    THREES - 1 - (GROUPS_53 * j + c)
}

/// 5-and-3 encode a sector into [`DATA_NIBBLES_53`] nibbles: 410 values, each XORed with the one
/// before (from 0), then the last as the checksum.
pub fn encode_data53(data: &[u8; SECTOR_BYTES], mut emit: impl FnMut(u8)) {
    let mut v = [0u8; THREES + SECTOR_BYTES];
    for g in 0..GROUPS_53 {
        let c = GROUPS_53 - 1 - g;
        let b = &data[5 * g..5 * g + 5];
        for j in 0..5 {
            v[THREES + GROUPS_53 * j + c] = b[j] >> 3;
        }
        for j in 0..3 {
            let shift = 2 - j;
            v[three_at(j, c)] = (b[j] & 7) << 2 | (b[3] >> shift & 1) << 1 | (b[4] >> shift & 1);
        }
    }
    v[0] = data[255] & 7;
    v[THREES + 255] = data[255] >> 3;
    let mut prev = 0u8;
    for &x in &v {
        emit(ENCODE53[(x ^ prev) as usize]);
        prev = x;
    }
    emit(ENCODE53[prev as usize]);
}

/// Reverse [`encode_data53`]. `None` if a nibble isn't a 5-and-3 nibble or the checksum is wrong.
pub fn decode_data53(nibbles: &[u8; DATA_NIBBLES_53]) -> Option<[u8; SECTOR_BYTES]> {
    let mut v = [0u8; THREES + SECTOR_BYTES];
    let mut prev = 0u8;
    for (d, &n) in v.iter_mut().zip(nibbles) {
        prev ^= decode5(n)?;
        *d = prev;
    }
    if decode5(nibbles[DATA_NIBBLES_53 - 1])? != prev {
        return None;
    }
    let mut out = [0u8; SECTOR_BYTES];
    for g in 0..GROUPS_53 {
        let c = GROUPS_53 - 1 - g;
        let t = [v[three_at(0, c)], v[three_at(1, c)], v[three_at(2, c)]];
        for j in 0..5 {
            let low = match j {
                0..=2 => t[j] >> 2,
                // Bit 1 (byte 3) or bit 0 (byte 4) of each three, three 0's the top.
                _ => {
                    let bit = 4 - j as u8;
                    (t[0] >> bit & 1) << 2 | (t[1] >> bit & 1) << 1 | (t[2] >> bit & 1)
                }
            };
            out[5 * g + j] = v[THREES + GROUPS_53 * j + c] << 3 | low;
        }
    }
    out[255] = v[THREES + 255] << 3 | v[0] & 7;
    Some(out)
}

// ---- Track layout -------------------------------------------------------------------------

/// Sync nibbles between the address and data fields. DOS 3.3 writes 5 before a data field it
/// rewrites, starting just after the address field.
const HEADER_SYNC: usize = 6;
/// `D5 AA 96` + 8 + `DE AA EB`.
const ADDRESS_NIBBLES: usize = 3 + 8 + 3;

/// A sector less the sync before its address field: address field, sync, `D5 AA AD`, data,
/// `DE AA EB`.
const fn fixed(enc: Encoding) -> usize {
    (ADDRESS_NIBBLES + 3 + enc.data_nibbles() + 3) * 8 + HEADER_SYNC * enc.sync_cells()
}

/// One revolution of an encoded track: 200 ms (300 RPM) of the Apple II's own write cell
/// (3.911 µs, `drive::A2_SECTOR_PIO_HZ`), a whole number of words: 16 sectors with 232 cells
/// (23 syncs) before each, or 13 with ~430 (48 9-cell syncs). It plays at the write cell so that
/// what the Apple II writes can be laid into it cell for cell (`writes::splice_a2`).
pub const TRACK_CELLS: usize = 51_136;

/// Cells of sync before the `k`th sector's address field on a track of `cells`: what's left
/// after the fixed parts, shared out evenly.
fn gap(enc: Encoding, cells: usize, k: usize) -> usize {
    let n = enc.sectors() as usize;
    let spare = cells - n * fixed(enc);
    spare / n + (k < spare % n) as usize
}

/// `cells` of sync nibbles, `None` under 5 nibbles. 6-and-2: `FF` with 2 zeros (10 cells), a few
/// with 1 or 3 to make the length. 5-and-3: `FF` with 1 zero, a few with none (never two zeros).
fn sync(bits: &mut Bits, enc: Encoding, cells: usize) -> Option<()> {
    match enc {
        Encoding::SixAndTwo => {
            let n = (cells + 5) / 10;
            if n < 5 {
                return None;
            }
            let delta = cells as isize - 10 * n as isize; // -5..=4
            for i in 0..n {
                let len = if (i as isize) < delta.abs() { (10 + delta.signum()) as u32 } else { 10 };
                bits.push(0xFF << (len - 8), len);
            }
        }
        Encoding::FiveAndThree => {
            let n = cells.div_ceil(9);
            let short = 9 * n - cells; // 0..=8 of 8 cells
            if n < 5 || short > n {
                return None;
            }
            for i in 0..n {
                if i < short { bits.push(0xFF, 8) } else { bits.push(0x1FE, 9) }
            }
        }
    }
    Some(())
}

fn nibbles(bits: &mut Bits, ns: &[u8]) {
    for &n in ns {
        bits.push(n.into(), 8);
    }
}

/// Encode `track` into `out`, exactly `cells` long, in `enc`: its sectors around the track in
/// [`Encoding::sector_at`]'s order (16-sector: 0–15 in order), each with its address field
/// (`volume`) and data field `sector_data(s)` for the one numbered `s`. Each sector is led by its
/// share of the sync, so the loop wraps in a gap. `None` if the sectors don't fit.
pub fn build_track_cells<'a>(
    enc: Encoding,
    track: u8,
    volume: u8,
    sector_data: impl Fn(u8) -> &'a [u8; SECTOR_BYTES],
    cells: usize,
    out: &mut TrackBuf,
) -> Option<usize> {
    let n = enc.sectors();
    if cells < n as usize * (fixed(enc) + 50) || cells > MAX_TRACK_WORDS * 32 {
        return None;
    }
    let mut bits = Bits::new(out);
    for k in 0..n {
        let s = enc.sector_at(k);
        sync(&mut bits, enc, gap(enc, cells, k as usize))?;
        nibbles(&mut bits, &[0xD5, 0xAA, enc.address_mark()]);
        nibbles(&mut bits, &address_field(volume, track, s));
        nibbles(&mut bits, &[0xDE, 0xAA, 0xEB]);
        for _ in 0..HEADER_SYNC {
            let c = enc.sync_cells() as u32;
            bits.push(0xFF << (c - 8), c);
        }
        nibbles(&mut bits, &[0xD5, 0xAA, 0xAD]);
        match enc {
            Encoding::SixAndTwo => encode_data(sector_data(s), |n| bits.push(n.into(), 8)),
            Encoding::FiveAndThree => encode_data53(sector_data(s), |n| bits.push(n.into(), 8)),
        }
        nibbles(&mut bits, &[0xDE, 0xAA, 0xEB]);
    }
    Some(bits.finish())
}

/// [`build_track_cells`] at the standard length, [`TRACK_CELLS`].
pub fn build_track<'a>(enc: Encoding, track: u8, volume: u8, sector_data: impl Fn(u8) -> &'a [u8; SECTOR_BYTES], out: &mut TrackBuf) -> usize {
    const { assert!(TRACK_CELLS % 32 == 0 && TRACK_CELLS / 32 <= MAX_TRACK_WORDS) };
    build_track_cells(enc, track, volume, sector_data, TRACK_CELLS, out).unwrap_or(0)
}

// ---- NIB ----------------------------------------------------------------------------------

/// A run of at least this many `FF`s in a `.nib` track is sync.
const NIB_SYNC_RUN: usize = 5;

/// Expand a `.nib` track (`byte(i)` for i < [`NIB_TRACK_BYTES`]) into cells in `out`. A `.nib`
/// keeps nibbles but not the zeros after sync `FF`s, so the `FF`s of each run of
/// [`NIB_SYNC_RUN`] or more get their 2 zeros back (some go without, to make whole words: the
/// track is 1,664 words of nibbles, and every 16 zero pairs is another word). Without them an
/// Apple II that starts reading mid-nibble couldn't fall into step. Returns the words.
pub fn build_nib_track(byte: impl Fn(usize) -> u8, out: &mut TrackBuf) -> usize {
    const N: usize = NIB_TRACK_BYTES;
    const { assert!(N * 8 % 32 == 0) };
    // Which bytes are sync, and how many: runs are found around the loop.
    let mut sync = [0u8; N / 8];
    let mut count = 0;
    let start = (0..N).find(|&i| byte(i) != 0xFF);
    if let Some(start) = start {
        let mut i = 0;
        while i < N {
            let at = (start + i) % N;
            if byte(at) != 0xFF {
                i += 1;
                continue;
            }
            let run = (i..N).take_while(|&k| byte((start + k) % N) == 0xFF).count();
            if run >= NIB_SYNC_RUN {
                for k in i..i + run {
                    let b = (start + k) % N;
                    sync[b / 8] |= 1 << (b % 8);
                }
                count += run;
            }
            i += run;
        }
    }
    let mut expand = count - count % 16;
    let mut bits = Bits::new(out);
    for i in 0..N {
        bits.push(byte(i).into(), 8);
        if expand > 0 && sync[i / 8] >> (i % 8) & 1 != 0 {
            bits.push(0, 2);
            expand -= 1;
        }
    }
    bits.finish()
}
