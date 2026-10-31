//! Apple II 5.25" disks, as a Disk II drive holds them: 35 (or 40) tracks of 16 sectors of 256
//! bytes, 6-and-2 GCR at ~4 µs cells, one side.
//!
//! - Sector images (`.dsk`/`.do`/`.po`, and inside `.2mg`) are encoded on the fly like the Mac's
//!   ([`build_track`]): each physical sector 0–15 around the track, in order, with an address
//!   field (`D5 AA 96`, 4-and-4 volume, track, sector, checksum, `DE AA EB`) and a data field
//!   (`D5 AA AD`, 342 6-and-2 nibbles and a checksum nibble, `DE AA EB`). DOS 3.3 and ProDOS
//!   images store the sectors of a track in different orders ([`file_sector`]).
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

/// The order of a track's sectors in a sector image.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Order {
    /// DOS 3.3 order (`.do`, most `.dsk`): logical sectors, which DOS maps to physical ones.
    Dos,
    /// ProDOS order (`.po`): 512-byte blocks, each two physical sectors.
    Prodos,
}

/// Where physical sector `phys` of a track is in a sector image's track: its 256-byte slot.
pub fn file_sector(order: Order, phys: u8) -> usize {
    const DOS: [u8; 16] = [0, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 15];
    const PRODOS: [u8; 16] = [0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15];
    let t = if order == Order::Dos { &DOS } else { &PRODOS };
    t[phys as usize & 0xF] as usize
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

/// The 8 nibbles after `D5 AA 96`: volume, track, sector and their checksum, 4-and-4.
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

// ---- Track layout -------------------------------------------------------------------------

/// Sync nibbles between the address and data fields. DOS 3.3 writes 5 before a data field it
/// rewrites, starting just after the address field.
const HEADER_SYNC: usize = 6;
/// `D5 AA 96` + 8 + `DE AA EB`.
const ADDRESS_NIBBLES: usize = 3 + 8 + 3;
/// `D5 AA AD` + 343 + `DE AA EB`.
const FIELD_NIBBLES: usize = 3 + DATA_NIBBLES + 3;
/// A sector less the sync before its address field.
const FIXED: usize = (ADDRESS_NIBBLES + FIELD_NIBBLES) * 8 + HEADER_SYNC * 10;
/// One revolution of an encoded track: 200 ms (300 RPM) of the Apple II's own write cell
/// (3.911 µs, `drive::A2_SECTOR_PIO_HZ`), a whole number of words: 16 sectors with 232 cells
/// (23 syncs) before each. It plays at the write cell so that what the Apple II writes can be
/// laid into it cell for cell (`writes::splice_a2`).
pub const TRACK_CELLS: usize = 51_136;

/// Cells of sync before sector `k`'s address field on a track of `cells`: what's left after the
/// fixed parts, shared out evenly.
fn gap(cells: usize, k: usize) -> usize {
    let spare = cells - 16 * FIXED;
    spare / 16 + (k < spare % 16) as usize
}

/// `cells` of sync nibbles: `FF` with 2 zeros (10 cells), a few with 1 or 3 to make the length.
/// `None` under 5 nibbles.
fn sync(bits: &mut Bits, cells: usize) -> Option<()> {
    let n = (cells + 5) / 10;
    if n < 5 {
        return None;
    }
    let delta = cells as isize - 10 * n as isize; // -5..=4
    for i in 0..n {
        let len = if (i as isize) < delta.abs() { (10 + delta.signum()) as u32 } else { 10 };
        bits.push(0xFF << (len - 8), len);
    }
    Some(())
}

fn nibbles(bits: &mut Bits, ns: &[u8]) {
    for &n in ns {
        bits.push(n.into(), 8);
    }
}

/// Encode `track` into `out`, exactly `cells` long: physical sectors 0–15 in order, each with
/// its address field (`volume`) and data field `sector_data(phys)`. Each sector is led by its
/// share of the sync, so the loop wraps in a gap. `None` if the sectors don't fit.
pub fn build_track_cells<'a>(
    track: u8,
    volume: u8,
    sector_data: impl Fn(u8) -> &'a [u8; SECTOR_BYTES],
    cells: usize,
    out: &mut TrackBuf,
) -> Option<usize> {
    if cells < 16 * (FIXED + 50) || cells > MAX_TRACK_WORDS * 32 {
        return None;
    }
    let mut bits = Bits::new(out);
    for s in 0..SECTORS {
        sync(&mut bits, gap(cells, s as usize))?;
        nibbles(&mut bits, &[0xD5, 0xAA, 0x96]);
        nibbles(&mut bits, &address_field(volume, track, s));
        nibbles(&mut bits, &[0xDE, 0xAA, 0xEB]);
        for _ in 0..HEADER_SYNC {
            bits.push(0b11_1111_1100, 10);
        }
        nibbles(&mut bits, &[0xD5, 0xAA, 0xAD]);
        encode_data(sector_data(s), |n| bits.push(n.into(), 8));
        nibbles(&mut bits, &[0xDE, 0xAA, 0xEB]);
    }
    Some(bits.finish())
}

/// [`build_track_cells`] at the standard length, [`TRACK_CELLS`].
pub fn build_track<'a>(track: u8, volume: u8, sector_data: impl Fn(u8) -> &'a [u8; SECTOR_BYTES], out: &mut TrackBuf) -> usize {
    const { assert!(TRACK_CELLS % 32 == 0 && TRACK_CELLS / 32 <= MAX_TRACK_WORDS) };
    build_track_cells(track, volume, sector_data, TRACK_CELLS, out).unwrap_or(0)
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
