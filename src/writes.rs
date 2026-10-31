//! Decoding what the Mac writes.
//!
//! `capture.pio` turns WR into cells (one bit per cell, 1 = flux reversal), the same form as a
//! track buffer. From those:
//! - raw images take the decoded sectors: [`gcr_data_fields`], [`mfm_fields`];
//! - MOOF images take the cells themselves, spliced into the stored track at the data field the
//!   Mac rewrote ([`splice_gcr`], [`splice_mfm`]). A format can't be copied like that (the
//!   Mac's layout needn't fit the stored track's length), so its sectors are decoded and the
//!   track re-encoded to fit (`gcr::build_track_cells`, `mfm::build_track_cells`).
//! - Apple II 5.25" sector images are held as cells while a track is loaded, and every write is
//!   laid into them where it was made ([`splice_a2`]); the sectors are then decoded from the track
//!   ([`a2_fields`]). DOS 3.3 formats a track a field at a time and reads it back expecting its
//!   own layout, so it can't be re-encoded.

use crate::{a2, gcr};
use crate::mfm;
use crate::track::TrackBuf;

/// Random-access cells (1 = flux reversal).
pub trait Cells {
    fn bit(&self, i: usize) -> bool;
}

/// A stored track: cells that loop after `len()`, and can be changed.
pub trait CellTrack: Cells {
    fn len(&self) -> usize;
    fn set(&mut self, i: usize, v: bool);
}

/// IWM framing: a nibble is complete when its top bit is set. Calls `f(nibble, first_cell)`
/// for each, where `first_cell` is the index of the nibble's leading 1.
fn frame(bits: impl Fn(usize) -> bool, range: core::ops::Range<usize>, mut f: impl FnMut(u8, usize) -> bool) {
    let mut shifter = 0u8;
    let mut first = 0;
    for i in range {
        let b = bits(i);
        if shifter == 0 {
            if !b {
                continue; // leading zeros are skipped
            }
            first = i;
        }
        shifter = shifter << 1 | b as u8;
        if shifter & 0x80 != 0 {
            if !f(shifter, first) {
                return;
            }
            shifter = 0;
        }
    }
}

const DATA_MARK: u32 = 0xD5_AAAD;
const ADDRESS_MARK: u32 = 0xD5_AA96;

/// Every intact GCR data field in `len` cells: `f(sector, tags + data)`. The sector number comes
/// from the data field itself.
pub fn gcr_data_fields(cells: &impl Cells, len: usize, mut f: impl FnMut(u8, &[u8; 524])) {
    let mut recent = 0u32;
    let mut sector: Option<u8> = None;
    let mut field = [0u8; 703];
    let mut fill: Option<usize> = None;
    frame(|i| cells.bit(i), 0..len, |n, _| {
        if let Some(k) = fill {
            field[k] = n;
            if k + 1 == field.len() {
                if let (Some(s), Some(data)) = (sector, gcr::decode_sector(&field)) {
                    f(s, &data);
                }
                fill = None;
            } else {
                fill = Some(k + 1);
            }
        } else if recent & 0xFF_FFFF == DATA_MARK {
            sector = gcr::decode6(n);
            fill = sector.map(|_| 0);
        }
        recent = recent << 8 | u32::from(n);
        true
    });
}

/// Every intact Apple II 5.25" data field in `len` cells: `f(address, data)`, where `address` is
/// the (track, sector) of an intact address field just before it, if there is one (a track or a
/// format has them; a sector write starts after the address field).
pub fn a2_fields(cells: &impl Cells, len: usize, mut f: impl FnMut(Option<(u8, u8)>, &[u8; a2::SECTOR_BYTES])) {
    enum Fill {
        Idle,
        Address(usize),
        Data(usize),
    }
    let mut recent = 0u32;
    let mut fill = Fill::Idle;
    let mut address = [0u8; 8];
    let mut field = [0u8; a2::DATA_NIBBLES];
    let mut header: Option<(u8, u8)> = None;
    frame(|i| cells.bit(i), 0..len, |n, _| {
        match fill {
            Fill::Address(k) => {
                address[k] = n;
                fill = if k + 1 < address.len() { Fill::Address(k + 1) } else { Fill::Idle };
                if k + 1 == address.len() {
                    let d = |j: usize| a2::decode44(address[2 * j], address[2 * j + 1]);
                    header = (d(0) ^ d(1) ^ d(2) == d(3)).then(|| (d(1), d(2)));
                }
            }
            Fill::Data(k) => {
                field[k] = n;
                fill = if k + 1 < field.len() { Fill::Data(k + 1) } else { Fill::Idle };
                if k + 1 == field.len()
                    && let Some(data) = a2::decode_data(&field)
                {
                    f(header.take(), &data);
                }
            }
            Fill::Idle if recent & 0xFF_FFFF == ADDRESS_MARK => {
                header = None;
                address[0] = n;
                fill = Fill::Address(1);
            }
            Fill::Idle if recent & 0xFF_FFFF == DATA_MARK => {
                field[0] = n;
                fill = Fill::Data(1);
            }
            Fill::Idle => {}
        }
        recent = recent << 8 | u32::from(n);
        true
    });
}

/// Raw cells of three `$A1` marks (clock bit dropped).
const MFM_SYNC: u64 = 0x4489_4489_4489;

/// An MFM field, CRC-checked.
pub enum MfmField<'a> {
    /// ID field: cylinder, head, sector (1-based), size code.
    Id([u8; 4]),
    Data(&'a [u8; 512]),
}

/// Data bits of the byte whose 16 cells start at `at`.
fn mfm_byte(cells: &impl Fn(usize) -> bool, at: usize) -> u8 {
    (0..8).fold(0, |v, k| v << 1 | cells(at + 2 * k + 1) as u8)
}

/// Every intact MFM ID and data field in `len` cells, in order: `f(field, first_cell)` where
/// `first_cell` is the start of the field's three `$A1` marks.
pub fn mfm_fields(cells: &impl Cells, len: usize, mut f: impl FnMut(MfmField, usize)) {
    let bit = |i: usize| cells.bit(i);
    let mut window = 0u64;
    let mut i = 0;
    let mut data = [0u8; 514];
    while i < len {
        window = (window << 1 | bit(i) as u64) & 0xFFFF_FFFF_FFFF;
        i += 1;
        if window != MFM_SYNC {
            continue;
        }
        let marks = i - 48;
        let mark = if i + 16 <= len { mfm_byte(&bit, i) } else { break };
        let body = i + 16;
        let n = match mark {
            0xFE => 6,
            0xFB => 514,
            _ => continue,
        };
        if body + n * 16 > len {
            break;
        }
        for (k, d) in data[..n].iter_mut().enumerate() {
            *d = mfm_byte(&bit, body + k * 16);
        }
        if mfm::crc16(mfm::crc16(0xFFFF, &[0xA1, 0xA1, 0xA1, mark]), &data[..n]) != 0 {
            continue;
        }
        if mark == 0xFE {
            f(MfmField::Id([data[0], data[1], data[2], data[3]]), marks);
        } else {
            f(MfmField::Data(data[..512].try_into().unwrap()), marks);
        }
        i = body + n * 16;
        window = 0;
    }
}

/// One past the last 1 in the capture: where the Mac stopped writing. The capture runs a little
/// past the gate (all 0s), and those cells must not overwrite the track.
fn written_len(cells: &impl Cells, len: usize) -> usize {
    (0..len).rev().find(|&i| cells.bit(i)).map_or(0, |i| i + 1)
}

/// Copy capture cells `from..end` over the track, cell `from` at `to` (wrapping: a write longer
/// than the track goes on over its own start, as on a disk). Returns where it ended.
fn overwrite(track: &mut impl CellTrack, to: usize, cells: &impl Cells, from: usize, end: usize) -> usize {
    let n = track.len();
    for k in (end - from).saturating_sub(n)..end - from {
        track.set((to + k) % n, cells.bit(from + k));
    }
    (to + end - from) % n
}

/// How far before the gate position to start looking for the header the Mac just read, and how
/// far past it a header may still end. The position is accurate to a word or two.
const LOOK_BACK: usize = 3_000;
const LOOK_AHEAD: usize = 256;

/// Splice a GCR data-field write into a stored track. `pos` is the track cell under the head
/// when the write gate opened. The capture's data mark (`D5 AA AD`) is laid exactly over the
/// data mark that follows the last address field before `pos`, so the sector keeps its place.
/// Returns where the capture's first cell falls and where the write ended.
pub fn splice_gcr(track: &mut impl CellTrack, pos: usize, cells: &impl Cells, len: usize) -> Result<(usize, usize), &'static str> {
    let mut from = None;
    let mut recent = 0u32;
    let mut starts = [0usize; 3];
    frame(|i| cells.bit(i), 0..len, |nib, first| {
        recent = recent << 8 | u32::from(nib);
        starts = [starts[1], starts[2], first];
        if recent & 0xFF_FFFF == DATA_MARK {
            from = Some(starts[0]);
            return false;
        }
        true
    });
    let from = from.ok_or("write: no data mark")?;

    let n = track.len();
    let window = (pos + n - LOOK_BACK % n) % n;
    let (mut to, mut header_end, mut count) = (None, None, 0usize);
    let (mut recent, mut starts) = (0u32, [0usize; 3]);
    frame(|i| track.bit((window + i) % n), 0..LOOK_BACK + LOOK_AHEAD + 2_000, |nib, first| {
        recent = recent << 8 | u32::from(nib);
        starts = [starts[1], starts[2], first];
        if recent & 0xFF_FFFF == ADDRESS_MARK && first <= LOOK_BACK + LOOK_AHEAD {
            header_end = Some(first + 7 * 8); // mark + 5 header nibbles + DE AA, roughly
            to = None;
            count = 0;
        } else if header_end.is_some() {
            count += 1;
            if recent & 0xFF_FFFF == DATA_MARK && to.is_none() && count <= 48 + 10 {
                to = Some(starts[0]);
            }
        }
        true
    });
    let to = to.ok_or("write: no data field on track")?;
    let at = (window + to) % n;
    Ok(((at + n - from % n) % n, overwrite(track, at, cells, from, written_len(cells, len))))
}

/// How near the end of the last Apple II write a new one may start and still be taken for its
/// continuation, in cells either way.
const A2_FOLLOW: usize = 48;
/// The gate position is the middle of the word the head is in (`drive.rs`): the head was within
/// this many cells of it.
const A2_POS_ERROR: usize = 16;
/// A continuation starts at least this many cells after the last write's end: turning write mode
/// off and on again takes an Apple II a few µs (DOS 3.3's formatter takes ~45 cycles, ~11 cells,
/// between an address field and its data field).
const A2_FOLLOW_GAP: usize = 2;

/// Where an Apple II write went on the track ([`splice_a2`]): `end` just past its last 1, and
/// `next` where the cell after its capture falls, which is where the rest of it goes.
#[derive(Clone, Copy)]
pub struct A2Write {
    pub end: usize,
    pub next: usize,
}

/// Lay an Apple II 5.25" write into a track that plays at the Apple II's own write cell (a sector
/// image's, while it's loaded; see `a2::TRACK_CELLS`), as a disk would take it. Returns where it
/// went, for the next one (`last`).
///
/// DOS 3.3's formatter writes each address field and each data field with the gate opened
/// afresh, ~11 cells apart, then reads the track back: sector 0's address field must come right
/// after sector 15 (otherwise it takes the drive for fast and writes the track again with less
/// sync, down to an I/O error). Core 1 keeps capturing across a gate that opens again that soon
/// and hands the rest over as a continuation (`rest`), which goes on from `last.next` cell for
/// cell. Otherwise a write with an address field in it (a formatter's), a format (`format`: a
/// gate open for a whole track), or one starting within [`A2_FOLLOW`] cells of the last one's end
/// is copied cell for cell at the head. That last kind goes at the earliest the head can have
/// been, but never over the last write: `pos` is only good to half a word. Anything else is a
/// sector write, laid over the data field it was for ([`splice_gcr`]).
pub fn splice_a2(
    track: &mut impl CellTrack,
    pos: usize,
    last: Option<A2Write>,
    cells: &impl Cells,
    len: usize,
    format: bool,
    rest: bool,
) -> Result<A2Write, &'static str> {
    let n = track.len();
    let end = written_len(cells, len);
    if rest && let Some(last) = last {
        // The last one's trailing 0s were cells of this write, not its gate's tail.
        for k in 0..(last.next + n - last.end) % n {
            track.set((last.end + k) % n, false);
        }
        let at = last.next;
        let end = if end == 0 { last.end } else { overwrite(track, at, cells, 0, end) };
        return Ok(A2Write { end, next: (at + len) % n });
    }
    if end == 0 {
        return Err("write: nothing written");
    }
    let pos = pos % n;
    let follows = last.map(|w| w.end).filter(|&a| (pos + n - a) % n <= A2_FOLLOW || (a + n - pos) % n <= A2_FOLLOW);
    let to = match follows {
        Some(a) => {
            let earliest = (pos + n - A2_POS_ERROR) % n;
            let past = (earliest + n - a) % n;
            Some(if (A2_FOLLOW_GAP..n / 2).contains(&past) { earliest } else { (a + A2_FOLLOW_GAP) % n })
        }
        None if format || has_address_mark(cells, end) => Some(pos),
        None => None,
    };
    let (at, end) = match to {
        Some(to) => (to, overwrite(track, to, cells, 0, end)),
        None => splice_gcr(track, pos, cells, len)?,
    };
    Ok(A2Write { end, next: (at + len) % n })
}

/// An Apple II address mark (`D5 AA 96`) in the first `len` cells. The capture may start in the
/// middle of sync (a gate that opened before the track was playing), with too few syncs left to
/// fall into step, so every framing is tried.
fn has_address_mark(cells: &impl Cells, len: usize) -> bool {
    (0..10.min(len)).any(|start| {
        let (mut recent, mut found) = (0u32, false);
        frame(|i| cells.bit(i), start..len, |nib, _| {
            recent = recent << 8 | u32::from(nib);
            found = recent & 0xFF_FFFF == ADDRESS_MARK;
            !found
        });
        found
    })
}

/// Splice an MFM data-field write into a stored track: the capture's first `A1 A1 A1` goes over
/// the `A1 A1 A1 FB` that follows the last ID field before `pos`. Returns where the write ended.
pub fn splice_mfm(track: &mut impl CellTrack, pos: usize, cells: &impl Cells, len: usize) -> Result<usize, &'static str> {
    let mut from = None;
    mfm_marks(&|i| cells.bit(i), 0..len, |mark, at| {
        if mark == 0xFB {
            from = Some(at);
            return false;
        }
        true
    });
    let from = from.ok_or("write: no data mark")?;

    let n = track.len();
    let window = (pos + n - LOOK_BACK % n) % n;
    let mut id_seen = false;
    let mut to = None;
    mfm_marks(&|i| track.bit((window + i) % n), 0..LOOK_BACK + LOOK_AHEAD + 2_000, |mark, at| {
        match mark {
            0xFE if at <= LOOK_BACK + LOOK_AHEAD => {
                id_seen = true;
                to = None;
            }
            0xFB if id_seen && to.is_none() => to = Some(at),
            _ => {}
        }
        true
    });
    let to = to.ok_or("write: no data field on track")?;
    Ok(overwrite(track, window + to, cells, from, written_len(cells, len)))
}

/// Every `A1 A1 A1 <mark>` in `range`: `f(mark, first_cell_of_the_A1s)`.
fn mfm_marks(bits: &impl Fn(usize) -> bool, range: core::ops::Range<usize>, mut f: impl FnMut(u8, usize) -> bool) {
    let mut window = 0u64;
    for i in range.clone() {
        window = (window << 1 | bits(i) as u64) & 0xFFFF_FFFF_FFFF;
        if window == MFM_SYNC && i + 17 <= range.end && !f(mfm_byte(bits, i + 1), i + 1 - 48) {
            return;
        }
    }
}


/// Copy a re-encoded track (exactly `track.len()` cells, MSB first) over a stored track.
pub fn copy_track(track: &mut impl CellTrack, buf: &TrackBuf) {
    for i in 0..track.len() {
        track.set(i, buf[i / 32] >> (31 - i % 32) & 1 != 0);
    }
}
