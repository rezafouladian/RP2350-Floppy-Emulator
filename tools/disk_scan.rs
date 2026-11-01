//! Scan Apple II 5.25" disk captures for copy protection (host tool).
//!
//! Reads A2R (1, 2 and 3: Applesauce's flux captures) and WOZ (1, 2) files, takes one revolution
//! of each quarter track, and reports per disk what isn't a standard DOS 3.3 / ProDOS (16-sector)
//! or DOS 3.2 (13-sector) format, then a summary of each feature across all disks with examples.
//! It looks for:
//! - marks after sync that aren't `D5 AA 96` / `D5 AA B5` / `D5 AA AD` (custom prologues);
//! - address and data fields with bad checksums or epilogues other than `DE AA`;
//! - address fields whose track number isn't the track they're on, odd sector counts, odd volumes;
//! - formatted tracks with no standard fields (a custom format or a nibble-only loader);
//! - data on half or quarter tracks that isn't just the neighbouring track heard (its own track
//!   numbers, or nothing at the whole tracks around it);
//! - tracks past 34;
//! - tracks written at another speed (their cell differs from the disk's: a WOZ stores that only
//!   as a longer or shorter track, and the firmware plays all of a disk's tracks at one cell);
//! - runs of 4 or more cells with no flux (weak bits: a Disk II's MC3470 makes up bits there) and,
//!   from flux, reversals half way between cells (fuzzy bits).
//!
//! A2R 1 isn't documented: as found, after the 8-byte header come 8 bytes (`01 00 8D 00 A5 01 00
//! 00`), a `DATA` chunk with a big-endian size holding the captures (location, type, big-endian
//! data length, big-endian loop point in 125 ns ticks, data; a location of `FF` ends it) and an
//! `INFO` chunk, also big-endian, holding the META text. Earlier A2R 1 files (no `FF 0A 0D 0A`,
//! no `DATA`) and `A2R0` aren't read. A2R 2 and 3 are as Applesauce documents.
//!
//!   rustc --edition 2024 -O tools/disk_scan.rs -o target/disk_scan && target/disk_scan <dir or file>... [--disks] [--examples N]
//!
//! `--disks` prints a line per disk with what was found; the summary always prints.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Ticks of 125 ns per cell at the Disk II's 4 µs.
const CELL_TICKS: f64 = 32.0;
/// A revolution at 300 RPM, in 125 ns ticks, when a capture doesn't say.
const REV_TICKS: f64 = 1_600_000.0;

/// One quarter track, as cells: `bits` holds at least a revolution (a capture's all, or a
/// bitstream twice over), `len` the cells of one. `cell` is the measured cell in 125 ns ticks (0
/// for a bitstream, which has none).
struct Rev {
    bits: Vec<bool>,
    len: usize,
    cell: f64,
    /// In the first revolution, runs of 3–6 zero cells, 7–15 (write splices) and 16 or more.
    gaps: u32,
    splices: u32,
    voids: u32,
    /// The longest run of zero cells.
    longest: usize,
}

impl Rev {
    fn count_zeros(&mut self) {
        let mut zeros = 0usize;
        for &b in &self.bits[..self.len] {
            if !b {
                zeros += 1;
                continue;
            }
            match zeros {
                3..=6 => self.gaps += 1,
                7..=15 => self.splices += 1,
                16.. => self.voids += 1,
                _ => {}
            }
            self.longest = self.longest.max(zeros);
            zeros = 0;
        }
    }
}

/// What was found on one quarter track.
#[derive(Default, Clone)]
struct Track {
    formatted: bool,
    cells: usize,
    cell: f64,
    addr16: u32,
    addr13: u32,
    addr_bad: u32,
    addr_epilogue: u32,
    data_ok: u32,
    data_bad: u32,
    data_epilogue: u32,
    sectors: u32,
    /// The track numbers the address fields give, and their volumes.
    tracks: BTreeMap<u8, u32>,
    volumes: BTreeMap<u8, u32>,
    /// Three nibbles after each run of 4+ sync `FF`s.
    marks: BTreeMap<[u8; 3], u32>,
    gaps: u32,
    voids: u32,
    longest: usize,
    /// Flux: the longest stretch of a second capture whose 8-nibble sequences the first doesn't
    /// have (one bit misread spoils 8; weak bits a long stretch).
    unstable: f64,
}

const WRITE_62: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA,
    0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3, 0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7,
    0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];
const WRITE_53: [u8; 32] = [
    0xAB, 0xAD, 0xAE, 0xAF, 0xB5, 0xB6, 0xB7, 0xBA, 0xBB, 0xBD, 0xBE, 0xBF, 0xD6, 0xD7, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xEA, 0xEB, 0xED,
    0xEE, 0xEF, 0xF5, 0xF6, 0xF7, 0xFA, 0xFB, 0xFD, 0xFE, 0xFF,
];

fn tables() -> ([u8; 256], [u8; 256]) {
    let (mut r62, mut r53) = ([0xFFu8; 256], [0xFFu8; 256]);
    for (i, &n) in WRITE_62.iter().enumerate() {
        r62[n as usize] = i as u8;
    }
    for (i, &n) in WRITE_53.iter().enumerate() {
        r53[n as usize] = i as u8;
    }
    (r62, r53)
}

// ---- Reading the files ----

fn be32(d: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_be_bytes(d.get(at..at + 4)?.try_into().ok()?))
}

fn le32(d: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(d.get(at..at + 4)?.try_into().ok()?))
}

/// A capture: its quarter track, kind (1 timing, 2 bits, 3 xtiming), revolution in ticks (0 if
/// not known), tick in 125 ns units, and data.
struct Capture<'a> {
    loc: u8,
    kind: u8,
    rev: f64,
    tick: f64,
    data: &'a [u8],
}

struct Disk<'a> {
    format: &'static str,
    /// Imaged with cross-track sync (A2R 2/3's INFO; unknown for A2R 1 and WOZ 1).
    synced: Option<bool>,
    /// 1 = 5.25"; anything else is skipped.
    disk_type: u8,
    captures: Vec<Capture<'a>>,
    /// WOZ: quarter track → bits.
    woz: BTreeMap<u8, Vec<bool>>,
}

fn a2r(d: &[u8]) -> Result<Disk<'_>, String> {
    let mut disk = Disk { format: "", synced: None, disk_type: 1, captures: Vec::new(), woz: BTreeMap::new() };
    match &d[..4] {
        b"A2R1" => {
            disk.format = "A2R1";
            if &d[4..8] != b"\xFF\x0A\x0D\x0A" || d.get(16..20) != Some(b"DATA") {
                return Err("an early A2R 1 layout (no line-ending check or DATA chunk), not read".into());
            }
            let end = 24 + be32(d, 20).ok_or("short")? as usize;
            let mut j = 24;
            while j + 10 <= end.min(d.len()) && d[j] != 0xFF {
                let (loc, kind) = (d[j], d[j + 1]);
                let (len, lp) = (be32(d, j + 2).ok_or("short")? as usize, be32(d, j + 6).ok_or("short")?);
                let data = d.get(j + 10..j + 10 + len).ok_or("capture past the end")?;
                disk.captures.push(Capture { loc, kind, rev: f64::from(lp), tick: 1.0, data });
                j += 10 + len;
            }
        }
        b"A2R2" => {
            disk.format = "A2R2";
            let mut i = 8;
            while i + 8 <= d.len() {
                let (id, n) = (&d[i..i + 4], le32(d, i + 4).unwrap() as usize);
                let body = d.get(i + 8..i + 8 + n).ok_or("chunk past the end")?;
                match id {
                    b"INFO" if n >= 36 => {
                        disk.disk_type = body[33];
                        disk.synced = Some(body[35] == 1);
                    }
                    b"STRM" => {
                        let mut j = 0;
                        while j + 10 <= body.len() && body[j] != 0xFF {
                            let (loc, kind) = (body[j], body[j + 1]);
                            let (len, lp) = (le32(body, j + 2).unwrap() as usize, le32(body, j + 6).unwrap());
                            let data = body.get(j + 10..j + 10 + len).ok_or("capture past the end")?;
                            disk.captures.push(Capture { loc, kind, rev: f64::from(lp), tick: 1.0, data });
                            j += 10 + len;
                        }
                    }
                    _ => {}
                }
                i += 8 + n;
            }
        }
        b"A2R3" => {
            disk.format = "A2R3";
            let mut i = 8;
            while i + 8 <= d.len() {
                let (id, n) = (&d[i..i + 4], le32(d, i + 4).unwrap() as usize);
                let body = d.get(i + 8..i + 8 + n).ok_or("chunk past the end")?;
                match id {
                    b"INFO" if n >= 36 => {
                        disk.disk_type = body[33];
                        disk.synced = Some(body[35] == 1);
                    }
                    b"RWCP" => {
                        let tick = f64::from(le32(body, 1).unwrap()) / 125_000.0;
                        let mut j = 16;
                        while j < body.len() && body[j] == b'C' {
                            let kind = body[j + 1];
                            let loc = u16::from_le_bytes([body[j + 2], body[j + 3]]);
                            let count = body[j + 4] as usize;
                            let index = le32(body, j + 5).unwrap_or(0);
                            let at = j + 5 + 4 * count;
                            let len = le32(body, at).ok_or("short")? as usize;
                            let data = body.get(at + 4..at + 4 + len).ok_or("capture past the end")?;
                            let rev = if count > 0 { f64::from(index) * tick } else { 0.0 };
                            disk.captures.push(Capture { loc: loc.min(255) as u8, kind, rev, tick, data });
                            j = at + 4 + len;
                        }
                    }
                    _ => {}
                }
                i += 8 + n;
            }
        }
        _ => return Err("not an A2R".into()),
    }
    Ok(disk)
}

fn woz(d: &[u8]) -> Result<Disk<'_>, String> {
    let mut disk = Disk { format: "", synced: None, disk_type: 1, captures: Vec::new(), woz: BTreeMap::new() };
    let v1 = match &d[..4] {
        b"WOZ1" => true,
        b"WOZ2" => false,
        _ => return Err("not a WOZ".into()),
    };
    disk.format = if v1 { "WOZ1" } else { "WOZ2" };
    let (mut tmap, mut trks) = (None, None);
    let mut i = 12;
    while i + 8 <= d.len() {
        let (id, n) = (&d[i..i + 4], le32(d, i + 4).unwrap() as usize);
        let at = i + 8;
        match id {
            b"INFO" => {
                disk.disk_type = d[at + 1];
                if !v1 {
                    disk.synced = Some(d[at + 3] == 1);
                }
            }
            b"TMAP" => tmap = Some(at),
            b"TRKS" => trks = Some(at),
            _ => {}
        }
        i = at + n;
    }
    let (tmap, trks) = (tmap.ok_or("no TMAP")?, trks.ok_or("no TRKS")?);
    for q in 0..160u8 {
        let t = d[tmap + q as usize];
        if t == 0xFF {
            continue;
        }
        let (bytes, count) = if v1 {
            let rec = trks + t as usize * 6656;
            let count = u16::from_le_bytes([d[rec + 6646], d[rec + 6647]]) as usize;
            (&d[rec..rec + 6646], count)
        } else {
            let rec = trks + t as usize * 8;
            let start = u16::from_le_bytes([d[rec], d[rec + 1]]) as usize * 512;
            let blocks = u16::from_le_bytes([d[rec + 2], d[rec + 3]]) as usize;
            let count = le32(d, rec + 4).unwrap() as usize;
            (d.get(start..start + blocks * 512).ok_or("track past the end")?, count)
        };
        let bits = (0..count.min(bytes.len() * 8)).map(|i| bytes[i / 8] >> (7 - i % 8) & 1 != 0).collect();
        disk.woz.insert(q, bits);
    }
    Ok(disk)
}

/// A timing capture as cells, through a simple PLL (the cell follows the drive's speed through
/// the track), with where one revolution ends.
fn flux_rev(c: &Capture) -> Rev {
    let mut iv = Vec::with_capacity(c.data.len());
    let mut acc = 0u32;
    for &b in c.data {
        acc += u32::from(b);
        if b != 255 {
            iv.push(f64::from(acc) * c.tick);
            acc = 0;
        }
    }
    let rev = if c.rev > 0.5 * REV_TICKS { c.rev } else { REV_TICKS };
    let mut cell = CELL_TICKS;
    let (mut t, mut len, mut sum, mut count) = (0.0, 0usize, 0.0, 0usize);
    let mut bits = Vec::with_capacity(c.data.len() * 2);
    for &x in &iv {
        let n = ((x / cell).round() as usize).max(1);
        if n <= 3 {
            // Follow the drive (and the media's own drift), slowly.
            cell += 0.05 * (x / n as f64 - cell);
            cell = cell.clamp(0.85 * CELL_TICKS, 1.15 * CELL_TICKS);
        }
        bits.extend(std::iter::repeat_n(false, n - 1));
        bits.push(true);
        t += x;
        if t < rev {
            len = bits.len();
            sum += cell;
            count += 1;
        }
    }
    let mut r = Rev { bits, len, cell: sum / count.max(1) as f64, gaps: 0, splices: 0, voids: 0, longest: 0 };
    r.count_zeros();
    r
}

/// A bitstream (A2R's deprecated bits capture, or a WOZ track): one revolution, no timing.
fn bits_rev(bits: Vec<bool>) -> Rev {
    let len = bits.len();
    let mut twice = bits.clone();
    twice.extend_from_slice(&bits);
    let mut r = Rev { bits: twice, len, cell: 0.0, gaps: 0, splices: 0, voids: 0, longest: 0 };
    r.count_zeros();
    r
}

// ---- Decoding a track ----

/// Nibbles as a Disk II frames them, with the cell each starts at.
fn nibbles(bits: &[bool]) -> Vec<(u8, usize)> {
    let mut out = Vec::with_capacity(bits.len() / 4);
    let (mut reg, mut start) = (0u8, 0usize);
    for (i, &b) in bits.iter().enumerate() {
        if reg == 0 {
            if !b {
                continue;
            }
            start = i;
        }
        reg = reg << 1 | b as u8;
        if reg & 0x80 != 0 {
            out.push((reg, start));
            reg = 0;
        }
    }
    out
}

fn decode(rev: &Rev, r62: &[u8; 256], r53: &[u8; 256]) -> Track {
    let n = nibbles(&rev.bits);
    let len = rev.len;
    let mut t = Track { cells: len, cell: rev.cell, gaps: rev.gaps, voids: rev.voids, longest: rev.longest, ..Default::default() };
    // Formatted: mostly valid disk bytes (6-and-2's, plus the marks').
    let first: Vec<u8> = n.iter().take_while(|&&(_, s)| s < len).map(|&(b, _)| b).collect();
    let valid = first.iter().filter(|&&b| r62[b as usize] != 0xFF || r53[b as usize] != 0xFF || b == 0xD5 || b == 0xAA).count();
    t.formatted = first.len() > 3_000 && valid * 100 >= first.len() * 85;
    if !t.formatted {
        return t;
    }
    let b: Vec<u8> = n.iter().map(|&(b, _)| b).collect();
    let in_first = |i: usize| n[i].1 < len;
    let mut syncs = 0;
    // The kind of the last address field: its data field is the same kind. A 5-and-3 field's
    // nibbles are all 6-and-2 ones too, and its first 343 pass a 6-and-2 checksum one time in 64.
    let mut last13 = false;
    for i in 0..b.len() {
        if b[i] == 0xFF {
            syncs += 1;
            continue;
        }
        if syncs >= 4 && in_first(i) && i + 3 <= b.len() {
            *t.marks.entry([b[i], b[i + 1], b[i + 2]]).or_default() += 1;
        }
        syncs = 0;
        if !in_first(i) || i + 3 > b.len() || b[i] != 0xD5 || b[i + 1] != 0xAA {
            continue;
        }
        let c = b[i + 2];
        if (c == 0x96 || c == 0xB5) && i + 13 <= b.len() {
            let f = |k: usize| (b[i + 3 + 2 * k] << 1 | 1) & b[i + 4 + 2 * k];
            let (vol, trk, sec, chk) = (f(0), f(1), f(2), f(3));
            if vol ^ trk ^ sec != chk {
                t.addr_bad += 1;
                continue;
            }
            if c == 0x96 { t.addr16 += 1 } else { t.addr13 += 1 }
            last13 = c == 0xB5;
            *t.tracks.entry(trk).or_default() += 1;
            *t.volumes.entry(vol).or_default() += 1;
            if sec < 32 {
                t.sectors |= 1 << sec;
            }
            if b[i + 11] != 0xDE || b[i + 12] != 0xAA {
                t.addr_epilogue += 1;
            }
        } else if c == 0xAD {
            // 6-and-2 (343 nibbles) or 5-and-3 (411): the XOR of all is 0 when the checksum holds.
            let field = |count: usize, table: &[u8; 256]| -> Option<bool> {
                let body = b.get(i + 3..i + 3 + count)?;
                let mut x = 0u8;
                for &v in body {
                    let v = table[v as usize];
                    if v == 0xFF {
                        return Some(false);
                    }
                    x ^= v;
                }
                Some(x == 0)
            };
            let (ok62, ok53) = (field(343, r62), field(411, r53));
            let (ok, count) = match (ok62, ok53) {
                (_, Some(true)) if last13 => (true, 411),
                (Some(true), _) => (true, 343),
                (_, Some(true)) => (true, 411),
                _ => (false, 343),
            };
            if ok {
                t.data_ok += 1;
                if b.get(i + 3 + count) != Some(&0xDE) || b.get(i + 4 + count) != Some(&0xAA) {
                    t.data_epilogue += 1;
                }
            } else {
                t.data_bad += 1;
            }
        }
    }
    t
}

/// The longest run of `b`'s 8-nibble sequences (one per nibble) that `a` doesn't have: under 8
/// or so for a track that reads the same each time, longer where it doesn't.
fn unstable(a: &[bool], b: &[bool]) -> f64 {
    let key = |w: &[(u8, usize)]| w.iter().fold(0u64, |k, &(n, _)| k << 8 | u64::from(n));
    let (na, nb) = (nibbles(a), nibbles(b));
    let seen: std::collections::HashSet<u64> = na.windows(8).map(key).collect();
    let (mut run, mut longest) = (0usize, 0usize);
    for w in nb.windows(8) {
        if seen.contains(&key(w)) {
            run = 0;
        } else {
            run += 1;
            longest = longest.max(run);
        }
    }
    longest as f64
}

// ---- Judging a disk ----

#[derive(Default)]
struct Report {
    format: &'static str,
    synced: Option<bool>,
    flags: BTreeMap<&'static str, String>,
}

fn judge(tracks: &BTreeMap<u8, Track>) -> BTreeMap<&'static str, String> {
    let mut f: BTreeMap<&'static str, String> = BTreeMap::new();
    let mut add = |k: &'static str, v: String| {
        let e = f.entry(k).or_default();
        if e.len() >= 160 {
            if !e.ends_with('…') {
                e.push_str(", …");
            }
            return;
        }
        if !e.is_empty() {
            e.push_str(", ");
        }
        e.push_str(&v);
    };
    let whole = |q: u8| tracks.get(&q).filter(|t| t.formatted);
    // The disk's cell: the median of its formatted whole tracks' (from flux).
    let mut cells: Vec<f64> = (0..40u8).filter_map(|t| whole(t * 4)).map(|t| t.cell).filter(|&c| c > 0.0).collect();
    cells.sort_by(f64::total_cmp);
    let median = cells.get(cells.len() / 2).copied().unwrap_or(0.0);
    let mut lens: Vec<usize> = (0..40u8).filter_map(|t| whole(t * 4)).map(|t| t.cells).collect();
    lens.sort();
    let median_len = lens.get(lens.len() / 2).copied().unwrap_or(0);
    let mut thirteen = 0;
    let mut sixteen = 0;
    for (&q, t) in tracks {
        if !t.formatted {
            continue;
        }
        let (track, quarter) = (q / 4, q % 4);
        let label = if quarter == 0 { format!("{track}") } else { format!("{track}.{:02}", quarter * 25) };
        let standard = t.addr16 + t.addr13;
        if quarter != 0 {
            // Heard from a neighbour? Then its address fields name that track, and that track is
            // formatted.
            let near = [q / 4 * 4, q.div_ceil(4) * 4];
            let own_numbers = t.tracks.keys().any(|&n| !near.iter().any(|&w| w / 4 == n));
            let neighbours_blank = near.iter().all(|&w| whole(w).is_none());
            if own_numbers || neighbours_blank {
                add("data on half/quarter tracks", label.clone());
            }
            continue;
        }
        if track >= 35 {
            add("tracks past 34", label.clone());
        }
        if standard == 0 {
            add("formatted, no standard address fields", label.clone());
        }
        if t.addr13 > t.addr16 {
            thirteen += 1;
        } else if t.addr16 > 0 {
            sixteen += 1;
        }
        if t.addr_bad > 0 {
            add("address field checksum errors", format!("{label}×{}", t.addr_bad));
        }
        if t.addr_epilogue > 0 {
            add("address epilogue not DE AA", label.clone());
        }
        if t.data_bad > 0 && standard > 0 {
            add("data field errors (bad checksum or nibbles)", format!("{label}×{}", t.data_bad));
        }
        if t.data_epilogue > 0 {
            add("data epilogue not DE AA", label.clone());
        }
        if standard > 0 && t.data_ok == 0 && t.data_bad == 0 {
            add("address fields, no data fields", label.clone());
        }
        if let Some((&n, _)) = t.tracks.iter().max_by_key(|&(_, c)| *c)
            && n != track
        {
            add("track number isn't the track", format!("{label}→{n}"));
        }
        let count = t.sectors.count_ones();
        if standard > 0 && count != 16 && count != 13 {
            add("sector count not 13 or 16", format!("{label}:{count}"));
        }
        if t.volumes.keys().any(|&v| v != 254) && standard > 0 {
            let v: Vec<String> = t.volumes.keys().map(|v| format!("{v:02X}")).collect();
            if v.len() > 1 {
                add("volume differs within a track", label.clone());
            }
        }
        let odd: Vec<String> = t
            .marks
            .iter()
            .filter(|&(m, &c)| c >= 4 && !matches!(m, [0xD5, 0xAA, 0x96] | [0xD5, 0xAA, 0xAD] | [0xD5, 0xAA, 0xB5]))
            .map(|(m, c)| format!("{label}:{:02X}{:02X}{:02X}×{c}", m[0], m[1], m[2]))
            .collect();
        for o in odd.into_iter().take(2) {
            add("custom marks after sync", o);
        }
        if t.cell > 0.0 && median > 0.0 && ((t.cell / median) - 1.0).abs() > 0.04 {
            add("track at another bit rate", format!("{label}:{:+.1}%", (t.cell / median - 1.0) * 100.0));
        } else if t.cell == 0.0 && median_len > 0 && ((t.cells as f64 / median_len as f64) - 1.0).abs() > 0.03 {
            add("track length differs", format!("{label}:{:+.1}%", (t.cells as f64 / median_len as f64 - 1.0) * 100.0));
        }
        // A write splice per sector is normal; weak-bit areas are more, or longer.
        if t.voids > 20 || t.longest >= 40 {
            add("no-flux areas (weak bits)", format!("{label}:{}×16+, longest {}", t.voids, t.longest));
        }
        if t.gaps > 8 {
            add("runs of 3-6 zero cells (not in GCR)", format!("{label}×{}", t.gaps));
        }
        // A misread bit can put the framing out of step until the next sync (a data field's worth),
        // so only much more than that counts.
        if t.unstable >= 500.0 {
            add("reads differently each time (weak bits, or damage)", format!("{label}:{} nibbles", t.unstable as usize));
        }
    }
    if thirteen > 0 && sixteen > 0 {
        add("13- and 16-sector tracks mixed", format!("{thirteen}/{sixteen}"));
    } else if thirteen > 0 {
        add("13-sector (DOS 3.2)", format!("{thirteen} tracks"));
    }
    // Unformatted whole tracks between formatted ones.
    let formatted: Vec<u8> = (0..40u8).filter(|&t| whole(t * 4).is_some()).collect();
    if let (Some(&lo), Some(&hi)) = (formatted.first(), formatted.last()) {
        let holes: Vec<String> = (lo..hi).filter(|t| !formatted.contains(t)).map(|t| t.to_string()).collect();
        if !holes.is_empty() {
            add("blank whole tracks between formatted ones", holes.join(" "));
        }
    }
    f
}

fn scan_file(path: &Path, r62: &[u8; 256], r53: &[u8; 256]) -> Result<Report, String> {
    let d = std::fs::read(path).map_err(|e| e.to_string())?;
    if d.len() < 16 {
        return Err("too short".into());
    }
    let disk = if d.starts_with(b"WOZ") { woz(&d)? } else { a2r(&d)? };
    if disk.disk_type != 1 {
        return Err(format!("not a 5.25\" disk (type {})", disk.disk_type));
    }
    let mut tracks = BTreeMap::new();
    if disk.format.starts_with("WOZ") {
        for (&q, bits) in &disk.woz {
            tracks.insert(q, decode(&bits_rev(bits.clone()), r62, r53));
        }
    } else {
        // The first timing capture of each location, else its bitstream; and a second timing
        // capture, to compare.
        let mut best: BTreeMap<u8, &Capture> = BTreeMap::new();
        let mut second: BTreeMap<u8, &Capture> = BTreeMap::new();
        for c in &disk.captures {
            let better = match best.get(&c.loc) {
                None => true,
                Some(b) => b.kind == 2 && c.kind != 2,
            };
            if better {
                best.insert(c.loc, c);
            } else if c.kind != 2 && !second.contains_key(&c.loc) {
                second.insert(c.loc, c);
            }
        }
        for (&q, c) in &best {
            let rev = if c.kind == 2 {
                let bits: Vec<bool> = (0..c.data.len() * 8).map(|i| c.data[i / 8] >> (7 - i % 8) & 1 != 0).collect();
                let n = if c.rev > 0.0 { (c.rev / CELL_TICKS) as usize } else { 50_000 };
                let mut r = bits_rev(bits[..n.min(bits.len())].to_vec());
                // The capture goes on past the revolution: decode that, not a wrap.
                r.bits = bits;
                r
            } else {
                flux_rev(c)
            };
            let mut t = decode(&rev, r62, r53);
            if t.formatted
                && c.kind != 2
                && let Some(c2) = second.get(&q)
            {
                let other = flux_rev(c2);
                t.unstable = unstable(&rev.bits, &other.bits[..other.len]);
            }
            tracks.insert(q, t);
        }
    }
    Ok(Report { format: disk.format, synced: disk.synced, flags: judge(&tracks) })
}

fn collect(path: &Path, out: &mut Vec<PathBuf>) {
    if path.is_dir() {
        let Ok(rd) = std::fs::read_dir(path) else { return };
        let mut entries: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.path()).collect();
        entries.sort();
        for e in entries {
            collect(&e, out);
        }
    } else if let Some(ext) = path.extension().and_then(|e| e.to_str())
        && (ext.eq_ignore_ascii_case("a2r") || ext.eq_ignore_ascii_case("woz"))
    {
        out.push(path.to_path_buf());
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let per_disk = args.iter().any(|a| a == "--disks");
    let examples = args.iter().position(|a| a == "--examples").and_then(|i| args.get(i + 1)?.parse().ok()).unwrap_or(6usize);
    let mut files = Vec::new();
    for a in args.iter().filter(|a| !a.starts_with("--") && a.parse::<usize>().is_err()) {
        collect(Path::new(a), &mut files);
    }
    let (r62, r53) = tables();
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<(usize, Result<Report, String>)>> = Mutex::new(Vec::new());
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get());
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(f) = files.get(i) else { break };
                    let r = scan_file(f, &r62, &r53);
                    results.lock().unwrap().push((i, r));
                }
            });
        }
    });
    let mut results = results.into_inner().unwrap();
    results.sort_by_key(|(i, _)| *i);

    let name = |i: usize| {
        let p = &files[i];
        let parent = p.parent().and_then(|p| p.file_name()).map(|s| s.to_string_lossy()).unwrap_or_default();
        format!("{}/{}", parent, p.file_name().unwrap().to_string_lossy())
    };
    let mut by_flag: BTreeMap<&'static str, Vec<usize>> = BTreeMap::new();
    let (mut clean, mut errors, mut synced) = (0, 0, 0);
    let mut formats: BTreeMap<&str, usize> = BTreeMap::new();
    for (i, r) in &results {
        match r {
            Ok(r) => {
                *formats.entry(r.format).or_default() += 1;
                synced += (r.synced == Some(true)) as usize;
                if r.flags.is_empty() {
                    clean += 1;
                }
                for k in r.flags.keys() {
                    by_flag.entry(k).or_default().push(*i);
                }
                if per_disk {
                    println!("{}", name(*i));
                    for (k, v) in &r.flags {
                        println!("    {k}: {v}");
                    }
                }
            }
            Err(e) => {
                errors += 1;
                if per_disk {
                    println!("{}: {e}", name(*i));
                }
            }
        }
    }
    println!(
        "\n{} disks ({:?}), {} unreadable; {} standard throughout; {} imaged with cross-track sync",
        results.len(),
        formats,
        errors,
        clean,
        synced
    );
    let mut flags: Vec<_> = by_flag.into_iter().collect();
    flags.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    for (k, v) in flags {
        println!("\n{k}: {} disks", v.len());
        for &i in v.iter().take(examples) {
            let detail = results.iter().find(|(j, _)| *j == i).and_then(|(_, r)| r.as_ref().ok()).and_then(|r| r.flags.get(k)).cloned().unwrap_or_default();
            println!("    {}  [{}]", name(i), detail);
        }
    }
}
