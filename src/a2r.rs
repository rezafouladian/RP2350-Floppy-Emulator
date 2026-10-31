//! A2R (Applesauce) flux captures of Apple II 5.25" disks, read-only: versions 1, 2 and 3.
//!
//! An A2R holds raw captures of every quarter track (several each, a revolution and more of flux
//! timings), not tracks: the loop has yet to be found. So:
//! - [`Indexer`] reads the container as `image::load`'s mapping pass streams the file's blocks
//!   past, noting each quarter track's first timing capture (A2R 3: its solved track if it has
//!   one, a single revolution), where its data is in the file, and its estimated loop point;
//! - `Cylinder` converts a capture's flux to cells as it's read (`moof::FluxCells`, at the
//!   track's own measured cell), and [`find_loop`] finds the revolution: where the cells match
//!   themselves one revolution on, near the loop point.
//!
//! Layouts (all 5.25" locations are quarter tracks):
//! - A2R 1 (not documented, as found): `A2R1 FF 0A 0D 0A`, 8 bytes, a `DATA` chunk with a
//!   big-endian size holding captures (location, type, big-endian data length, big-endian loop
//!   point in 125 ns ticks, data; a location of `FF` ends them), then an `INFO` chunk of META
//!   text. Earlier A2R 1 files (no `FF 0A 0D 0A`) aren't read.
//! - A2R 2: little-endian chunks; INFO (disk type, write protect, synchronized at 33–35); STRM:
//!   captures as A2R 1's but little-endian, `FF` ends them. 125 ns ticks.
//! - A2R 3: INFO as A2R 2's; RWCP: version, resolution (ps a tick, 62,500), reserved to 16, then
//!   `C` entries (type, u16 location, index count, index times, u32 length, data) to an `X`; SLVD:
//!   the same header, `T` entries (u16 location, mirror distances, reserved, index count, index
//!   times, u32 length, flux of one revolution) to an `X`.
//!
//! Capture type 1 is timing, 3 xtiming (more revolutions); type 2 (a deprecated bitstream) isn't
//! used.
//!
//! Hardware-free, so the host tests compile it directly.

/// Quarter tracks a capture can be at (as a WOZ's TMAP).
pub const LOCATIONS: usize = 160;

/// Where one quarter track's capture is in the file.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Capture {
    /// Its flux data: file byte offset and length.
    pub offset: u32,
    pub len: u32,
    /// The estimated loop point (one revolution), in ticks; 0 if not known.
    pub loop_ticks: u32,
    /// An A2R 3 solved track: the data is exactly one revolution.
    pub solved: bool,
}

pub struct A2r {
    /// 1, 2 or 3.
    pub version: u8,
    pub write_protected: bool,
    /// Imaged with cross-track sync: every capture starts at the same angle.
    pub synchronized: bool,
    /// Picoseconds a tick: 125,000 (A2R 1, 2), or the file's (A2R 3).
    pub tick_ps: u32,
    captures: [Capture; LOCATIONS],
}

impl A2r {
    /// Quarter track `q`'s capture, if it has one.
    pub fn capture(&self, q: u8) -> Option<Capture> {
        self.captures.get(q as usize).copied().filter(|c| c.len > 0)
    }

    /// The Disk II's 4 µs cell in 1/100 of this file's ticks (3,200 at 125 ns).
    pub fn nominal_cell(&self) -> u32 {
        // 4 µs = 4,000,000 ps, in hundredths of a tick.
        (400_000_000 / u64::from(self.tick_ps.max(1))) as u32
    }

    /// One revolution in this file's ticks, at 300 RPM: for a capture with no loop point.
    pub fn revolution_ticks(&self) -> u32 {
        (200_000_000_000 / u64::from(self.tick_ps.max(1))) as u32
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum St {
    Magic,
    /// A2R 1: the 8 bytes after the signature and the DATA chunk's header.
    A1Header,
    /// A2R 1 and 2: a capture's location, then the rest of its header.
    RecLoc,
    RecRest,
    /// A2R 2 and 3: a chunk header, and an INFO chunk's data.
    Chunk,
    Info,
    /// A2R 3: RWCP's or SLVD's header, an entry's mark, the rest of its header, its first index
    /// time, its data length.
    Head3,
    Mark3,
    Entry3,
    Index3,
    Len3,
    Done,
}

/// Reads an A2R's container from its bytes in order ([`feed`](Self::feed)), skipping the flux data.
pub struct Indexer {
    st: St,
    /// File bytes consumed, bytes to skip before the next field, and the field being collected.
    pos: u64,
    skip: u64,
    buf: [u8; 40],
    have: usize,
    want: usize,
    /// The end of the chunk whose records are being walked (A2R 1: of DATA).
    end: u64,
    /// The chunk being walked, for A2R 3: SLVD (solved) or RWCP.
    solved: bool,
    // The capture under way.
    loc: u16,
    kind: u8,
    count: u8,
    index0: u32,
    disk_type: u8,
    err: Option<&'static str>,
    a2r: A2r,
}

fn le32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

impl Indexer {
    pub fn new() -> Self {
        Self {
            st: St::Magic,
            pos: 0,
            skip: 0,
            buf: [0; 40],
            have: 0,
            want: 8,
            end: 0,
            solved: false,
            loc: 0,
            kind: 0,
            count: 0,
            index0: 0,
            disk_type: 1,
            err: None,
            a2r: A2r { version: 0, write_protected: true, synchronized: false, tick_ps: 125_000, captures: [Capture::default(); LOCATIONS] },
        }
    }

    /// The next `n` bytes are the field for state `st`.
    fn need(&mut self, st: St, n: usize) {
        self.st = st;
        self.want = n;
        self.have = 0;
    }

    fn fail(&mut self, e: &'static str) {
        self.err.get_or_insert(e);
        self.st = St::Done;
    }

    /// Note a capture at the current position: keep the first timing capture of each quarter
    /// track, or a solved track over it.
    fn capture(&mut self, len: u32, loop_ticks: u32) {
        let at = self.pos as u32;
        if let Some(c) = self.a2r.captures.get_mut(self.loc as usize) {
            let timing = matches!(self.kind, 1 | 3);
            if self.solved && !c.solved || timing && c.len == 0 {
                *c = Capture { offset: at, len, loop_ticks, solved: self.solved };
            }
        }
        self.skip = u64::from(len);
    }

    /// Feed the file's next bytes.
    pub fn feed(&mut self, mut bytes: &[u8]) {
        while !bytes.is_empty() && self.st != St::Done {
            if self.skip > 0 {
                let k = self.skip.min(bytes.len() as u64);
                self.skip -= k;
                self.pos += k;
                bytes = &bytes[k as usize..];
                continue;
            }
            // Walking a chunk's records: at its end, the next chunk.
            if self.have == 0 && matches!(self.st, St::RecLoc | St::Mark3) && self.pos >= self.end {
                if self.a2r.version == 1 {
                    self.st = St::Done; // what follows DATA is META text
                    continue;
                }
                self.need(St::Chunk, 8);
            }
            let k = (self.want - self.have).min(bytes.len());
            self.buf[self.have..self.have + k].copy_from_slice(&bytes[..k]);
            self.have += k;
            self.pos += k as u64;
            bytes = &bytes[k..];
            if self.have == self.want {
                self.step();
            }
        }
    }

    fn step(&mut self) {
        let b = self.buf;
        match self.st {
            St::Magic => {
                if &b[4..8] != b"\xFF\x0A\x0D\x0A" {
                    return self.fail(if &b[..4] == b"A2R1" { "A2R: an early A2R 1 layout" } else { "A2R: bad signature" });
                }
                match &b[..4] {
                    b"A2R1" => {
                        self.a2r.version = 1;
                        self.need(St::A1Header, 16);
                    }
                    b"A2R2" | b"A2R3" => {
                        self.a2r.version = b[3] - b'0';
                        self.need(St::Chunk, 8);
                    }
                    _ => self.fail("A2R: bad signature"),
                }
            }
            St::A1Header => {
                if &b[8..12] != b"DATA" {
                    return self.fail("A2R: A2R 1 without DATA");
                }
                self.end = self.pos + u64::from(be32(&b[12..16]));
                self.need(St::RecLoc, 1);
            }
            St::RecLoc => {
                if b[0] == 0xFF {
                    // The end of the captures.
                    self.skip = self.end.saturating_sub(self.pos);
                    self.need(St::RecLoc, 1);
                    return;
                }
                self.loc = u16::from(b[0]);
                self.need(St::RecRest, 9);
            }
            St::RecRest => {
                self.kind = b[0];
                let (len, lp) = if self.a2r.version == 1 { (be32(&b[1..5]), be32(&b[5..9])) } else { (le32(&b[1..5]), le32(&b[5..9])) };
                self.capture(len, lp);
                self.need(St::RecLoc, 1);
            }
            St::Chunk => {
                let size = u64::from(le32(&b[4..8]));
                self.end = self.pos + size;
                match &b[..4] {
                    b"INFO" if size >= 36 => {
                        self.skip = 0;
                        self.need(St::Info, 36);
                    }
                    b"STRM" if self.a2r.version == 2 => self.need(St::RecLoc, 1),
                    b"RWCP" | b"SLVD" if self.a2r.version == 3 => {
                        self.solved = &b[..4] == b"SLVD";
                        self.need(St::Head3, 16);
                    }
                    _ => {
                        self.skip = size;
                        self.need(St::Chunk, 8);
                    }
                }
            }
            St::Info => {
                self.disk_type = b[33];
                self.a2r.write_protected = b[34] == 1;
                self.a2r.synchronized = b[35] == 1;
                self.skip = self.end.saturating_sub(self.pos);
                self.need(St::Chunk, 8);
            }
            St::Head3 => {
                let ps = le32(&b[1..5]);
                if ps != 0 {
                    self.a2r.tick_ps = ps;
                }
                self.need(St::Mark3, 1);
            }
            St::Mark3 => match (b[0], self.solved) {
                (b'C', false) => self.need(St::Entry3, 4),
                (b'T', true) => self.need(St::Entry3, 11),
                _ => {
                    // `X`: the end of the entries.
                    self.skip = self.end.saturating_sub(self.pos);
                    self.need(St::Mark3, 1);
                }
            },
            St::Entry3 => {
                if self.solved {
                    self.loc = u16::from_le_bytes([b[0], b[1]]);
                    self.kind = 1;
                    self.count = b[10];
                } else {
                    self.kind = b[0];
                    self.loc = u16::from_le_bytes([b[1], b[2]]);
                    self.count = b[3];
                }
                self.index0 = 0;
                if self.count > 0 {
                    self.need(St::Index3, 4);
                } else {
                    self.need(St::Len3, 4);
                }
            }
            St::Index3 => {
                self.index0 = le32(&b[..4]);
                self.skip = 4 * u64::from(self.count - 1);
                self.need(St::Len3, 4);
            }
            St::Len3 => {
                let lp = if self.solved { 0 } else { self.index0 };
                self.capture(le32(&b[..4]), lp);
                self.need(St::Mark3, 1);
            }
            St::Done => {}
        }
    }

    /// The captures found, once the whole file has been fed.
    pub fn finish(self) -> Result<A2r, &'static str> {
        if let Some(e) = self.err {
            return Err(e);
        }
        if self.disk_type != 1 {
            return Err("A2R: not a 5.25\" disk");
        }
        if self.a2r.captures.iter().all(|c| c.len == 0) {
            return Err("A2R: no timing captures");
        }
        Ok(self.a2r)
    }
}

impl Default for Indexer {
    fn default() -> Self {
        Self::new()
    }
}

/// 32 cells from cell `i`, MSB first, of the cells whose byte `j` is `byte(j)`.
fn word_at(byte: &impl Fn(usize) -> u8, i: usize) -> u32 {
    let at = i / 8;
    let w = (0..5).fold(0u64, |w, k| w << 8 | u64::from(byte(at + k)));
    (w >> (8 - i % 8)) as u32
}

/// Cells of `a..a + 32 * words` that differ from those of `b..`.
fn mismatches(byte: &impl Fn(usize) -> u8, a: usize, b: usize, words: usize) -> u32 {
    (0..words).map(|k| (word_at(byte, a + 32 * k) ^ word_at(byte, b + 32 * k)).count_ones()).sum()
}

/// The cells in one revolution of a capture of `n` cells (MSB first, byte `j` being `byte(j)`),
/// played from cell `start`: the offset whose cells best continue those at `start`, where the
/// splice is; of equals, the nearest `estimate`.
///
/// With a loop point (`timed`), `estimate` is the cells at it (`FluxCells::watch`: one
/// revolution of the index sensor, true to a few cells), and only offsets within [`LOOP_SLACK`]
/// of it are tried, comparing only the splice: over a longer stretch real flux slips a cell now
/// and then (an interval rounded the other way) and the cells compared go out of step, while sync,
/// or one nibble over and over, matches itself every period (10 cells, or 8) anywhere.
///
/// Without one, `estimate` is 200 ms over the measured cell, out by the capturing drive's speed:
/// offsets within 1/16 of it are compared over stretches longer than a gap of sync at a few
/// places along the capture, the best few over a longer one, and the winner settled at the
/// splice.
/// `estimate` if the capture is too short to tell.
pub fn find_loop(byte: impl Fn(usize) -> u8, n: usize, start: usize, estimate: usize, timed: bool) -> usize {
    const SPLICE: usize = 8; // words, 256 cells
    // Longer than any gap of sync, at places a sector apart or less.
    const COARSE: usize = 32;
    const FINE: usize = 128;
    const ANCHORS: [usize; 5] = [0, 2_500, 5_000, 7_500, 10_000];
    let room = n.saturating_sub(start);
    let splice = |p: usize, slack: usize| {
        let (lo, hi) = (p.saturating_sub(slack).max(1), (p + slack).min(room.saturating_sub(32 * SPLICE)));
        (lo..=hi).map(|q| (mismatches(&byte, start, start + q, SPLICE), q.abs_diff(p), q)).min().map_or(p, |(_, _, q)| q)
    };
    if timed {
        return splice(estimate, LOOP_SLACK);
    }
    let d = estimate / 16;
    let lo = estimate.saturating_sub(d).max(1);
    let hi = (estimate + d).min(room.saturating_sub(32 * COARSE));
    if hi <= lo {
        return estimate;
    }
    let fits = |a: usize| a + hi + 32 * COARSE <= room;
    let mut best = [(u32::MAX, 0usize); 8];
    for p in lo..=hi {
        let m: u32 = ANCHORS.iter().filter(|&&a| a == 0 || fits(a)).map(|&a| mismatches(&byte, start + a, start + a + p, COARSE)).sum();
        if m < best[7].0 {
            best[7] = (m, p);
            best.sort_unstable();
        }
    }
    let fine = |p: usize| FINE.min(room.saturating_sub(p) / 32).max(COARSE);
    let p = best
        .iter()
        .filter(|&&(m, _)| m != u32::MAX)
        .map(|&(_, p)| (u64::from(mismatches(&byte, start, start + p, fine(p))) * 1024 / fine(p) as u64, p))
        .min()
        .map_or(estimate, |(_, p)| p);
    splice(p, 3)
}

/// Where to cut a capture of `n` cells into its revolution, given the revolution of `p` cells
/// [`find_loop`] found from `start`: `(s, q)`, the cells `s..s + q` being played. A capture runs
/// on past a revolution, so `s` can be anywhere from `start` to its end less `p`, and is put in a
/// run of sync (five or more `FF`s, framed as a Disk II does) so no field crosses the splice: the
/// last such run where the revolutions line up, taking as much of the overlap as can from the
/// second (which the first may have read worse, a damaged spot read badly once). The comparison
/// there runs from the sync into the field after it (an address field: nothing else matches it),
/// so `q` is settled to the cell, within [`LOOP_SLACK`] of `p`: [`find_loop`] compares where the
/// capture happens to start, which may be in data of one nibble over and over, matching itself
/// every 8 cells; and real flux slips a cell now and then between revolutions. `(start, p)` if
/// no run lines up.
pub fn splice_point(byte: impl Fn(usize) -> u8, n: usize, start: usize, p: usize) -> (usize, usize) {
    const RUNS: usize = 32;
    let bit = |i: usize| byte(i / 8) >> (7 - i % 8) & 1 != 0;
    let last = n.saturating_sub(p + LOOP_SLACK + 32 * 8);
    // The middles of runs of sync, the last few kept.
    let mut runs = [0usize; RUNS];
    let mut count = 0usize;
    let (mut reg, mut syncs, mut run_start) = (0u8, 0usize, start);
    for i in start..last {
        if reg == 0 && !bit(i) {
            continue;
        }
        reg = reg << 1 | bit(i) as u8;
        if reg & 0x80 == 0 {
            continue;
        }
        if reg == 0xFF {
            if syncs == 0 {
                run_start = i;
            }
            syncs += 1;
            if syncs == 5 {
                // Noted once a run has five; its middle once it ends (below), or here for now.
                runs[count % RUNS] = (run_start + i) / 2;
                count += 1;
            } else if syncs > 5 {
                runs[(count - 1) % RUNS] = (run_start + i) / 2;
            }
        } else {
            syncs = 0;
        }
        reg = 0;
    }
    for k in (count.saturating_sub(RUNS)..count).rev() {
        let s = runs[k % RUNS];
        let (m, _, q) = (p.saturating_sub(LOOP_SLACK)..=p + LOOP_SLACK)
            .map(|q| (mismatches(&byte, s, s + q, 8), q.abs_diff(p), q))
            .min()
            .unwrap_or((u32::MAX, 0, p));
        if m <= 8 {
            return (s, q);
        }
    }
    (start, p)
}

/// How far from its estimate [`find_loop`] looks for a revolution's end, in cells.
pub const LOOP_SLACK: usize = 24;
