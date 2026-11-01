//! The cylinder under the head: its blocks from the card, the track buffers built from them,
//! and the Mac's writes applied to both and saved back to the card.
//!
//! Block layout in `blocks`, by format:
//! - raw GCR (also DiskCopy and an Apple II 3.5" `.2mg`, shifted into place): side 0's sectors in
//!   sector order, then side 1's;
//! - raw MFM: side 0's sectors 1–9 (720K) or 1–18 (1.44 MB), then side 1's;
//! - MOOF: side `s`'s track data at `s * MAX_TRACK_BLOCKS`, as stored in the file; a flux track
//!   is converted to cells on the way in, so it's held like a bitstream track;
//! - Apple II: one track (one side). A sector image's track is 8 blocks of two 256-byte sectors
//!   each, in the file's order (`a2::file_sector`); a 13-sector (`.d13`) track's 6.5 blocks are
//!   read whole from the block it starts in, its sectors from byte 0 or 256 of the first
//!   ([`a2_slot`]); and the track as it plays, in cells, at
//!   [`A2_CELLS`] (writes go into those, and the sectors are decoded back out); a `.nib` track 13
//!   blocks of nibbles; a WOZ track its data as stored (a WOZ 1 record is read 256 bytes in, so it
//!   starts at block 0 too). The track is a key from `Disk::a2_track`, not the head's quarter
//!   track.

use embedded_sdmmc::{Block, BlockDevice, BlockIdx};

use crate::drive::{self, CAP_FORMAT, CAP_REST, CAP_SIDE};
use crate::a2r::{self, A2r};
use crate::woz;
use crate::image::{BlockMap, Disk};
use crate::moof::{self, Moof};
use crate::track::{BLOCK, MAX_CYLINDER_BLOCKS, MAX_TRACK_BLOCKS, MAX_TRACK_WORDS, TrackBuf};
use crate::weak::{self, WeakList};
use crate::writes::{self, A2Write, CellTrack, Cells, MfmField};
use crate::{a2, dc42, gcr, mfm};

/// `rotations` entry for a MOOF track whose loop start hasn't been found yet.
const NOT_FOUND: u32 = u32::MAX;

/// Bytes in the MOOF header holding its CRC-32, cleared on the first write.
const MOOF_CRC: core::ops::Range<usize> = 8..12;

/// Card blocks read at a time while converting a flux track (on the stack).
const FLUX_CHUNK: usize = 8;
/// The cell of an A2R capture its revolution is played from: the capture's first cells may come
/// from part of an interval.
const A2R_START: usize = 64;

/// DiskCopy images use at most 37 of `blocks` (36 sectors and one more file block), so the second
/// half stages file blocks for saving and for the checksum.
const STAGE: usize = MAX_TRACK_BLOCKS;
/// After a DiskCopy save: how long the disk must be left alone before the checksum is redone,
/// and how many file blocks are read per step.
const CHECKSUM_DELAY_US: u64 = 2_000_000;
const CHECKSUM_CHUNK: usize = 32;

/// Where a sector image's track is held as cells (`a2::TRACK_CELLS`, MSB first), past its
/// sectors (8 blocks, 9 for a `.2mg` read shifted) and before the staging blocks.
const A2_CELLS: usize = 16;
const A2_CELL_BLOCKS: usize = (a2::TRACK_CELLS / 8).div_ceil(BLOCK);
/// Cells past the end of the track that sectors are decoded from, so one across the wrap is found.
const A2_DECODE_EXTRA: usize = 4_000;

pub struct Cylinder<'a, D> {
    card: &'a D,
    map: &'a BlockMap,
    disk: &'a Disk,
    blocks: &'a mut [Block; MAX_CYLINDER_BLOCKS],
    /// Address and words of the noise buffer blank sides play (core 0 keeps changing it).
    noise: (u32, u32),
    /// Per MOOF TRK record: where its loop starts (see `moof::find_rotation`).
    rotations: [u32; moof::TRK_COUNT],
    /// Track whose data is in `blocks`.
    track: Option<u8>,
    /// For each side of a MOOF flux track: cells after conversion (the TRK record holds bytes),
    /// and its fuzzy reversals, in stored cells.
    flux_cells: [u32; 2],
    fuzzy: [WeakList; 2],
    /// Blocks changed since the last flush, bit per index into `blocks`.
    dirty: u128,
    moof_crc_cleared: bool,
    /// DiskCopy: the header's data checksum is being recomputed after a save, and when the save
    /// was (µs).
    checksum: Option<dc42::ChecksumJob>,
    saved_at: u64,
    /// Apple II sector image: the track's cells at [`A2_CELLS`] are built; where the last write
    /// into them went (`writes::splice_a2`) and the ring word after its capture (a continuation
    /// starts there); and whether they've changed since their sectors were last decoded
    /// ([`Self::settle`]).
    a2_cells: bool,
    a2_last: Option<A2Write>,
    a2_ring: u32,
    a2_settle: bool,
    /// A Disk II's tracks (Apple II mode, and dual mode's drive B) are played by `src/disk2.pio`,
    /// which loops exactly a revolution's cells, so they're built without padding, in one word
    /// more than their whole words.
    a2_exact: bool,
    /// The last build's track for core 1: its cells (0 for the noise buffer) and its rotation in
    /// cells (the stored cell at buffer cell 0).
    geometry: (u32, u32),
    /// An A2R's capture as read: the cell its revolution is played from, and the revolution's
    /// cells (0: nothing to play).
    a2r_loop: (usize, usize),
}

/// One finished write capture from core 1 (see `drive::CAP_START` and friends).
pub struct Capture {
    pub start: u32,
    pub words: u32,
    pub pos: u32,
    pub info: u32,
}

/// Captured cells in the ring.
struct RingCells {
    start: usize,
}

impl Cells for RingCells {
    fn bit(&self, i: usize) -> bool {
        drive::RING.word(self.start + i / 32) >> (31 - i % 32) & 1 != 0
    }
}

/// One side of a MOOF track held in `blocks`, as cells; marks blocks dirty as it changes.
struct StoredSide<'b> {
    blocks: &'b mut [Block],
    bits: usize,
    first: usize,
    dirty: &'b mut u128,
}

impl Cells for StoredSide<'_> {
    fn bit(&self, i: usize) -> bool {
        let i = i % self.bits;
        self.blocks[i / (BLOCK * 8)].contents[i / 8 % BLOCK] >> (7 - i % 8) & 1 != 0
    }
}

impl CellTrack for StoredSide<'_> {
    fn len(&self) -> usize {
        self.bits
    }

    fn set(&mut self, i: usize, v: bool) {
        let i = i % self.bits;
        let block = i / (BLOCK * 8);
        let byte = &mut self.blocks[block].contents[i / 8 % BLOCK];
        let mask = 0x80 >> (i % 8);
        if v { *byte |= mask } else { *byte &= !mask }
        *self.dirty |= 1 << (self.first + block);
    }
}

impl<'a, D: BlockDevice> Cylinder<'a, D> {
    pub fn new(
        card: &'a D,
        map: &'a BlockMap,
        disk: &'a Disk,
        blocks: &'a mut [Block; MAX_CYLINDER_BLOCKS],
        noise: (u32, u32),
        a2_exact: bool,
    ) -> Self {
        Self {
            card,
            map,
            disk,
            blocks,
            noise,
            rotations: [NOT_FOUND; moof::TRK_COUNT],
            track: None,
            flux_cells: [0; 2],
            fuzzy: [const { WeakList::new() }; 2],
            dirty: 0,
            moof_crc_cleared: false,
            checksum: None,
            saved_at: 0,
            a2_cells: false,
            a2_last: None,
            a2_ring: 0,
            a2_settle: false,
            a2_exact,
            geometry: (0, 0),
            a2r_loop: (0, 0),
        }
    }

    pub fn track(&self) -> Option<u8> {
        self.track
    }

    /// The last [`build`](Self::build)'s Disk II track: its cells (0 if it's the noise buffer) and
    /// rotation in cells, for `drive::CYL_BITS` and `CYL_ROTC`.
    pub fn a2_geometry(&self) -> (u32, u32) {
        self.geometry
    }

    /// The PIO clock the last build's track plays at: a WOZ's or A2R's own (its cells in a 200 ms
    /// revolution, `woz::track_hz`), else the disk's.
    pub fn a2_hz(&self) -> u32 {
        match self.disk {
            Disk::Woz(w) if self.geometry.0 > 0 => woz::track_hz(self.geometry.0, w.cell_hz()),
            Disk::A2r(_) if self.geometry.0 > 0 => woz::track_hz(self.geometry.0, crate::drive::A2_PIO_HZ),
            disk => disk.cell_hz(),
        }
    }

    /// The tracks' rotations found so far (`moof::find_rotation`, by TRK index), to hand to the
    /// next `Cylinder` of the same image ([`set_rotations`](Self::set_rotations)).
    pub fn rotations(&self) -> &[u32; moof::TRK_COUNT] {
        &self.rotations
    }

    pub fn set_rotations(&mut self, rotations: &[u32; moof::TRK_COUNT]) {
        self.rotations = *rotations;
    }

    pub fn disk(&self) -> &'a Disk {
        self.disk
    }

    /// Something written isn't saved yet (or, on an Apple II track, not yet decoded).
    pub fn is_dirty(&self) -> bool {
        self.dirty != 0 || self.a2_settle
    }

    /// For a WOZ: find every track's rotation now (`moof::find_rotation`, a few ms each), so
    /// stepping to a track for the first time costs only its SD read. A loader that steps and
    /// reads a track a revolution (Hard Hat Mack) needs it playing within ~15 ms. Leaves nothing
    /// loaded.
    pub fn prepare(&mut self) -> Result<(), D::Error> {
        let Disk::Woz(w) = self.disk else { return Ok(()) };
        for key in 0..woz::QUARTERS as u8 {
            let Some(trk) = w.trk(key) else { continue };
            if trk.bits == 0 || self.rotations[key as usize] != NOT_FOUND {
                continue;
            }
            self.read(key)?;
            let bits = if trk.flux { self.flux_cells[0] } else { trk.bits };
            let blocks = &*self.blocks;
            self.rotations[key as usize] = moof::find_rotation(|i| blocks[i / BLOCK].contents[i % BLOCK], bits);
        }
        self.track = None;
        Ok(())
    }

    /// Read cylinder `track` from the card into `blocks`. Flush first: unsaved writes would be
    /// lost.
    pub fn read(&mut self, track: u8) -> Result<(), D::Error> {
        debug_assert!(self.dirty == 0);
        self.track = None;
        self.a2_cells = false;
        self.a2_last = None;
        self.a2_settle = false;
        match self.disk {
            Disk::Gcr { sides, offset, .. } => {
                let n = gcr::sectors_per_track(track) as usize * *sides as usize;
                // A `.2mg`'s data may start past its first block.
                let first = gcr::first_block(track, *sides) + *offset as usize / BLOCK;
                self.read_sectors(first, n, *offset as usize % BLOCK)?;
            }
            Disk::Mfm { format, offset } => {
                let n = 2 * format.sectors() as usize;
                let first = format.first_block(track, 0);
                self.read_sectors(first, n, *offset as usize)?;
            }
            Disk::Moof(m) => {
                for side in 0..2u8 {
                    if let Some((_, trk)) = m.track(track, side) {
                        let (first, n) = (trk.start as usize, trk.blocks as usize);
                        let at = side as usize * MAX_TRACK_BLOCKS;
                        if trk.flux {
                            self.flux_cells[side as usize] = self.read_flux(Some(moof::FluxCells::new(m.mfm)), trk, at, side as usize)?;
                        } else {
                            read_blocks(self.card, self.map, first, &mut self.blocks[at..at + n])?;
                        }
                    }
                }
            }
            Disk::A2 { order: a2::Order::Dos32, .. } if track != a2::BLANK => {
                // Whole blocks: an odd track starts halfway into one (`a2_slot`).
                let (first, n) = self.sector_range(track);
                read_blocks(self.card, self.map, first, &mut self.blocks[..n])?;
            }
            Disk::A2 { offset, .. } | Disk::Nib { offset, .. } if track != a2::BLANK => {
                let (at, bytes) = self.a2_track_bytes(track, *offset);
                self.read_sectors(at / BLOCK, bytes / BLOCK, at % BLOCK)?;
            }
            Disk::Woz(w) => {
                if let Some(trk) = w.trk(track) {
                    if trk.flux {
                        // At the track's own cell, measured as it's read.
                        self.flux_cells[0] = self.read_flux(None, trk, 0, 0)?;
                    } else if w.version == 1 {
                        self.read_sectors(trk.start as usize, woz::V1_BLOCKS, woz::V1_OFFSET)?;
                    } else {
                        read_blocks(self.card, self.map, trk.start as usize, &mut self.blocks[..trk.blocks as usize])?;
                    }
                }
            }
            // A sector or `.nib` image between tracks: nothing to read.
            Disk::A2 { .. } | Disk::Nib { .. } => {}
            Disk::A2r(r) => self.read_a2r(r, track)?,
        }
        self.track = Some(track);
        Ok(())
    }

    /// Where Apple II track `track` of a sector or `.nib` image is in the file, starting `offset`
    /// bytes in: its first byte and its length.
    fn a2_track_bytes(&self, track: u8, offset: u16) -> (usize, usize) {
        let bytes = match self.disk {
            Disk::Nib { .. } => a2::NIB_TRACK_BYTES,
            Disk::A2 { order, .. } => order.encoding().track_bytes(),
            _ => a2::TRACK_BYTES,
        };
        (offset as usize + track as usize * bytes, bytes)
    }

    /// Where a sector image's track starts in `blocks[0]`: 256 for a 13-sector track that starts
    /// halfway into a block (read whole, not shifted), else 0.
    fn a2_base(&self, track: u8) -> usize {
        match self.disk {
            Disk::A2 { order: a2::Order::Dos32, offset, .. } => self.a2_track_bytes(track, *offset).0 % BLOCK,
            _ => 0,
        }
    }

    /// Read image sectors `first..first + n` into `blocks`. With an `offset` (DiskCopy), sector `i`
    /// starts `offset` bytes into file block `first + i`, so read one more block and shift.
    fn read_sectors(&mut self, first: usize, n: usize, offset: usize) -> Result<(), D::Error> {
        let extra = (offset != 0) as usize;
        read_blocks(self.card, self.map, first, &mut self.blocks[..n + extra])?;
        if offset != 0 {
            for i in 0..n {
                let (this, rest) = self.blocks.split_at_mut(i + 1);
                dc42::shift_sector(&mut this[i].contents, &rest[0].contents, offset);
            }
        }
        Ok(())
    }

    /// Read flux track `trk` a few blocks at a time and convert it into cells at `blocks[at..]`
    /// (see [`read_flux_bytes`](Self::read_flux_bytes)).
    fn read_flux(&mut self, conv: Option<moof::FluxCells>, trk: moof::Trk, at: usize, side: usize) -> Result<u32, D::Error> {
        let (cells, _, _) = self.read_flux_bytes(conv, trk.start as usize * BLOCK, trk.bits as usize, usize::MAX, 3_200, 0, at, side)?;
        Ok(cells)
    }

    /// An A2R's capture of quarter track `q`: converted from its start to a little over a
    /// revolution, its revolution found (`a2r::find_loop`, from [`A2R_START`], near the cells at
    /// its loop point; a solved track is one), and cut where `a2r::splice_point` says.
    fn read_a2r(&mut self, r: &A2r, q: u8) -> Result<(), D::Error> {
        self.a2r_loop = (0, 0);
        let Some(cap) = r.capture(q) else { return Ok(()) };
        let nominal = r.nominal_cell();
        let ticks = if cap.loop_ticks > 0 { cap.loop_ticks } else { r.revolution_ticks() };
        // Enough to find the loop (`a2r::LOOP_SLACK` cells of its estimate) and match the splice,
        // with a margin for the estimate over the nominal cell.
        let rough = (u64::from(ticks) * 100 / u64::from(nominal)) as usize;
        // All of a capture past the revolution leaves room for the splice (`a2r::splice_point`).
        let stop = if cap.solved { usize::MAX } else { usize::MAX.min(rough * 2) };
        // The cells at the loop point, if the capture has one.
        let watch = u64::from(cap.loop_ticks);
        let (n, cell, at_loop) = self.read_flux_bytes(None, cap.offset as usize, cap.len as usize, stop, nominal, watch, 0, 0)?;
        let n = n as usize;
        let blocks = &*self.blocks;
        let byte = |i: usize| blocks.get(i / BLOCK).map_or(0, |b| b.contents[i % BLOCK]);
        self.a2r_loop = if cap.solved {
            (0, n)
        } else {
            // The cells the loop point's time came to; else that time over the measured cell.
            let estimate = if at_loop > 0 { at_loop } else { (u64::from(ticks) * 100 / u64::from(cell.max(1))) as usize };
            let p = a2r::find_loop(&byte, n, A2R_START, estimate, at_loop > 0);
            // A capture of less than a revolution: what there is.
            let p = p.min(n.saturating_sub(A2R_START));
            a2r::splice_point(&byte, n, A2R_START, p)
        };
        Ok(())
    }

    /// Read `len` bytes of flux timings from file byte `from` a few blocks at a time and convert
    /// them into cells at `blocks[at..]`, up to one side's room (`MAX_TRACK_BLOCKS`, which is
    /// also a track buffer) or `stop` cells, noting the fuzzy reversals in `fuzzy[side]`. `conv`
    /// converts at a MOOF's cell; `None` measures the cell on the first two blocks
    /// (`moof::measure_cell`, near `nominal`, 1/100 tick: a 5.25" disk's). Returns the cells, the
    /// cell (1/100 tick), and the cells at `watch` ticks (`FluxCells::watch`; 0 for none or not
    /// reached); cells past the room are dropped.
    #[allow(clippy::too_many_arguments)]
    fn read_flux_bytes(
        &mut self,
        conv: Option<moof::FluxCells>,
        from: usize,
        len: usize,
        stop: usize,
        nominal: u32,
        watch: u64,
        at: usize,
        side: usize,
    ) -> Result<(u32, u32, usize), D::Error> {
        let room = (MAX_TRACK_BLOCKS * BLOCK * 8).min(MAX_TRACK_WORDS * 32);
        let cells = &mut self.blocks[at..at + MAX_TRACK_BLOCKS];
        for b in cells.iter_mut() {
            b.contents.fill(0);
        }
        let fuzzy = &mut self.fuzzy[side];
        fuzzy.clear();
        let mut conv = conv;
        let mut chunk: [Block; FLUX_CHUNK] = core::array::from_fn(|_| Block::new());
        // The first block's bytes before `from` aren't the track's.
        let (first, mut skip) = (from / BLOCK, from % BLOCK);
        let n = (skip + len).div_ceil(BLOCK);
        let mut left = len;
        for block in (first..first + n).step_by(FLUX_CHUNK) {
            let k = FLUX_CHUNK.min(first + n - block);
            read_blocks(self.card, self.map, block, &mut chunk[..k])?;
            let conv = conv.get_or_insert_with(|| {
                let mut start = [0u8; 2 * BLOCK];
                for (i, b) in chunk[..k.min(2)].iter().enumerate() {
                    start[i * BLOCK..][..BLOCK].copy_from_slice(&b.contents);
                }
                let end = (skip + left).min(k.min(2) * BLOCK);
                let mut c = moof::FluxCells::with_cell(moof::measure_cell(&start[skip..end], nominal));
                c.watch(watch);
                c
            });
            for b in &chunk[..k] {
                let take = left.min(BLOCK - skip);
                conv.feed(&b.contents[skip..skip + take], |i, other| {
                    if i < room {
                        cells[i / (BLOCK * 8)].contents[i / 8 % BLOCK] |= 0x80 >> (i % 8);
                        if other != moof::Other::None {
                            fuzzy.push_fuzzy(i, other == moof::Other::Later);
                        }
                    }
                });
                left -= take;
                skip = 0;
            }
            if conv.cells() >= stop.min(room) {
                break;
            }
        }
        Ok(conv.map_or((0, nominal, 0), |c| (c.cells().min(room) as u32, c.cell(), c.at_watch())))
    }

    /// Build both sides of the loaded cylinder into `bufs`, and list each one's weak bits in
    /// `weak` (MOOF tracks only; see `weak.rs`). Returns each side's (buffer address, words,
    /// rotation in words) for core 1. Raw images are encoded; MOOF tracks are copied, GCR ones
    /// rotated so the loop wraps in a gap. The rotation (`drive::CYL_ROT`) says where word 0 is in
    /// the track as imaged, so core 1 can keep the angle across steps.
    pub fn build(&mut self, bufs: &mut [TrackBuf; 2], weak: &mut [WeakList; 2]) -> [(u32, u32, u32); 2] {
        let track = self.track.expect("cylinder loaded");
        let noise = (self.noise.0, self.noise.1, 0);
        for w in weak.iter_mut() {
            w.clear();
        }
        let mut sides = [(0u32, 0u32, 0u32); 2];
        self.geometry = (0, 0);
        // One word more for `disk2.pio`: the last cells, or a word it takes and drops.
        let extra = self.a2_exact as usize;
        let disk = self.disk;
        match disk {
            Disk::Gcr { sides: count, format, .. } => {
                let spt = gcr::sectors_per_track(track) as usize;
                for side in 0..*count {
                    let data = &self.blocks[side as usize * spt..][..spt];
                    let buf = &mut bufs[side as usize];
                    let words = gcr::build_track(track, side, *format, |s| &data[s as usize].contents, buf);
                    sides[side as usize] = (buf.as_ptr() as u32, words as u32, 0);
                }
                if *count == 1 {
                    // Single-sided: the other head sees side 0 too.
                    sides[1] = sides[0];
                }
            }
            Disk::Mfm { format, .. } => {
                let spt = format.sectors() as usize;
                for side in 0..2u8 {
                    let data = &self.blocks[side as usize * spt..][..spt];
                    let buf = &mut bufs[side as usize];
                    let words = mfm::build_track(*format, track, side, |s| &data[s as usize - 1].contents, buf);
                    sides[side as usize] = (buf.as_ptr() as u32, words as u32, 0);
                }
            }
            Disk::Moof(m) => {
                for side in 0..2u8 {
                    let converted = self.flux_cells[side as usize];
                    sides[side as usize] = match m.track(track, side) {
                        // A flux track with no reversals has nothing to play.
                        Some((_, trk)) if trk.flux && converted == 0 => noise,
                        Some((index, trk)) => {
                            let bits = if trk.flux { converted } else { trk.bits };
                            let blocks = &self.blocks[side as usize * MAX_TRACK_BLOCKS..];
                            let byte = |i: usize| blocks[i / BLOCK].contents[i % BLOCK];
                            // GCR tracks wrap in their longest gap (found once, then cached);
                            // MFM tracks start at the index.
                            let rotation = if m.mfm {
                                0
                            } else {
                                let r = &mut self.rotations[index];
                                if *r == NOT_FOUND {
                                    *r = moof::find_rotation(byte, bits);
                                }
                                *r
                            };
                            let pad = if m.mfm { moof::Pad::Mfm } else { moof::Pad::Gcr };
                            let buf = &mut bufs[side as usize];
                            let words = moof::load_track(byte, bits, rotation, pad, buf);
                            let w = &mut weak[side as usize];
                            w.find_runs(&buf[..], bits as usize, if m.mfm { weak::MFM_RUN } else { weak::GCR_RUN });
                            if trk.flux {
                                w.add_fuzzy(&self.fuzzy[side as usize], bits as usize, rotation as usize, &buf[..]);
                            }
                            (buf.as_ptr() as u32, words as u32, rotation / 32)
                        }
                        // Single-sided image: side 1 mirrors side 0, as for a raw 400K image.
                        None if side == 1 && sides[0] != noise => sides[0],
                        None => noise,
                    };
                }
            }
            // Apple II: one side, and nothing between tracks. Core 1 only plays side 0.
            Disk::A2 { .. } | Disk::Nib { .. } if track == a2::BLANK => sides = [noise; 2],
            Disk::A2 { order, volume, .. } => {
                const { assert!(A2_CELLS >= 9 && A2_CELLS + A2_CELL_BLOCKS <= STAGE) };
                let buf = &mut bufs[0];
                let words = a2::TRACK_CELLS / 32;
                let base = self.a2_base(track);
                let (sectors, cells) = self.blocks.split_at_mut(A2_CELLS);
                let cells = &mut cells[..A2_CELL_BLOCKS];
                if self.a2_cells {
                    // As written to.
                    for (i, w) in buf[..words].iter_mut().enumerate() {
                        *w = u32::from_be_bytes(cells[i / 128].contents[i % 128 * 4..][..4].try_into().unwrap());
                    }
                } else {
                    a2::build_track(
                        order.encoding(),
                        track,
                        *volume,
                        |p| {
                            let (b, at) = a2_slot(base, a2::file_sector(*order, p));
                            sectors[b].contents[at..][..a2::SECTOR_BYTES].try_into().unwrap()
                        },
                        buf,
                    );
                    for (i, w) in buf[..words].iter().enumerate() {
                        cells[i / 128].contents[i % 128 * 4..][..4].copy_from_slice(&w.to_be_bytes());
                    }
                    self.a2_cells = true;
                }
                buf[words] = 0;
                self.geometry = (a2::TRACK_CELLS as u32, 0);
                sides = [(buf.as_ptr() as u32, (words + extra) as u32, 0); 2];
            }
            Disk::Nib { .. } => {
                let blocks = &*self.blocks;
                let buf = &mut bufs[0];
                let words = a2::build_nib_track(|i| blocks[i / BLOCK].contents[i % BLOCK], buf);
                buf[words] = 0;
                self.geometry = (words as u32 * 32, 0);
                sides = [(buf.as_ptr() as u32, (words + extra) as u32, 0); 2];
            }
            Disk::Woz(w) => {
                sides[0] = match w.trk(track) {
                    None => noise,
                    // A flux track with no reversals has nothing to play.
                    Some(trk) if trk.flux && self.flux_cells[0] == 0 => noise,
                    Some(trk) => {
                        let bits = if trk.flux { self.flux_cells[0] } else { trk.bits };
                        let blocks = &*self.blocks;
                        let byte = |i: usize| blocks[i / BLOCK].contents[i % BLOCK];
                        // Wrap in the longest gap, as for a GCR MOOF (the marks are the same).
                        let r = &mut self.rotations[track as usize];
                        if *r == NOT_FOUND {
                            *r = moof::find_rotation(byte, bits);
                        }
                        let rotation = *r;
                        let buf = &mut bufs[0];
                        let pad = if self.a2_exact { moof::Pad::None } else { moof::Pad::Gcr };
                        let words = moof::load_track(byte, bits, rotation, pad, buf);
                        weak[0].find_runs(&buf[..], bits as usize, weak::A2_RUN);
                        if trk.flux {
                            weak[0].add_fuzzy(&self.fuzzy[0], bits as usize, rotation as usize, &buf[..]);
                        }
                        self.geometry = (bits, rotation);
                        (buf.as_ptr() as u32, words as u32, rotation / 32)
                    }
                };
                sides[1] = sides[0];
            }
            Disk::A2r(_) => {
                let (start, bits) = self.a2r_loop;
                sides[0] = if bits == 0 {
                    noise
                } else {
                    // The revolution from `start`, as it was imaged (a synchronized capture starts at
                    // the index): that's its rotation.
                    let blocks = &*self.blocks;
                    let cell = |i: usize| blocks[i / (BLOCK * 8)].contents[i / 8 % BLOCK] >> (7 - i % 8) & 1;
                    let byte = |i: usize| (0..8).fold(0u8, |b, k| b << 1 | cell(start + 8 * i + k));
                    let buf = &mut bufs[0];
                    let words = moof::load_track(byte, bits as u32, 0, moof::Pad::None, buf);
                    weak[0].find_runs(&buf[..], bits, weak::A2_RUN);
                    weak[0].add_fuzzy(&self.fuzzy[0], bits, start, &buf[..]);
                    self.geometry = (bits as u32, start as u32);
                    (buf.as_ptr() as u32, words as u32, (start / 32) as u32)
                };
                sides[1] = sides[0];
            }
        }
        sides
    }

    /// Apply one write capture to the loaded cylinder. `scratch` is a free track buffer, used
    /// when a MOOF track is reformatted. Returns how many sectors (or splices) it changed.
    pub fn apply(&mut self, cap: &Capture, scratch: &mut TrackBuf) -> Result<u32, &'static str> {
        let track = drive::cylinder_track(cap.info & 0x1_FFFF);
        // Apple II captures name the quarter track; the cylinder holds what's heard there.
        let track = if self.disk.is_apple2() { track.map(|q| self.disk.a2_track(q)) } else { track };
        if track.is_none() || track != self.track {
            return Err("write: not the loaded cylinder");
        }
        if !self.disk.writable() {
            return Err("write: read-only image"); // R3 says so; no gate should have opened
        }
        let track = self.track.unwrap_or(0);
        let side = (cap.info & CAP_SIDE != 0) as u8;
        let format = cap.info & CAP_FORMAT != 0;
        let cells = RingCells { start: cap.start as usize };
        let len = cap.words as usize * 32;
        let mut changed = 0;
        let disk = self.disk;
        match disk {
            Disk::A2r(_) => return Err("write: read-only image"),
            Disk::Gcr { sides, .. } => {
                if side >= *sides {
                    return Err("write: side 1 of a 400K disk");
                }
                let spt = gcr::sectors_per_track(track);
                let (blocks, dirty) = (&mut *self.blocks, &mut self.dirty);
                writes::gcr_data_fields(&cells, len, |sector, data| {
                    if sector < spt {
                        let k = side as usize * spt as usize + sector as usize;
                        blocks[k].contents.copy_from_slice(&data[12..]);
                        *dirty |= 1 << k;
                        changed += 1;
                    }
                });
            }
            Disk::Mfm { format, .. } => {
                let spt = format.sectors();
                let (blocks, dirty) = (&mut *self.blocks, &mut self.dirty);
                let mut id_sector = None;
                writes::mfm_fields(&cells, len, |field, _| match field {
                    MfmField::Id([c, h, r, _]) => id_sector = (c == track && h == side).then_some(r),
                    MfmField::Data(data) => {
                        // A format carries IDs; a sector write is for the ID just read.
                        let sector = id_sector.take().unwrap_or_else(|| mfm::sector_at(*format, cap.pos as usize));
                        if (1..=spt).contains(&sector) {
                            let k = side as usize * spt as usize + sector as usize - 1;
                            blocks[k].contents.copy_from_slice(data);
                            *dirty |= 1 << k;
                            changed += 1;
                        }
                    }
                });
            }
            Disk::Moof(m) => {
                let (index, trk) = m.track(track, side).ok_or("write: blank MOOF track")?;
                let first = side as usize * MAX_TRACK_BLOCKS;
                let mut stored = StoredSide {
                    blocks: &mut self.blocks[first..first + trk.blocks as usize],
                    bits: trk.bits as usize,
                    first,
                    dirty: &mut self.dirty,
                };
                if format {
                    // The Mac's layout needn't fit this track's fixed length: keep its sectors,
                    // re-encode them to fit.
                    reformat(m, track, side, &cells, len, &mut stored, scratch)?;
                    self.rotations[index] = NOT_FOUND;
                } else {
                    // `cap.pos` is a cell of the rotated loop; the stored track isn't rotated.
                    let rotation = if m.mfm { 0 } else { self.rotations[index] };
                    let rotation = if rotation == NOT_FOUND { 0 } else { rotation as usize };
                    let pos = (cap.pos as usize + rotation) % stored.bits;
                    if m.mfm {
                        writes::splice_mfm(&mut stored, pos, &cells, len)?;
                    } else {
                        writes::splice_gcr(&mut stored, pos, &cells, len)?;
                    }
                }
                changed = 1;
            }
            Disk::A2 { .. } if track == a2::BLANK => return Err("write: between tracks"),
            Disk::A2 { .. } => {
                if !self.a2_cells {
                    return Err("write: track not built");
                }
                // Into the track as it plays (rotation 0: `cap.pos` is a cell of it). Its sectors
                // are decoded once the computer stops writing (`settle`): DOS 3.3's formatter
                // writes 32 times a revolution, faster than they could be after each.
                let rest = cap.info & CAP_REST != 0 && cap.start == self.a2_ring;
                self.a2_ring = (cap.start + cap.words) % drive::RING_WORDS as u32;
                let mut unused = 0;
                let mut cells_held = StoredSide {
                    blocks: &mut self.blocks[A2_CELLS..A2_CELLS + A2_CELL_BLOCKS],
                    bits: a2::TRACK_CELLS,
                    first: 0,
                    dirty: &mut unused,
                };
                let last = self.a2_last.take();
                self.a2_last = Some(writes::splice_a2(&mut cells_held, cap.pos as usize, last, &cells, len, format, rest)?);
                self.a2_settle = true;
                changed = 1;
            }
            Disk::Nib { .. } => return Err("write: .nib images are read-only"),
            Disk::Woz(w) => {
                let trk = w.trk(track).ok_or("write: blank WOZ track")?;
                let mut stored = StoredSide { blocks: &mut self.blocks[..trk.blocks as usize], bits: trk.bits as usize, first: 0, dirty: &mut self.dirty };
                if format {
                    a2_reformat(&cells, len, &mut stored, scratch)?;
                    self.rotations[track as usize] = NOT_FOUND;
                } else {
                    let rotation = self.rotations[track as usize];
                    let rotation = if rotation == NOT_FOUND { 0 } else { rotation as usize };
                    let pos = (cap.pos as usize + rotation) % stored.bits;
                    writes::splice_gcr(&mut stored, pos, &cells, len)?;
                }
                changed = 1;
            }
        }
        Ok(changed)
    }

    /// An Apple II sector image's track after writes: every sector found in its cells that
    /// differs from the image's is taken, to be saved. Returns how many.
    pub fn settle(&mut self) -> u32 {
        let (Disk::A2 { order, .. }, Some(track), true) = (self.disk, self.track, self.a2_settle) else {
            return 0;
        };
        self.a2_settle = false;
        let (enc, base) = (order.encoding(), self.a2_base(track));
        let (sectors, held) = self.blocks.split_at_mut(A2_CELLS);
        let mut unused = 0;
        let cells_held = StoredSide { blocks: &mut held[..A2_CELL_BLOCKS], bits: a2::TRACK_CELLS, first: 0, dirty: &mut unused };
        let (dirty, mut changed) = (&mut self.dirty, 0);
        // Only the image's own kind of sector fits it (DOS 3.3 formatting a 13-sector image's
        // track plays until it's next read, but can't be saved).
        writes::a2_fields(&cells_held, a2::TRACK_CELLS + A2_DECODE_EXTRA, enc, |address, kind, data| {
            if let Some((t, s)) = address
                && t == track
                && kind == enc
                && s < enc.sectors()
            {
                let (b, at) = a2_slot(base, a2::file_sector(*order, s));
                let sector = &mut sectors[b].contents[at..][..a2::SECTOR_BYTES];
                if sector != data {
                    sector.copy_from_slice(data);
                    *dirty |= 1 << b;
                    changed += 1;
                }
            }
        });
        changed
    }

    /// Write changed blocks back to the card, one multi-block write per run of consecutive card
    /// sectors. An Apple II track's sectors are decoded first ([`Self::settle`]).
    pub fn flush(&mut self, now_us: u64) -> Result<(), D::Error> {
        self.settle();
        if self.dirty == 0 {
            return Ok(());
        }
        let track = self.track.unwrap_or(0);
        if self.disk.diskcopy() {
            self.flush_shifted(track, dc42::HEADER_BYTES)?;
            // The header's data checksum is now out of date; redo it once things are quiet.
            self.checksum = Some(dc42::ChecksumJob::new(self.data_size()));
            self.saved_at = now_us;
            return Ok(());
        }
        if let Disk::A2 { offset, .. } | Disk::Gcr { offset, .. } = *self.disk
            && offset as usize % BLOCK != 0
        {
            // A `.2mg`'s sectors start partway into a block.
            return self.flush_shifted(track, offset as usize % BLOCK);
        }
        if let Disk::Moof(_) | Disk::Woz(_) = self.disk
            && !self.moof_crc_cleared
        {
            // The header CRC would no longer match; 0 means "not computed".
            let mut header = [Block::new()];
            let (lba, _) = self.map.sectors(0);
            self.card.read(&mut header, BlockIdx(lba))?;
            header[0].contents[MOOF_CRC].fill(0);
            self.card.write(&header, BlockIdx(lba))?;
            self.moof_crc_cleared = true;
        }
        let mut k = 0;
        while k < MAX_CYLINDER_BLOCKS {
            if self.dirty & 1 << k == 0 {
                k += 1;
                continue;
            }
            let (lba, _) = self.map.sectors(self.file_block(track, k));
            let mut n = 1;
            while k + n < MAX_CYLINDER_BLOCKS
                && self.dirty & 1 << (k + n) != 0
                && self.map.sectors(self.file_block(track, k + n)).0 == lba + n as u32
            {
                n += 1;
            }
            self.card.write(&self.blocks[k..k + n], BlockIdx(lba))?;
            for i in k..k + n {
                self.dirty &= !(1 << i);
            }
            k += n;
        }
        Ok(())
    }

    /// [`flush`](Self::flush) for a DiskCopy image, whose sector `i` starts `offset` bytes into
    /// file block `i`. Each run of changed sectors spans one more file block than it has sectors;
    /// the first and last of those also hold bytes that aren't the run's (the header, neighbouring
    /// sectors, tags), so they start from the card. The run is staged in `blocks[STAGE..]`.
    fn flush_shifted(&mut self, track: u8, offset: usize) -> Result<(), D::Error> {
        let (first, n) = self.sector_range(track);
        let mut k = 0;
        while k < n {
            if self.dirty & 1 << k == 0 {
                k += 1;
                continue;
            }
            let mut end = k;
            while end < n && self.dirty & 1 << end != 0 {
                end += 1;
            }
            let count = end - k + 1;
            let (sectors, stage) = self.blocks.split_at_mut(STAGE);
            read_blocks(self.card, self.map, first + k, &mut stage[..1])?;
            read_blocks(self.card, self.map, first + end, &mut stage[count - 1..count])?;
            for j in k..end {
                let (this, next) = stage[j - k..].split_at_mut(1);
                dc42::place_sector(&mut this[0].contents, &mut next[0].contents, &sectors[j].contents, offset);
            }
            write_blocks(self.card, self.map, first + k, &stage[..count])?;
            for j in k..end {
                self.dirty &= !(1 << j);
            }
            k = end;
        }
        Ok(())
    }

    /// The image sectors of cylinder `track`: the first one and how many (both sides).
    fn sector_range(&self, track: u8) -> (usize, usize) {
        match self.disk {
            Disk::Gcr { sides, offset, .. } => {
                let first = gcr::first_block(track, *sides) + *offset as usize / BLOCK;
                (first, gcr::sectors_per_track(track) as usize * *sides as usize)
            }
            Disk::Mfm { format, .. } => (format.first_block(track, 0), 2 * format.sectors() as usize),
            Disk::A2 { order: a2::Order::Dos32, offset, .. } => {
                let (at, bytes) = self.a2_track_bytes(track, *offset);
                (at / BLOCK, (at % BLOCK + bytes).div_ceil(BLOCK))
            }
            Disk::A2 { offset, .. } | Disk::Nib { offset, .. } => {
                let (at, bytes) = self.a2_track_bytes(track, *offset);
                (at / BLOCK, bytes / BLOCK)
            }
            Disk::Moof(_) | Disk::Woz(_) | Disk::A2r(_) => (0, 0),
        }
    }

    /// Bytes of sector data in a sector image.
    fn data_size(&self) -> usize {
        let (first, n) = self.sector_range(crate::track::TRACKS - 1);
        (first + n) * BLOCK
    }

    /// A DiskCopy checksum still has to be written after a save.
    pub fn checksum_pending(&self) -> bool {
        self.checksum.is_some()
    }

    /// After a DiskCopy save: recompute the data checksum from the card, `CHECKSUM_CHUNK` file
    /// blocks per call, then write it into the header. Waits until `CHECKSUM_DELAY_US` after the
    /// last save, unless `urgent` (another image is about to be loaded). Returns whether it did
    /// anything. Call only with nothing dirty; a later save starts it over.
    pub fn checksum_step(&mut self, now_us: u64, urgent: bool) -> Result<bool, D::Error> {
        let total = (dc42::HEADER_BYTES + self.data_size()).div_ceil(BLOCK);
        let Some(job) = self.checksum.as_mut() else { return Ok(false) };
        if !urgent && now_us.wrapping_sub(self.saved_at) < CHECKSUM_DELAY_US {
            return Ok(false);
        }
        if let Some(next) = job.next_block() {
            let n = CHECKSUM_CHUNK.min(total - next);
            let stage = &mut self.blocks[STAGE..STAGE + n];
            read_blocks(self.card, self.map, next, stage)?;
            for b in stage.iter() {
                job.feed(&b.contents);
            }
        }
        if job.next_block().is_none() {
            let sum = job.finish();
            let (lba, _) = self.map.sectors(0);
            let mut header = [Block::new()];
            self.card.read(&mut header, BlockIdx(lba))?;
            header[0].contents[dc42::DATA_CHECKSUM].copy_from_slice(&sum.to_be_bytes());
            self.card.write(&header, BlockIdx(lba))?;
            self.checksum = None;
        }
        Ok(true)
    }

    /// File block behind `blocks[k]`.
    fn file_block(&self, track: u8, k: usize) -> usize {
        match self.disk {
            Disk::Gcr { .. } => self.sector_range(track).0 + k,
            Disk::Mfm { format, .. } => format.first_block(track, 0) + k,
            Disk::Moof(m) => {
                let side = (k / MAX_TRACK_BLOCKS) as u8;
                let (_, trk) = m.track(track, side).unwrap_or_default();
                trk.start as usize + k % MAX_TRACK_BLOCKS
            }
            Disk::A2 { .. } | Disk::Nib { .. } => self.sector_range(track).0 + k,
            Disk::Woz(w) => w.trk(track).unwrap_or_default().start as usize + k,
            Disk::A2r(_) => 0, // read-only: nothing is saved
        }
    }
}

/// Decode the sectors of a format capture and re-encode them into `stored`'s fixed length.
fn reformat(m: &Moof, track: u8, side: u8, cells: &RingCells, len: usize, stored: &mut StoredSide, scratch: &mut TrackBuf) -> Result<(), &'static str> {
    // MFM MOOFs are always 1.44 MB: the format has no 720K disk type.
    let mut sectors = [[0u8; BLOCK]; mfm::MAX_SECTORS as usize];
    if m.mfm {
        let mut id = None;
        writes::mfm_fields(cells, len, |field, _| match field {
            MfmField::Id([_, _, r, _]) => id = Some(r),
            MfmField::Data(d) => {
                if let Some(r @ 1..=mfm::MAX_SECTORS) = id.take() {
                    sectors[r as usize - 1] = *d;
                }
            }
        });
        mfm::build_track_cells(mfm::Format::Hd, track, side, |s| &sectors[s as usize - 1], stored.bits, scratch)
            .ok_or("write: format doesn't fit the MOOF track")?;
    } else {
        writes::gcr_data_fields(cells, len, |s, d| {
            if let Some(slot) = sectors.get_mut(s as usize) {
                slot.copy_from_slice(&d[12..]);
            }
        });
        // A WOZ is an Apple II disk: its 4:1 interleave.
        gcr::build_track_cells(track, side, gcr::format(m.sides, m.woz), |s| &sectors[s as usize], stored.bits, scratch)
            .ok_or("write: format doesn't fit the MOOF track")?;
    }
    writes::copy_track(stored, scratch);
    Ok(())
}

/// Decode the sectors of an Apple II format capture and re-encode them into `stored`'s fixed
/// length (a WOZ track's), on the track its address fields name, 16 or 13 sectors as they are.
fn a2_reformat(cells: &RingCells, len: usize, stored: &mut StoredSide, scratch: &mut TrackBuf) -> Result<(), &'static str> {
    let mut sectors = [[0u8; a2::SECTOR_BYTES]; a2::SECTORS as usize];
    // The track and encoding (16 or 13 sectors) of the first address field.
    let mut first = None;
    writes::a2_fields(cells, len, a2::Encoding::SixAndTwo, |address, enc, d| {
        if let Some((t, s)) = address
            && s < enc.sectors()
            && *first.get_or_insert((t, enc)) == (t, enc)
        {
            sectors[s as usize] = *d;
        }
    });
    let (track, enc) = first.ok_or("write: format with no sectors")?;
    a2::build_track_cells(enc, track, a2::VOLUME, |s| &sectors[s as usize], stored.bits, scratch)
        .ok_or("write: format doesn't fit the WOZ track")?;
    writes::copy_track(stored, scratch);
    Ok(())
}

/// Where sector slot `f` of a sector image's track is in `blocks`, the track starting `base` bytes
/// into the first (0, or 256 for a 13-sector track: a sector never straddles two blocks): the
/// block and the byte in it.
fn a2_slot(base: usize, f: usize) -> (usize, usize) {
    let at = base + f * a2::SECTOR_BYTES;
    (at / BLOCK, at % BLOCK)
}

/// Write `blocks` to file blocks `first..first + blocks.len()`, one multi-block write per run of
/// consecutive card sectors.
pub fn write_blocks<D: BlockDevice, const N: usize>(card: &D, map: &BlockMap<N>, first: usize, blocks: &[Block]) -> Result<(), D::Error> {
    let mut i = 0;
    while i < blocks.len() {
        let (lba, run) = map.sectors(first + i);
        let n = run.min(blocks.len() - i);
        card.write(&blocks[i..i + n], BlockIdx(lba))?;
        i += n;
    }
    Ok(())
}

/// Read file blocks `first..first + out.len()` into `out`, one multi-block read per run of
/// consecutive card sectors.
pub fn read_blocks<D: BlockDevice, const N: usize>(card: &D, map: &BlockMap<N>, first: usize, out: &mut [Block]) -> Result<(), D::Error> {
    let mut i = 0;
    while i < out.len() {
        let (lba, run) = map.sectors(first + i);
        let n = run.min(out.len() - i);
        card.read(&mut out[i..i + n], BlockIdx(lba))?;
        i += n;
    }
    Ok(())
}
