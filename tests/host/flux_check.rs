//! Host-side check of MOOF FLUX tracks (`src/moof.rs`: the FLUX map in `Moof::parse`, and
//! `FluxCells`). Every odd track of a 400K GCR MOOF (Macintosh Pascal's traits) and a 1.44 MB MFM
//! one is turned into flux timings, the way Applesauce would have recorded it with a drive
//! running a little off speed and some jitter, and listed in a FLUX map. Each flux track is then
//! converted back block by block, as `Cylinder::read_flux` does, and must give the original cells
//! exactly (rotated to start after the last reversal, since flux has no index).
//!
//! Flux like real disks' too, made at test time (`disks.rs`) after two real ones. Oids (one flux
//! track with fuzzy reversals in one sector, its FLUX chunk in the file's partial last block).
//! LabelWriter II: an A2R 3 capture of a disk and the MOOF Fast Imager made of it; each track
//! side's capture (1.25 revolutions) goes through `FluxCells`, and every sector decoded from it
//! must match the MOOF.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/flux_check.rs -o target/flux_check && target/flux_check

mod disks;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/moof.rs"]
mod moof;
#[path = "../../src/track.rs"]
mod track;
#[path = "../../src/weak.rs"]
mod weak;

use track::{BLOCK, MAX_TRACK_BLOCKS, MAX_TRACK_WORDS, TRACKS};

fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// Flux bytes for a tick count: 255 means "add and keep reading".
fn push_ticks(out: &mut Vec<u8>, mut t: u64) {
    while t >= 255 {
        out.push(255);
        t -= 255;
    }
    out.push(t as u8);
}

/// One revolution of `bits` cells as flux bytes. Each reversal is placed at its cell's time on a
/// drive running `speed` times slow, plus up to ±`jitter` ticks of noise; the first interval runs
/// from the last reversal of the previous turn.
fn to_flux(bits: &[bool], cell_ticks: f64, speed: f64, jitter: u64, seed: &mut u64) -> Vec<u8> {
    let n = bits.len();
    let at = |cell: usize, s: &mut u64| {
        let noise = (rng(s) % (2 * jitter + 1)) as f64 - jitter as f64;
        ((cell + 1) as f64 * cell_ticks * speed + noise).round() as u64
    };
    let ones: Vec<usize> = (0..n).filter(|&i| bits[i]).collect();
    let total = (n as f64 * cell_ticks * speed).round() as u64;
    let times: Vec<u64> = ones.iter().map(|&i| at(i, seed)).collect();
    let mut out = vec![];
    let mut prev = *times.last().unwrap() as i64 - total as i64;
    for &t in &times {
        push_ticks(&mut out, (t as i64 - prev) as u64);
        prev = t as i64;
    }
    out
}

/// Convert a flux track the way `Cylinder::read_flux` does: block by block, up to the byte count.
/// Returns the cells and the fuzzy reversals (cell, which way the other cell is).
fn from_flux(file: &[u8], trk: &moof::Trk, mfm: bool) -> (Vec<bool>, Vec<(usize, moof::Other)>) {
    let room = (MAX_TRACK_BLOCKS * BLOCK * 8).min(MAX_TRACK_WORDS * 32);
    let mut cells = vec![false; room];
    let mut fuzzy = vec![];
    let mut conv = moof::FluxCells::new(mfm);
    let mut left = trk.bits as usize;
    for b in 0..trk.blocks as usize {
        let block = &file[(trk.start as usize + b) * BLOCK..][..BLOCK];
        let take = left.min(BLOCK);
        conv.feed(&block[..take], |i, other| {
            if i < room {
                cells[i] = true;
                if other != moof::Other::None {
                    fuzzy.push((i, other));
                }
            }
        });
        left -= take;
    }
    cells.truncate(conv.cells().min(room));
    (cells, fuzzy)
}

/// A copy of MOOF `file` with the mapped sides of every odd track turned into flux and moved to
/// a FLUX map at block 3. Returns the new file and how many sides became flux.
fn with_flux(file: &[u8], mfm: bool, speed: f64, jitter: u64, seed: &mut u64) -> (Vec<u8>, usize) {
    let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    let cell_ticks = if mfm { 8.0 } else { 16.34 };
    let mut out = file[..moof::HEADER_BYTES].to_vec();
    out.resize(4 * BLOCK, 0);
    out[3 * BLOCK..][..4].copy_from_slice(b"FLUX");
    out[3 * BLOCK + 4..][..4].copy_from_slice(&160u32.to_le_bytes());
    out[3 * BLOCK + 8..][..160].fill(0xFF);
    out[88..88 + 160].fill(0xFF);
    out[256..1536].fill(0);
    let (mut flux_sides, mut largest_flux, mut largest) = (0, 0u16, 0u16);
    for k in 0..160usize {
        let Some((_, trk)) = m.track(k as u8 / 2, k as u8 % 2) else { continue };
        let data = &file[trk.start as usize * BLOCK..][..trk.blocks as usize * BLOCK];
        let flux = k / 2 % 2 == 1;
        let (bytes, count) = if flux {
            let bits: Vec<bool> = (0..trk.bits as usize).map(|i| data[i / 8] >> (7 - i % 8) & 1 != 0).collect();
            let f = to_flux(&bits, cell_ticks, speed, jitter, seed);
            let len = f.len() as u32;
            (f, len)
        } else {
            (data.to_vec(), trk.bits)
        };
        let start = out.len() / BLOCK;
        let blocks = bytes.len().div_ceil(BLOCK) as u16;
        out.extend(&bytes);
        out.resize((start + blocks as usize) * BLOCK, 0);
        // One TRK record per side, at index k.
        let r = 256 + k * 8;
        out[r..r + 2].copy_from_slice(&(start as u16).to_le_bytes());
        out[r + 2..r + 4].copy_from_slice(&blocks.to_le_bytes());
        out[r + 4..r + 8].copy_from_slice(&count.to_le_bytes());
        if flux {
            out[3 * BLOCK + 8 + k] = k as u8;
            flux_sides += 1;
            largest_flux = largest_flux.max(blocks);
        } else {
            out[88 + k] = k as u8;
            largest = largest.max(blocks);
        }
    }
    out[58..60].copy_from_slice(&largest.to_le_bytes());
    out[60..62].copy_from_slice(&3u16.to_le_bytes());
    out[62..64].copy_from_slice(&largest_flux.to_le_bytes());
    out[8..12].fill(0); // CRC not computed
    (out, flux_sides)
}

fn check_converter() {
    let cells = |mfm: bool, bytes: &[u8]| {
        let mut conv = moof::FluxCells::new(mfm);
        let mut ones = vec![];
        conv.feed(bytes, |i, _| ones.push(i));
        (ones, conv.cells())
    };
    let fuzzy = |bytes: &[u8]| {
        let mut conv = moof::FluxCells::new(false);
        let mut out = vec![];
        conv.feed(bytes, |i, other| out.push((i, other)));
        out
    };
    // GCR: 2, 4 and 6 µs reversals (16, 33, 49 ticks) are 1, 2 and 3 cells.
    assert_eq!(cells(false, &[16, 33, 49]), (vec![0, 2, 5], 6));
    // 255, 255, 10 = 520 ticks = 31.8 cells.
    assert_eq!(cells(false, &[255, 255, 10]), (vec![31], 32));
    // 255 then 0 is exactly 255 ticks.
    assert_eq!(cells(false, &[255, 0]), (vec![15], 16));
    // Under half a cell is noise, merged into the next interval.
    assert_eq!(cells(false, &[3, 14]), (vec![0], 1));
    // MFM: 2, 3 and 4 µs reversals are 2, 3 and 4 half-cells.
    assert_eq!(cells(true, &[16, 24, 32]), (vec![1, 4, 8], 9));
    // A reversal on the IWM's boundary: 1.53 cells after the last (or 2.45), then 2.51 back to
    // the grid. It's fuzzy; the next one, measured from its true place, isn't, and lands on the
    // grid (rounding 2.51 alone would put it, and everything after, one cell late). 1.41 cells
    // (23 ticks) is clear of the boundary.
    use moof::Other::*;
    assert_eq!(fuzzy(&[16, 25, 41, 16]), vec![(0, None), (2, Earlier), (4, None), (5, None)]);
    assert_eq!(fuzzy(&[16, 40, 41]), vec![(0, None), (2, Later), (5, None)]);
    assert_eq!(fuzzy(&[16, 23, 33]), vec![(0, None), (1, None), (3, None)]);
    println!("FluxCells: GCR and MFM intervals, 255 runs, noise merged, fuzzy reversals");
}

fn check_image(name: &str, file: &[u8], mfm: bool, speed: f64, jitter: u64) {
    let orig = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    let mut seed = 0x5EED_F1A5u64;
    let (flux, flux_sides) = with_flux(file, mfm, speed, jitter, &mut seed);
    let header: &[u8; moof::HEADER_BYTES] = flux[..moof::HEADER_BYTES].try_into().unwrap();
    assert_eq!(moof::Moof::flux_block(header), Some(3));
    let chunk: &[u8; BLOCK] = flux[3 * BLOCK..4 * BLOCK].try_into().unwrap();
    let blocks = flux.len() / BLOCK;
    assert!(moof::Moof::parse(header, None, blocks, MAX_TRACK_BLOCKS).is_err(), "FLUX named but not given");
    let mut bad = *chunk;
    bad[0] = b'X';
    assert!(moof::Moof::parse(header, Some(&bad), blocks, MAX_TRACK_BLOCKS).is_err(), "bad FLUX chunk ID");
    let m = moof::Moof::parse(header, Some(chunk), blocks, MAX_TRACK_BLOCKS).unwrap();
    assert!(m.has_flux);

    let (mut largest, mut flagged) = (0, 0);
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let Some((_, o)) = orig.track(track, side) else {
                assert!(m.track(track, side).is_none());
                continue;
            };
            let (_, trk) = m.track(track, side).expect("mapped");
            assert_eq!(trk.flux, track % 2 == 1, "track {track} side {side} kind");
            if !trk.flux {
                continue;
            }
            largest = largest.max(trk.blocks);
            let data = &file[o.start as usize * BLOCK..];
            let bits: Vec<bool> = (0..o.bits as usize).map(|i| data[i / 8] >> (7 - i % 8) & 1 != 0).collect();
            let (got, fuzzy) = from_flux(&flux, &trk, mfm);
            flagged += fuzzy.len();
            assert_eq!(got.len(), bits.len(), "track {track} side {side}: cells per revolution");
            // Flux starts after the last reversal of the previous turn.
            let last = bits.iter().rposition(|&b| b).unwrap();
            let n = bits.len();
            let same = (0..n).all(|i| got[i] == bits[(i + last + 1) % n]);
            assert!(same, "track {track} side {side}: cells differ");
        }
    }
    println!(
        "{name}: {flux_sides} sides as flux (drive {:.0}% {}, ±{jitter} ticks, up to {largest} blocks), all convert back bit-exact, {flagged} jittered reversals taken for fuzzy",
        (speed - 1.0).abs() * 100.0,
        if speed > 1.0 { "slow" } else { "fast" }
    );
}

/// GCR 6-and-2 nibbles, for reading sector numbers from address fields.
const ENC: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

/// Address marks in a run of cells: (cell where `D5 AA 96` starts, sector number).
fn gcr_marks(cells: &[bool]) -> Vec<(usize, Option<u8>)> {
    let mut nib = vec![];
    let mut sh = 0u8;
    for (i, &c) in cells.iter().enumerate() {
        sh = sh << 1 | c as u8;
        if sh & 0x80 != 0 {
            nib.push((sh, i));
            sh = 0;
        }
    }
    let mut out = vec![];
    for k in 0..nib.len().saturating_sub(5) {
        if (nib[k].0, nib[k + 1].0, nib[k + 2].0) == (0xD5, 0xAA, 0x96) {
            out.push((nib[k].1 - 7, ENC.iter().position(|&e| e == nib[k + 4].0).map(|s| s as u8)));
        }
    }
    out
}

/// GCR sectors in a run of cells, framed as an IWM would: (sector, decoded tags + data) for every
/// address field whose data field follows whole and decodes.
fn gcr_sectors(cells: &[bool]) -> Vec<(u8, [u8; 524])> {
    let mut nib = vec![];
    let mut sh = 0u8;
    for &c in cells {
        sh = sh << 1 | c as u8;
        if sh & 0x80 != 0 {
            nib.push(sh);
            sh = 0;
        }
    }
    let mut out = vec![];
    let mut sector = None;
    for k in 0..nib.len().saturating_sub(3) {
        match (nib[k], nib[k + 1], nib[k + 2]) {
            (0xD5, 0xAA, 0x96) if k + 5 < nib.len() => {
                sector = ENC.iter().position(|&e| e == nib[k + 4]).map(|s| s as u8);
            }
            (0xD5, 0xAA, 0xAD) if k + 4 + 703 <= nib.len() => {
                let body: &[u8; 703] = nib[k + 4..k + 4 + 703].try_into().unwrap();
                if let (Some(s), Some(d)) = (sector.take(), gcr::decode_sector(body)) {
                    out.push((s, d));
                }
            }
            _ => {}
        }
    }
    out
}

/// One RWCP capture of an A2R 3 file: its location (track × 2 + side for 3.5" disks), the time
/// of one revolution, and the flux bytes, in 62.5 ns ticks.
struct A2rCapture<'a> {
    location: u16,
    revolution: u32,
    flux: &'a [u8],
}

/// The timing (type 1) captures of an A2R 3 file.
fn a2r_captures(d: &[u8]) -> Vec<A2rCapture<'_>> {
    assert_eq!(&d[..8], b"A2R3\xFF\x0A\x0D\x0A", "A2R 3 signature");
    let u32_at = |at: usize| u32::from_le_bytes(d[at..at + 4].try_into().unwrap());
    let mut out = vec![];
    let mut i = 8;
    while i + 8 <= d.len() {
        let size = u32_at(i + 4) as usize;
        if &d[i..i + 4] == b"RWCP" {
            assert_eq!(u32_at(i + 9), 62_500, "RWCP resolution in ps");
            // Version, resolution, 11 reserved bytes; then 'C' entries until 'X'.
            let mut j = i + 8 + 16;
            while d[j] == b'C' {
                let kind = d[j + 1];
                let location = u16::from_le_bytes([d[j + 2], d[j + 3]]);
                let indexes = d[j + 4] as usize;
                let revolution = if indexes > 0 { u32_at(j + 5) } else { 0 };
                let len = u32_at(j + 5 + 4 * indexes) as usize;
                let at = j + 9 + 4 * indexes;
                if kind == 1 {
                    out.push(A2rCapture { location, revolution, flux: &d[at..at + len] });
                }
                j = at + len;
            }
            assert_eq!(d[j], b'X', "RWCP end mark");
        }
        i += 8 + size;
    }
    out
}

fn check_a2r(a2r: &str, d: &[u8], file: &[u8]) {
    let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    let captures = a2r_captures(d);
    assert_eq!(captures.len(), 160, "one timing capture per track side");
    let (mut matched, mut flagged) = (0, 0);
    let mut deviations = vec![];
    for c in &captures {
        let (track, side) = ((c.location / 2) as u8, (c.location % 2) as u8);
        // 62.5 ns ticks to MOOF's 125 ns, one interval at a time; also count the cells in the
        // first revolution.
        let mut conv = moof::FluxCells::new(false);
        let mut cells = vec![];
        let (mut pending, mut elapsed, mut rev_cells) = (0u64, 0u64, None);
        for &b in c.flux {
            pending += u64::from(b);
            if b == 255 {
                continue;
            }
            let mut bytes = vec![];
            push_ticks(&mut bytes, (pending + 1) / 2);
            conv.feed(&bytes, |i, other| {
                cells.resize(i + 1, false);
                cells[i] = true;
                flagged += (other != moof::Other::None) as usize;
            });
            elapsed += pending;
            pending = 0;
            if rev_cells.is_none() && elapsed >= u64::from(c.revolution) {
                rev_cells = Some(conv.cells());
            }
        }
        // Every sector in the capture matches the MOOF's.
        let (_, trk) = m.track(track, side).expect("MOOF track");
        let data = &file[trk.start as usize * BLOCK..];
        let n = trk.bits as usize;
        let bits: Vec<bool> = (0..2 * n).map(|i| data[i % n / 8] >> (7 - i % n % 8) & 1 != 0).collect();
        let want = gcr_sectors(&bits);
        let mut got = gcr_sectors(&cells);
        got.sort_by_key(|g| g.0);
        got.dedup_by_key(|g| g.0);
        let spt = gcr::sectors_per_track(track) as usize;
        assert_eq!(got.len(), spt, "track {track} side {side}: sectors decoded from flux");
        for (sector, d) in &got {
            let w = want.iter().find(|w| w.0 == *sector).expect("sector in the MOOF");
            assert!(w.1 == *d, "track {track} side {side} sector {sector}: flux and MOOF differ");
            matched += 1;
        }
        deviations.push((rev_cells.expect("a whole revolution") as f64 / n as f64 - 1.0).abs() * 100.0);
    }
    // Cells in one revolution against the MOOF's bit count. Fast Imager gives every track of a
    // zone the same standard length; the real disk's duplicator wrote some tracks at ~1.995 µs
    // cells, so a revolution can hold a few percent more.
    // A few near the IWM's boundary, as on the real disk (51); a looser `FUZZY` takes thousands.
    assert!(flagged < 200, "{flagged} reversals taken for fuzzy");
    deviations.sort_by(f64::total_cmp);
    println!(
        "{a2r}: A2R 3 flux, all {matched} sectors decode and match the MOOF; cells per revolution vs the MOOF: median {:.1}%, worst {:.1}%; {flagged} reversals taken for fuzzy",
        deviations[deviations.len() / 2],
        deviations[deviations.len() - 1]
    );
}

/// Oids, a FLUX MOOF: track 1 side 0 is flux, the other 159 sides bitstream. Its FLUX chunk is
/// in the file's last, partial block, as `image::load` must handle. The flux track converts to a
/// normal-looking track, with ambiguous (fuzzy) reversals in one sector.
fn check_oids() {
    let name = "Oids";
    let (_, file) = disks::oids();
    let header: &[u8; moof::HEADER_BYTES] = file[..moof::HEADER_BYTES].try_into().unwrap();
    let block = moof::Moof::flux_block(header).expect("Oids has FLUX");
    // As `load_candidate` captures it: the block's bytes, zero-padded.
    let tail = &file[block * BLOCK..];
    assert!(tail.len() < BLOCK, "the FLUX chunk is the last, partial block");
    let mut chunk = [0u8; BLOCK];
    chunk[..tail.len()].copy_from_slice(tail);
    let m = moof::Moof::parse(header, Some(&chunk), file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    assert!(m.has_flux);
    let mut flux_sides = vec![];
    for track in 0..TRACKS {
        for side in 0..2u8 {
            if let Some((_, trk)) = m.track(track, side).filter(|(_, t)| t.flux) {
                flux_sides.push((track, side, trk));
            }
        }
    }
    assert_eq!(flux_sides.len(), 1);
    let (track, side, trk) = flux_sides[0];
    assert_eq!((track, side), (1, 0));
    let (cells, fuzzy) = from_flux(&file, &trk, false);
    let n = cells.len();
    // One revolution, close to its bitstream neighbours' lengths.
    let (_, t0) = m.track(0, 0).unwrap();
    assert!((n as f64 / t0.bits as f64 - 1.0).abs() < 0.01, "{n} cells");
    // Every address field and data mark is there; looped twice so a sector across the wrap
    // counts. Sector 5's data field holds the fuzzy reversals.
    let twice: Vec<bool> = cells.iter().chain(&cells).copied().collect();
    let mut sectors: Vec<u8> = gcr_sectors(&twice).into_iter().map(|s| s.0).collect();
    sectors.sort();
    sectors.dedup();
    // Converted with fixed rounding, sector 5 always fails its checksum; a real drive reads it
    // differently on each pass.
    assert_eq!(sectors, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11]);
    // The firmware's rotation on the converted cells: the wrap lands mid-gap.
    let mut packed = vec![0u8; n.div_ceil(8)];
    for (i, &c) in cells.iter().enumerate() {
        packed[i / 8] |= (c as u8) << (7 - i % 8);
    }
    let rot = moof::find_rotation(|i| packed[i], n as u32);

    // The fuzzy reversals are all in sector 5's data field: after its data mark, before the
    // next address mark.
    let marks = gcr_marks(&cells);
    let s5 = marks.iter().position(|&(_, m)| m == Some(5)).expect("sector 5 header");
    let (from, to) = (marks[s5].0, marks[s5 + 1].0);
    assert!(!fuzzy.is_empty() && fuzzy.iter().all(|&(i, _)| (from..to).contains(&i)), "fuzzy outside sector 5");

    // Played the firmware's way (rotated, the weak list randomized before each pass), sector 5
    // reads differently from pass to pass, and the rest never changes.
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    let words = moof::load_track(|i| packed[i], n as u32, rot, moof::Pad::Gcr, &mut buf);
    let mut stored = weak::WeakList::new();
    for &(i, other) in &fuzzy {
        stored.push_fuzzy(i, other == moof::Other::Later);
    }
    let mut list = weak::WeakList::new();
    list.find_runs(&buf[..], n, weak::GCR_RUN);
    let runs = list.len();
    list.add_fuzzy(&stored, n, rot as usize, &buf[..]);
    let mut rng = weak::Rng::new(1);
    let mut reads = std::collections::HashSet::new();
    let mut ok = 0;
    for _ in 0..20 {
        list.randomize(&mut buf[..], &mut rng);
        let pass: Vec<bool> = (0..2 * words * 32).map(|i| buf[i % (words * 32) / 32] >> (31 - i % 32) & 1 != 0).collect();
        let got = gcr_sectors(&pass);
        let mut others: Vec<u8> = got.iter().map(|g| g.0).filter(|&s| s != 5).collect();
        others.sort();
        others.dedup();
        assert_eq!(others, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11], "the other sectors stay readable");
        ok += got.iter().any(|g| g.0 == 5) as usize;
        // What the IWM would hand the ROM for sector 5: its data field's nibbles.
        let at = (from + n - rot as usize) % n;
        reads.insert((at..at + (to - from)).map(|i| pass[i]).collect::<Vec<_>>());
    }
    println!(
        "{name}: FLUX chunk in a partial last block; track {track} side {side} is flux ({} bytes): {n} cells, \
         all sectors but 5 decode, wrap at cell {rot}; {} fuzzy reversals, all in sector 5 ({} kept, {runs} no-flux runs); \
         20 randomized passes give {} different sector 5 readings ({ok} with a good checksum)",
        trk.bits,
        fuzzy.len(),
        list.len() - runs,
        reads.len()
    );
}

fn main() {
    check_converter();
    check_oids();
    let (_, moof, a2r) = disks::labelwriter();
    check_a2r("LabelWriter II", &a2r, &moof);
    let pascal = disks::moof(&disks::pascal());
    check_image("Macintosh Pascal", &pascal, false, 1.01, 3);
    let mfm = disks::moof(&disks::mfm_disk(&disks::random_bytes(1_474_560, 0x1440)));
    check_image("1.44 MB MOOF", &mfm, true, 1.01, 1);
    check_image("1.44 MB MOOF", &mfm, true, 0.99, 1);
    println!("all flux checks passed");
}
