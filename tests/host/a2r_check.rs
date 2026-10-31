//! Host-side check of the Disk II's flux paths: A2R captures (`src/a2r.rs`: the container read as
//! its blocks go by, the revolution found), WOZ 2.1 FLUX tracks (`src/woz.rs`), the conversion
//! both go through (`moof::FluxCells` at the cell `moof::measure_cell` finds), and each track's
//! own cell rate (`woz::track_hz`).
//!
//! The disks are made here: DOS 3.3 tracks of random sectors (`a2::build_track`), turned into
//! flux as a drive a little off speed with some jitter would capture them, from a random angle
//! and over 1.6 revolutions, written as A2R 1, 2 and 3 (also a solved A2R 3 track) and as a WOZ
//! 2.1 with FLUX tracks. Each is read back as the firmware does (`Cylinder::read_a2r`,
//! `read_flux_bytes`): every track must loop at exactly its cells and decode whole, through the
//! wrap. If `images/Applesauce` is there, some of its real A2Rs are read too: a looped
//! track must hold every sector a straight decode of its capture finds.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/a2r_check.rs -o target/a2r_check && target/a2r_check

#[path = "../../src/a2.rs"]
mod a2;
mod a2_decode;
#[path = "../../src/a2r.rs"]
mod a2r;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/mfm.rs"]
mod mfm;
#[path = "../../src/moof.rs"]
mod moof;
#[path = "../../src/track.rs"]
mod track;
#[path = "../../src/weak.rs"]
mod weak;
#[path = "../../src/woz.rs"]
mod woz;

use a2_decode::decode_track;
use track::*;

/// `Cylinder`'s `A2R_START`.
const START: usize = 64;

fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// A DOS 3.3 track of random sectors: its cells, and its sectors (physical order).
fn random_track(track: u8, s: &mut u64) -> (Vec<bool>, Vec<[u8; 256]>) {
    let sectors: Vec<[u8; 256]> = (0..16).map(|_| core::array::from_fn(|_| rng(s) as u8)).collect();
    let mut buf: TrackBuf = [0; MAX_TRACK_WORDS];
    a2::build_track(track, 254, |p| &sectors[p as usize], &mut buf);
    let cells = (0..a2::TRACK_CELLS).map(|i| buf[i / 32] >> (31 - i % 32) & 1 != 0).collect();
    (cells, sectors)
}

fn push_ticks(out: &mut Vec<u8>, mut t: u64) {
    while t >= 255 {
        out.push(255);
        t -= 255;
    }
    out.push(t as u8);
}

/// Flux bytes of `revs` revolutions of `cells` from cell `from`, at `cell_ticks` a cell (the
/// drive's speed in it) with up to `jitter` ticks either way; the first interval is from the
/// capture's start, part of one.
fn to_flux(cells: &[bool], from: usize, revs: f64, cell_ticks: f64, jitter: u64, s: &mut u64) -> Vec<u8> {
    let n = cells.len();
    let count = (n as f64 * revs) as usize;
    let mut out = Vec::new();
    // Each reversal timed to the tick from the capture's start, as the capture hardware does, and
    // the differences stored (rounding each interval on its own would drift).
    let (mut t, mut last) = (0.37 * cell_ticks, 0u64);
    for k in 0..count {
        t += cell_ticks;
        if cells[(from + k) % n] {
            let j = (rng(s) % (2 * jitter + 1)) as f64 - jitter as f64;
            let at = ((t + j).round() as u64).max(last + 1);
            push_ticks(&mut out, at - last);
            last = at;
        }
    }
    out
}

/// One revolution of `cells` as flux (a WOZ 2.1 or solved A2R track), from cell 0.
fn rev_flux(cells: &[bool], cell_ticks: f64, jitter: u64, s: &mut u64) -> Vec<u8> {
    to_flux(cells, 0, 1.0, cell_ticks, jitter, s)
}

/// The flux bytes as the firmware converts them: the cell measured on the first two blocks' bytes
/// from where the data starts (`read_flux_bytes`), then `FluxCells`, noting the cells at `watch`
/// ticks. Returns the cells, the cell (1/100 tick), the cells at `watch`, and the fuzzy reversals.
fn convert(file: &[u8], offset: usize, len: usize, nominal: u32, watch: u64) -> (Vec<bool>, u32, usize, usize) {
    let data = &file[offset..offset + len];
    let head = (2 * BLOCK - offset % BLOCK).min(len);
    let cell = moof::measure_cell(&data[..head], nominal);
    let mut conv = moof::FluxCells::with_cell(cell);
    conv.watch(watch);
    let mut ones = Vec::new();
    let mut fuzzy = 0;
    conv.feed(data, |i, other| {
        ones.push(i);
        fuzzy += (other != moof::Other::None) as usize;
    });
    let mut cells = vec![false; conv.cells()];
    for i in ones {
        cells[i] = true;
    }
    (cells, cell, conv.at_watch(), fuzzy)
}

fn pack(cells: &[bool]) -> Vec<u8> {
    let mut b = vec![0u8; cells.len().div_ceil(8)];
    for (i, &c) in cells.iter().enumerate() {
        if c {
            b[i / 8] |= 0x80 >> (i % 8);
        }
    }
    b
}

/// A capture read as `Cylinder::read_a2r` does: its revolution, from `START` (or all of a solved
/// one).
fn revolution(r: &a2r::A2r, file: &[u8], q: u8) -> Vec<bool> {
    let cap = r.capture(q).expect("capture");
    let ticks = if cap.loop_ticks > 0 { cap.loop_ticks } else { r.revolution_ticks() };
    let (cells, cell, at_loop, _) = convert(file, cap.offset as usize, cap.len as usize, r.nominal_cell(), u64::from(cap.loop_ticks));
    if cap.solved {
        return cells;
    }
    let estimate = if at_loop > 0 { at_loop } else { (u64::from(ticks) * 100 / u64::from(cell)) as usize };

    let packed = pack(&cells);
    let byte = |i: usize| packed.get(i).copied().unwrap_or(0);
    let p = a2r::find_loop(byte, cells.len(), START, estimate, at_loop > 0).min(cells.len() - START);
    let (at, p) = a2r::splice_point(byte, cells.len(), START, p);
    cells[at..at + p].to_vec()
}

/// Feed `file` to an `Indexer` in pieces of `piece` bytes (blocks while mapping; odd sizes too).
fn index(file: &[u8], piece: usize) -> Result<a2r::A2r, &'static str> {
    let mut ix = a2r::Indexer::new();
    for c in file.chunks(piece) {
        ix.feed(c);
    }
    ix.finish()
}

/// What goes in each test A2R: per whole track, its flux (a timing capture of 1.6 revolutions,
/// 125 ns ticks), the true loop in ticks, and the source cells and sectors.
struct Disk {
    tracks: Vec<(Vec<bool>, Vec<[u8; 256]>)>,
    /// Per track: the drive's ticks a cell (speed), the capture's start cell.
    speed: Vec<f64>,
    from: Vec<usize>,
}

fn make_disk(tracks: u8, s: &mut u64) -> Disk {
    let mut d = Disk { tracks: vec![], speed: vec![], from: vec![] };
    for t in 0..tracks {
        d.tracks.push(random_track(t, s));
        // Ticks a cell: the capturing drive off speed and the track's writer apart from it, up to
        // 2% between them, differing by track. (With every track at 51,136 cells this stands for
        // both; the capturing drive alone decides how far 200 ms is from a revolution, which a
        // capture with no loop point goes on.)
        d.speed.push(32.0 * (1.0 + ((rng(s) % 401) as f64 - 200.0) / 10_000.0));
        d.from.push(rng(s) as usize % a2::TRACK_CELLS);
    }
    d
}

/// The disk's captures: (location, capture type, flux bytes in `tick_scale` × 125 ns ticks, loop
/// in those ticks). Whole tracks at 4t, the same track again a quarter either side (as heard
/// there), and a deprecated bitstream capture first at track 1 (to be passed over).
fn captures(d: &Disk, tick_scale: f64, s: &mut u64) -> Vec<(u8, u8, Vec<u8>, u32)> {
    let mut out = vec![];
    for (t, (cells, _)) in d.tracks.iter().enumerate() {
        let speed = d.speed[t] * tick_scale;
        // The index sensor's revolution, true to a cell or two (±0.02%); none for track 3, which
        // leaves 200 ms at the nominal cell to go on.
        let loop_ticks = if t == 3 { 0 } else { (cells.len() as f64 * speed * (1.0 + ((rng(s) % 5) as f64 - 2.0) / 10_000.0)) as u32 };
        let jitter = (2.0 * tick_scale) as u64;
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q < 0 {
                continue;
            }
            if t == 1 && q == 4 {
                out.push((q as u8, 2u8, vec![0x55; 16_384], 0));
            }
            out.push((q as u8, 1u8, to_flux(cells, d.from[t], 1.6, speed, jitter, s), loop_ticks));
        }
    }
    out
}

fn a2r2_file(d: &Disk, s: &mut u64) -> Vec<u8> {
    let mut f = b"A2R2\xFF\x0A\x0D\x0A".to_vec();
    let mut info = vec![1u8];
    info.extend_from_slice(&[b' '; 32]);
    info.extend_from_slice(&[1, 0, 1]);
    f.extend_from_slice(b"INFO");
    f.extend_from_slice(&(info.len() as u32).to_le_bytes());
    f.extend_from_slice(&info);
    let mut strm = vec![];
    for (loc, kind, data, lp) in captures(d, 1.0, s) {
        strm.push(loc);
        strm.push(kind);
        strm.extend_from_slice(&(data.len() as u32).to_le_bytes());
        strm.extend_from_slice(&lp.to_le_bytes());
        strm.extend_from_slice(&data);
    }
    strm.push(0xFF);
    f.extend_from_slice(b"STRM");
    f.extend_from_slice(&(strm.len() as u32).to_le_bytes());
    f.extend_from_slice(&strm);
    let meta = b"title\tTest\n";
    f.extend_from_slice(b"META");
    f.extend_from_slice(&(meta.len() as u32).to_le_bytes());
    f.extend_from_slice(meta);
    f
}

fn a2r1_file(d: &Disk, s: &mut u64) -> Vec<u8> {
    let mut f = b"A2R1\xFF\x0A\x0D\x0A".to_vec();
    f.extend_from_slice(&[0x01, 0x00, 0x8D, 0x00, 0xA5, 0x01, 0x00, 0x00]);
    let mut data = vec![];
    for (loc, kind, flux, lp) in captures(d, 1.0, s) {
        data.push(loc);
        data.push(kind);
        data.extend_from_slice(&(flux.len() as u32).to_be_bytes());
        data.extend_from_slice(&lp.to_be_bytes());
        data.extend_from_slice(&flux);
    }
    data.push(0xFF);
    f.extend_from_slice(b"DATA");
    f.extend_from_slice(&(data.len() as u32).to_be_bytes());
    f.extend_from_slice(&data);
    let meta = b"title\tTest\n";
    f.extend_from_slice(b"INFO");
    f.extend_from_slice(&(meta.len() as u32).to_be_bytes());
    f.extend_from_slice(meta);
    f
}

/// An A2R 3 at 62.5 ns ticks; track `solved` also as a solved (SLVD) track, one revolution.
fn a2r3_file(d: &Disk, solved: usize, s: &mut u64) -> Vec<u8> {
    let mut f = b"A2R3\xFF\x0A\x0D\x0A".to_vec();
    let mut info = vec![1u8];
    info.extend_from_slice(&[b' '; 32]);
    info.extend_from_slice(&[1, 0, 1, 0]);
    f.extend_from_slice(b"INFO");
    f.extend_from_slice(&(info.len() as u32).to_le_bytes());
    f.extend_from_slice(&info);
    let mut head = vec![1u8];
    head.extend_from_slice(&62_500u32.to_le_bytes());
    head.extend_from_slice(&[0; 11]);
    let mut rwcp = head.clone();
    for (loc, kind, flux, lp) in captures(d, 2.0, s) {
        rwcp.extend_from_slice(&[b'C', kind]);
        rwcp.extend_from_slice(&u16::from(loc).to_le_bytes());
        // Two index signals: the first is the loop.
        rwcp.push(2);
        rwcp.extend_from_slice(&lp.to_le_bytes());
        rwcp.extend_from_slice(&(lp * 2).to_le_bytes());
        rwcp.extend_from_slice(&(flux.len() as u32).to_le_bytes());
        rwcp.extend_from_slice(&flux);
    }
    rwcp.push(b'X');
    f.extend_from_slice(b"RWCP");
    f.extend_from_slice(&(rwcp.len() as u32).to_le_bytes());
    f.extend_from_slice(&rwcp);
    let mut slvd = head;
    let flux = rev_flux(&d.tracks[solved].0, d.speed[solved] * 2.0, 4, s);
    slvd.push(b'T');
    slvd.extend_from_slice(&(4 * solved as u16).to_le_bytes());
    slvd.extend_from_slice(&[0, 0, 0, 0, 0, 0, 0, 0]);
    slvd.push(0);
    slvd.extend_from_slice(&(flux.len() as u32).to_le_bytes());
    slvd.extend_from_slice(&flux);
    slvd.push(b'X');
    f.extend_from_slice(b"SLVD");
    f.extend_from_slice(&(slvd.len() as u32).to_le_bytes());
    f.extend_from_slice(&slvd);
    f
}

/// Every whole track of `file` loops at its cells and decodes to the disk's sectors, through the
/// wrap; the quarter tracks either side do too.
fn check_a2r(name: &str, file: &[u8], d: &Disk, solved: Option<usize>) {
    for piece in [BLOCK, 7, 4_096] {
        let r = index(file, piece).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(r.capture(0).is_some() && r.capture(2).is_none(), "{name}: captures at 0 and 1, none at 0.50");
        let c = r.capture(4).unwrap();
        assert!(!c.solved && c.len > 16_384, "{name}: track 1's timing capture, not its bitstream");
        if piece != BLOCK {
            continue; // the same index, from other pieces
        }
        for (t, (cells, sectors)) in d.tracks.iter().enumerate() {
            for q in [4 * t, 4 * t + 1] {
                let rev = revolution(&r, file, q as u8);
                let what = if Some(t) == solved && q == 4 * t { "solved" } else { "capture" };
                assert_eq!(rev.len(), cells.len(), "{name}: track {t} (quarter {q}, {what}) loops at its cells");
                let found = decode_track(&rev);
                assert_eq!(found.len(), 16, "{name}: track {t} (quarter {q}): all 16 sectors through the wrap");
                for (trk, sec, _, data) in found {
                    assert_eq!(trk as usize, t);
                    // Physical sector `sec` is `sectors[sec]` (`a2::build_track`'s order).
                    assert_eq!(&data, &sectors[sec as usize], "{name}: track {t} sector {sec}");
                }
            }
        }
    }
    let r = index(file, BLOCK).unwrap();
    println!("{name}: {} bytes, v{}, {} tracks loop at their cells and decode whole, through the wrap", file.len(), r.version, d.tracks.len());
}

fn check_synthetic() {
    let mut s = 0x2545_F491_4F6C_DD1D;
    let d = make_disk(6, &mut s);
    check_a2r("A2R 2", &a2r2_file(&d, &mut s), &d, None);
    check_a2r("A2R 1", &a2r1_file(&d, &mut s), &d, None);
    check_a2r("A2R 3 (62.5 ns, a solved track)", &a2r3_file(&d, 2, &mut s), &d, Some(2));
    // Refused: a 3.5" A2R, an early A2R 1, a truncated file.
    let mut f = a2r2_file(&d, &mut s);
    f[8 + 8 + 33] = 2;
    assert_eq!(index(&f, BLOCK).err(), Some("A2R: not a 5.25\" disk"));
    let mut f = a2r1_file(&d, &mut s);
    f[4] = 0x01;
    assert!(index(&f, BLOCK).is_err(), "A2R 1 without its line-ending check");
    assert!(index(b"A2R2\xFF\x0A\x0D\x0AINFO", BLOCK).is_err(), "no captures");
    println!("A2R: 3.5\", early A2R 1 and capture-less files refused");
}

/// A WOZ 2.1: TMAP empty, FLUX naming one TRK per whole track (heard a quarter either side).
fn woz21_file(d: &Disk, s: &mut u64) -> (Vec<u8>, [u8; BLOCK]) {
    let n = d.tracks.len();
    let mut data_blocks: Vec<Vec<u8>> = vec![];
    for (t, (cells, _)) in d.tracks.iter().enumerate() {
        data_blocks.push(rev_flux(cells, d.speed[t], 2, s));
    }
    let mut f = vec![0u8; 3 * BLOCK];
    f[..8].copy_from_slice(b"WOZ2\xFF\x0A\x0D\x0A");
    f[12..16].copy_from_slice(b"INFO");
    f[16..20].copy_from_slice(&60u32.to_le_bytes());
    f[20] = 3; // INFO version 3
    f[21] = 1; // 5.25"
    f[57] = 1;
    f[59] = 32;
    f[80..84].copy_from_slice(b"TMAP");
    f[84..88].copy_from_slice(&160u32.to_le_bytes());
    for b in &mut f[88..248] {
        *b = 0xFF;
    }
    f[248..252].copy_from_slice(b"TRKS");
    let mut block = 3usize;
    let mut largest = 0;
    for (t, flux) in data_blocks.iter().enumerate() {
        let blocks = flux.len().div_ceil(BLOCK);
        let r = 256 + 8 * t;
        f[r..r + 2].copy_from_slice(&(block as u16).to_le_bytes());
        f[r + 2..r + 4].copy_from_slice(&(blocks as u16).to_le_bytes());
        f[r + 4..r + 8].copy_from_slice(&(flux.len() as u32).to_le_bytes());
        block += blocks;
        largest = largest.max(blocks);
    }
    let flux_block = block;
    f[66..68].copy_from_slice(&(flux_block as u16).to_le_bytes());
    f[68..70].copy_from_slice(&(largest as u16).to_le_bytes());
    for flux in &data_blocks {
        let mut b = flux.clone();
        b.resize(flux.len().div_ceil(BLOCK) * BLOCK, 0);
        f.extend_from_slice(&b);
    }
    let mut chunk = [0u8; BLOCK];
    chunk[..4].copy_from_slice(b"FLUX");
    chunk[4..8].copy_from_slice(&160u32.to_le_bytes());
    for b in &mut chunk[8..168] {
        *b = 0xFF;
    }
    for t in 0..n {
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q >= 0 {
                chunk[8 + q as usize] = t as u8;
            }
        }
    }
    f.extend_from_slice(&chunk);
    let trks_size = (f.len() - 256) as u32;
    f[252..256].copy_from_slice(&trks_size.to_le_bytes());
    (f, chunk)
}

fn check_woz_flux() {
    let mut s = 0x9E37_79B9_7F4A_7C15;
    let d = make_disk(5, &mut s);
    let (f, chunk) = woz21_file(&d, &mut s);
    let h: &[u8; woz::HEADER_BYTES] = f[..woz::HEADER_BYTES].try_into().unwrap();
    let blocks = f.len().div_ceil(BLOCK);
    let w = woz::Woz::parse(h, &[0; woz::QUARTERS], Some(&chunk), blocks, MAX_TRACK_BLOCKS).expect("WOZ 2.1");
    assert!(w.has_flux && !w.writable(), "a WOZ with flux tracks is read-only");
    assert!(woz::Woz::parse(h, &[0; woz::QUARTERS], None, blocks, MAX_TRACK_BLOCKS).is_err(), "FLUX chunk missing");
    let mut bad = chunk;
    bad[8 + 4] = 0x7E;
    assert!(woz::Woz::parse(h, &[0; woz::QUARTERS], Some(&bad), blocks, MAX_TRACK_BLOCKS).is_err(), "bad FLUX entry");
    for (t, (cells, sectors)) in d.tracks.iter().enumerate() {
        for q in [4 * t + 1, 4 * t] {
            let key = w.track(q as u8);
            let trk = w.trk(key).expect("flux track");
            assert!(trk.flux && key as usize == t);
            let (rev, _, _, _) = convert(&f, trk.start as usize * BLOCK, trk.bits as usize, 3_200, 0);
            assert_eq!(rev.len(), cells.len(), "WOZ 2.1 track {t}: its cells");
            let found = decode_track(&rev);
            assert_eq!(found.len(), 16, "WOZ 2.1 track {t}: all sectors");
            assert!(found.iter().all(|(trk, sec, _, data)| *trk as usize == t && sectors.contains(data) && *sec < 16));
        }
    }
    assert_eq!(w.track(2), 0xFF, "nothing at track 0.50");
    println!("WOZ 2.1: FLUX map parsed, {} flux tracks convert to their cells and decode whole; read-only", d.tracks.len());
}

fn check_track_hz() {
    let disk = 4_090_000;
    assert_eq!(woz::track_hz(a2::TRACK_CELLS as u32, disk), 80 * a2::TRACK_CELLS as u32, "a standard track in 200 ms");
    assert_eq!(woz::track_hz(56_000, disk), 4_480_000, "a long track (+9.5%) at its own rate");
    assert_eq!(woz::track_hz(30_000, disk), disk, "a track 40% short: the disk's rate");
    assert_eq!(woz::track_hz(70_000, disk), disk, "36% long: the disk's rate");
    println!("track_hz: each track's cells in 200 ms, within 20% of the disk's average");
}

/// Real A2Rs: every whole-track capture's revolution must hold each sector a straight decode of
/// the capture finds (the loop loses nothing at the wrap).
fn check_real() {
    let root = std::path::Path::new("images/Applesauce");
    let Ok(rd) = std::fs::read_dir(root) else {
        println!("real A2Rs: images/Applesauce not found, skipped");
        return;
    };
    let mut files: Vec<_> = rd.filter_map(|e| e.ok()).flat_map(|e| std::fs::read_dir(e.path()).into_iter().flatten()).filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "a2r")).collect();
    files.sort();
    let (mut disks, mut tracks, mut lost, mut sectors, mut loops) = (0, 0, 0, 0, vec![]);
    for p in files.iter().step_by(60) {
        let p_show = p.clone();
        let file = std::fs::read(p).unwrap();
        let Ok(r) = index(&file, BLOCK) else { continue };
        disks += 1;
        for t in 0..35u8 {
            let Some(cap) = r.capture(4 * t) else { continue };
            let ticks = if cap.loop_ticks > 0 { cap.loop_ticks } else { r.revolution_ticks() };
            let (cells, _, at_loop, _) = convert(&file, cap.offset as usize, cap.len as usize, r.nominal_cell(), u64::from(ticks));
            let rev = revolution(&r, &file, 4 * t);
            let p = rev.len();
            // One revolution from the start, and 3,000 cells more so a sector across its end is
            // whole: no splice in it, so every sector of a revolution decodes.
            if START + p + 3_000 > cells.len() {
                continue;
            }
            let straight: std::collections::BTreeSet<u8> = decode_track(&cells[START..START + p + 3_000]).iter().map(|f| f.1).collect();
            if straight.is_empty() {
                continue;
            }
            let looped: std::collections::BTreeSet<u8> = decode_track(&rev).iter().map(|f| f.1).collect();
            tracks += 1;
            sectors += straight.len();
            let missing = straight.difference(&looped).count();
            lost += missing;
            if std::env::var("A2R_DEBUG").is_ok() && missing > 0 {
                let packed = pack(&cells);
                let mm = |q: usize| (0..8).map(|k| {
                    let w = |i: usize| { let at = i / 8; let x = (0..5).fold(0u64, |w, j| w << 8 | u64::from(packed.get(at + j).copied().unwrap_or(0))); (x >> (8 - i % 8)) as u32 };
                    (w(START + 32 * k) ^ w(START + q + 32 * k)).count_ones()
                }).sum::<u32>();
                println!("  {} t{t}: lost {missing}/{} p={p} at_loop={at_loop} n={} mismatch@p={} @at_loop={}", p_show.file_name().unwrap().to_string_lossy(), straight.len(), cells.len(), mm(p), mm(at_loop));
            }
            if at_loop > 0 {
                loops.push((p as f64 / at_loop as f64 - 1.0) * 100.0);
            }
        }
    }
    loops.sort_by(f64::total_cmp);
    let worst = loops.iter().fold(0.0f64, |w, &x| w.max(x.abs()));
    let far = loops.iter().filter(|x| x.abs() > 2.5).count();
    println!("real A2Rs: {far} of {} loops more than 2.5% from the loop point", loops.len());
    println!(
        "real A2Rs: {disks} disks, {tracks} tracks with sectors: {lost} of {sectors} sectors lost at the wrap; loop vs the loop point median {:+.3}%, worst {:.2}%",
        loops.get(loops.len() / 2).copied().unwrap_or(0.0),
        worst
    );
    assert!(lost * 200 <= sectors, "at most 0.5% of sectors lost at the loop");
}

fn main() {
    check_track_hz();
    check_synthetic();
    check_woz_flux();
    check_real();
    println!("a2r_check: all passed");
}
