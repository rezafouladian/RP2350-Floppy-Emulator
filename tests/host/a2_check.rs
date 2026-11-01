//! Host-side check of Apple II 5.25" support: the 6-and-2 and 5-and-3 codecs (`src/a2.rs`),
//! track building from sector (16 and 13) and `.nib` images, the stepper model, WOZ parsing (`src/woz.rs`) and the write
//! path (`writes::a2_fields`, `a2::sector_at`, `writes::splice_gcr` on WOZ tracks, and the GCR
//! separator in `capture.pio` at the Apple II's write clock). The WOZs are made at test time
//! (`disks.rs`): a DOS 3.3 disk, a ProDOS one, and one whose tracks are longer than its 
//! optimal bit timing says (as Border Zone's).
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/a2_check.rs -o target/a2_check && target/a2_check

#[path = "../../src/a2.rs"]
mod a2;
mod disks;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/mfm.rs"]
mod mfm;
#[path = "../../src/moof.rs"]
mod moof;
mod a2_decode;
mod pio_play;
mod pio_sim;
#[path = "../../src/track.rs"]
mod track;
#[path = "../../src/weak.rs"]
mod weak;
#[path = "../../src/woz.rs"]
mod woz;
#[path = "../../src/writes.rs"]
mod writes;

use a2_decode::{decode_track, decode_track13};
use track::*;
use writes::{CellTrack, Cells};


fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn random_sector(s: &mut u64) -> [u8; 256] {
    core::array::from_fn(|_| rng(s) as u8)
}

/// Cells as bools.
struct Bools(Vec<bool>);

impl Cells for Bools {
    fn bit(&self, i: usize) -> bool {
        self.0[i]
    }
}

fn buf_cells(buf: &TrackBuf, cells: usize) -> Vec<bool> {
    (0..cells).map(|i| buf[i / 32] >> (31 - i % 32) & 1 != 0).collect()
}

fn check_codec() {
    let mut s = 0x5A5Au64;
    for round in 0..2_000 {
        let data = if round == 0 { [0u8; 256] } else if round == 1 { [0xFF; 256] } else { random_sector(&mut s) };
        let mut nibbles = vec![];
        a2::encode_data(&data, |n| nibbles.push(n));
        assert_eq!(nibbles.len(), 343);
        assert_eq!(nibbles[..], disks::encode62(&data)[..], "encoder mismatch");
        let arr: [u8; 343] = nibbles.clone().try_into().unwrap();
        assert_eq!(a2::decode_data(&arr), Some(data));
        // A changed nibble is caught (by the checksum, or as a non-data nibble).
        let mut bad = arr;
        let k = (rng(&mut s) % 343) as usize;
        bad[k] = gcr::ENCODE[(gcr::decode6(bad[k]).unwrap() as usize + 1 + (rng(&mut s) % 62) as usize) % 64];
        assert_eq!(a2::decode_data(&bad), None);
    }
    for b in 0..=255u8 {
        let [x, y] = a2::encode44(b);
        assert!(x & 0xAA == 0xAA && y & 0xAA == 0xAA);
        assert_eq!(a2::decode44(x, y), b);
    }
    println!("codec: 6-and-2 matches and round-trips 2,000 sectors; 4-and-4 all bytes");
}

/// A WOZ of a DOS 3.3 or ProDOS disk: every sector, and the catalog or volume directory linked
/// up through the firmware's skew tables.
fn check_blank_woz(name: &str, file: &[u8]) {
    let h: [u8; woz::HEADER_BYTES] = file[..woz::HEADER_BYTES].try_into().unwrap();
    let blocks = file.len().div_ceil(BLOCK);
    let w = woz::Woz::parse(&h, &[0; woz::QUARTERS], None, blocks, MAX_TRACK_BLOCKS).expect("parse");
    let mut crc = moof::Crc32::new();
    crc.update(&file[12..]);
    assert!(w.stored_crc == 0 || w.stored_crc == crc.finish(), "CRC");
    assert_eq!(w.bit_timing, 32);
    let mut disk = vec![[0u8; 256]; 35 * 16];
    let mut sectors = 0;
    for t in 0..35u8 {
        let key = w.track(t * 4);
        assert_eq!(key, w.track(t * 4 + 1), "quarter after track {t}");
        let trk = w.trk(key).expect("track mapped");
        let data = &file[trk.start as usize * BLOCK..];
        let rot = moof::find_rotation(|i| data[i], trk.bits);
        let mut buf = [0u32; MAX_TRACK_WORDS];
        let words = moof::load_track(|i| data[i], trk.bits, rot, moof::Pad::Gcr, &mut buf);
        let cells = buf_cells(&buf, words * 32);
        let found = decode_track(&cells);
        for &(trk, sec, _, d) in &found {
            assert_eq!(trk, t);
            disk[t as usize * 16 + a2::file_sector(a2::Order::Dos, sec)] = d;
        }
        let mut secs: Vec<u8> = found.iter().map(|f| f.1).collect();
        secs.sort();
        secs.dedup();
        assert_eq!(secs.len(), 16, "track {t}: sectors {secs:?}");
        sectors += secs.len();
    }
    // What it holds: DOS 3.3 (VTOC at T17 S0) or ProDOS (volume directory at block 2).
    let vtoc = &disk[17 * 16];
    let dos = vtoc[1] == 17 && vtoc[3] == 3 && vtoc[0x27] == 122 && vtoc[0x34] == 35 && vtoc[0x35] == 16;
    if dos {
        // The catalog runs down track 17 from logical sector 15, each linking to the next: only
        // true if physical sectors were put in DOS order correctly.
        let mut s = vtoc[2];
        let mut n = 0;
        while s != 0 {
            let c = &disk[17 * 16 + s as usize];
            assert!(c[1] == 17 || c[1] == 0, "catalog sector {s} links to track {}", c[1]);
            if c[1] == 0 {
                break;
            }
            assert_eq!(c[2], s - 1, "catalog sector {s} links to {}", c[2]);
            s = c[2];
            n += 1;
        }
        assert_eq!(n, 14);
        println!("{name}: {sectors} sectors decode; DOS 3.3 VTOC, catalog chain of {n} links in DOS order");
    } else {
        let prodos_block = |b: usize| -> Vec<u8> {
            let t = b / 8;
            let phys_of = |slot: usize| (0..16u8).find(|&p| a2::file_sector(a2::Order::Prodos, p) == slot).unwrap();
            let dos_slot = |p: u8| a2::file_sector(a2::Order::Dos, p);
            let mut v = vec![];
            for half in 0..2 {
                v.extend_from_slice(&disk[t * 16 + dos_slot(phys_of((b % 8) * 2 + half))]);
            }
            v
        };
        let key = prodos_block(2);
        let len = key[4] & 0xF;
        println!(
            "{name}: {sectors} sectors decode; block 2 via ProDOS order: type {:X}, name {:?}, entry {} x {}",
            key[4] >> 4,
            String::from_utf8_lossy(&key[5..5 + len as usize]),
            key[0x23],
            key[0x24]
        );
        assert_eq!(key[4] >> 4, 0xF, "volume directory header");
        // The volume directory runs blocks 2-5, each linking to its neighbours.
        for b in 2..6usize {
            let blk = prodos_block(b);
            let (prev, next) = (u16::from_le_bytes([blk[0], blk[1]]), u16::from_le_bytes([blk[2], blk[3]]));
            assert_eq!(prev as usize, if b == 2 { 0 } else { b - 1 }, "block {b} prev");
            assert_eq!(next as usize, if b == 5 { 0 } else { b + 1 }, "block {b} next");
        }
        println!("{name}: ProDOS volume directory blocks 2-5 link up in ProDOS order");
    }
}

/// A stored track (a WOZ track's bytes), changeable.
struct Stored {
    bytes: Vec<u8>,
    n: usize,
}

impl Cells for Stored {
    fn bit(&self, i: usize) -> bool {
        let i = i % self.n;
        self.bytes[i / 8] >> (7 - i % 8) & 1 != 0
    }
}

impl CellTrack for Stored {
    fn len(&self) -> usize {
        self.n
    }
    fn set(&mut self, i: usize, v: bool) {
        let i = i % self.n;
        let m = 0x80 >> (i % 8);
        if v { self.bytes[i / 8] |= m } else { self.bytes[i / 8] &= !m }
    }
}

impl Stored {
    fn cells(&self) -> Vec<bool> {
        (0..self.n).map(|i| self.bit(i)).collect()
    }
}

fn check_stepper() {
    // Phase bit i = PHi. One magnet at a time, then each with its neighbour: whole and quarter
    // steps outward and back.
    let on = |i: u32| 1u32 << (i % 4);
    let mut q = 0u8;
    // DOS 3.3 style: half-track steps, the next magnet on, then the last one off.
    for half in 0..68u32 {
        q = a2::step(q, on(half) | on(half + 1));
        q = a2::step(q, on(half + 1));
        assert_eq!(q as u32, 2 * (half + 1), "half-track step {half}");
    }
    assert_eq!(q, 136, "track 34");
    for half in (0..68u32).rev() {
        q = a2::step(q, on(half + 1) | on(half));
        q = a2::step(q, on(half));
        assert_eq!(q as u32, 2 * half);
    }
    // Quarter tracks: one magnet holds the head on a half track, two neighbours between them.
    q = a2::step(0, on(0));
    for k in 0..40u32 {
        q = a2::step(q, on(k / 2) | on((k + 1) / 2));
        assert_eq!(q as u32, k, "quarter {k}");
    }
    // Opposite magnets, none, all four: no pull.
    for m in [0b0101, 0b1010, 0, 0b1111] {
        assert_eq!(a2::step(50, m), 50);
    }
    // A magnet 2 away pulls a half track; 3 magnets pull to the middle one.
    assert_eq!(a2::step(4, on(1)), 2);
    assert_eq!(a2::step(4, on(1) | on(2) | on(3)), 4);
    // The stops: recalibrating against track 0 (DOS steps 80 half tracks outward from wherever
    // it thinks it is), and the far end.
    q = 6;
    for half in (0..80u32).rev() {
        q = a2::step(q, on(half));
    }
    assert_eq!(q, 0, "recalibrate");
    q = a2::QUARTERS - 1;
    assert_eq!(a2::step(q, on(q as u32 / 2 + 1)), q, "far stop");
    // Every quarter track is heard as the nearest whole track, half tracks blank.
    for qt in 0..a2::QUARTERS {
        let t = a2::quarter_track(qt, 35);
        match qt % 4 {
            2 => assert_eq!(t, a2::BLANK),
            _ if (qt + 1) / 4 >= 35 => assert_eq!(t, a2::BLANK),
            _ => assert_eq!(t as u32, (qt as u32 + 1) / 4),
        }
    }
    println!("stepper: half-track seeks 0-34 and back, quarter tracks, no-pull cases, both stops");
}

/// A sector image of `tracks` tracks with random sectors.
fn random_image(tracks: usize, s: &mut u64) -> Vec<u8> {
    (0..tracks * a2::TRACK_BYTES).map(|_| rng(s) as u8).collect()
}

fn image_sector(img: &[u8], order: a2::Order, track: u8, phys: u8) -> &[u8; 256] {
    let at = track as usize * a2::TRACK_BYTES + a2::file_sector(order, phys) * 256;
    img[at..at + 256].try_into().unwrap()
}

fn check_sector_tracks() {
    let mut s = 0xA2A2u64;
    // Each order maps the 16 physical sectors onto the 16 slots.
    for order in [a2::Order::Dos, a2::Order::Prodos] {
        let mut slots: Vec<usize> = (0..16).map(|p| a2::file_sector(order, p)).collect();
        slots.sort();
        assert_eq!(slots, (0..16).collect::<Vec<_>>());
    }
    for (tracks, order) in [(35, a2::Order::Dos), (35, a2::Order::Prodos), (40, a2::Order::Dos)] {
        let img = random_image(tracks, &mut s);
        for t in 0..tracks as u8 {
            let mut buf = [0u32; MAX_TRACK_WORDS];
            let words = a2::build_track(a2::Encoding::SixAndTwo, t, 254, |p| image_sector(&img, order, t, p), &mut buf);
            assert_eq!(words * 32, a2::TRACK_CELLS);
            let cells = buf_cells(&buf, words * 32);
            let found = decode_track(&cells);
            assert_eq!(found.len(), 16, "track {t}");
            for (k, &(trk, sec, vol, d)) in found.iter().enumerate() {
                assert_eq!((trk, sec, vol), (t, k as u8, 254));
                assert_eq!(&d, image_sector(&img, order, t, sec));
            }
            // No weak-bit runs in what we encode.
            let mut w = weak::WeakList::new();
            w.find_runs(&buf, words * 32, weak::A2_RUN);
            assert_eq!(w.len(), 0);
        }
    }
    // Fitted to other lengths (WOZ tracks being reformatted): exact length, all sectors.
    let img = random_image(1, &mut s);
    for cells in (48_224..56_000).step_by(37).chain([50_000, 51_024, 51_200, 53_248]) {
        let mut buf = [0u32; MAX_TRACK_WORDS];
        a2::build_track_cells(a2::Encoding::SixAndTwo, 0, 254, |p| image_sector(&img, a2::Order::Dos, 0, p), cells, &mut buf).expect("fits");
        let found = decode_track(&buf_cells(&buf, cells));
        assert_eq!(found.len(), 16, "{cells} cells");
    }
    assert!(a2::build_track_cells(a2::Encoding::SixAndTwo, 0, 254, |p| image_sector(&img, a2::Order::Dos, 0, p), 48_000, &mut [0; MAX_TRACK_WORDS]).is_none());
    println!("sector tracks: 35/40 tracks, DOS and ProDOS order, every sector decodes in place; fitted lengths 48,224-56,000");
}

/// Nibble-level `.nib` track of `img`'s track `t`: what a nibble copier stores (8-bit `FF`s).
fn nib_track(img: &[u8], order: a2::Order, t: u8, lead: usize) -> Vec<u8> {
    let mut v = vec![0xFFu8; lead];
    for p in 0..16u8 {
        v.extend([0xFF; 16]);
        v.extend([0xD5, 0xAA, 0x96]);
        v.extend(a2::address_field(254, t, p));
        v.extend([0xDE, 0xAA, 0xEB]);
        v.extend([0xFF; 6]);
        v.extend([0xD5, 0xAA, 0xAD]);
        a2::encode_data(image_sector(img, order, t, p), |n| v.push(n));
        v.extend([0xDE, 0xAA, 0xEB]);
    }
    v.resize(a2::NIB_TRACK_BYTES, 0xFF);
    // Start mid-sector, as a .nib may.
    v.rotate_left(2_000);
    v
}

fn check_nib() {
    let mut s = 0x0123u64;
    let img = random_image(35, &mut s);
    for t in [0u8, 17, 34] {
        let nib = nib_track(&img, a2::Order::Dos, t, 40);
        let mut buf = [0u32; MAX_TRACK_WORDS];
        let words = a2::build_nib_track(|i| nib[i], &mut buf);
        let cells = buf_cells(&buf, words * 32);
        let found = decode_track(&cells);
        let mut secs: Vec<u8> = found.iter().map(|f| f.1).collect();
        secs.sort();
        secs.dedup();
        assert_eq!(secs.len(), 16, "nib track {t}");
        for &(_, sec, _, d) in &found {
            assert_eq!(&d, image_sector(&img, a2::Order::Dos, t, sec));
        }
        // Sync got its zeros back: an IWM framing from any cell falls into step within a few
        // sync nibbles.
        let zeros_after_ff = cells.windows(10).filter(|w| w[..8].iter().all(|&b| b) && !w[8] && !w[9]).count();
        assert!(zeros_after_ff > 300, "{zeros_after_ff}");
        for start in 0..10 {
            let shifted: Vec<bool> = cells[start..].iter().chain(&cells[..start]).copied().collect();
            assert!(decode_track(&shifted).len() >= 15, "framing from cell {start}");
        }
        println!("nib track {t}: {} words, 16 sectors decode from any start", words);
    }
}

/// The cells of a DOS 3.3 sector write, as RWTS's WRITE16 sends them: 5 sync (40-cycle `FF`s,
/// 10 cells), `D5 AA AD`, 343 nibbles, `DE AA EB`, one more `FF`.
fn a2_write_cells(data: &[u8; 256]) -> Vec<bool> {
    a2_write_cells_as(a2::Encoding::SixAndTwo, data)
}

/// [`a2_write_cells`] in either encoding: DOS 3.2's with 12 9-cell syncs (as its writes read on
/// real disks) and 411 5-and-3 nibbles.
fn a2_write_cells_as(enc: a2::Encoding, data: &[u8; 256]) -> Vec<bool> {
    let mut bits = vec![];
    let (zeros, syncs) = if enc == a2::Encoding::SixAndTwo { (2, 5) } else { (1, 12) };
    for _ in 0..syncs {
        bits.extend([true; 8]);
        bits.extend(vec![false; zeros]);
    }
    let mut nibbles = vec![0xD5, 0xAA, 0xAD];
    match enc {
        a2::Encoding::SixAndTwo => a2::encode_data(data, |n| nibbles.push(n)),
        a2::Encoding::FiveAndThree => a2::encode_data53(data, |n| nibbles.push(n)),
    }
    nibbles.extend([0xDE, 0xAA, 0xEB, 0xFF]);
    for n in nibbles {
        bits.extend((0..8).rev().map(|k| n >> k & 1 != 0));
    }
    bits
}

/// Cells written (the Apple II's 3.911 µs) through the real `gcr_write` program clocked for 32
/// cycles per cell, NRZI, with `speed` error and `jitter` cycles.
fn capture(cells: &[bool], speed: f64, jitter: u64, seed: u64) -> Vec<bool> {
    let src = std::fs::read_to_string("src/capture.pio").unwrap();
    let prog = pio_sim::parse(&src, "gcr_write");
    let mut s = seed | 1;
    let cell = 32.0 * (1.0 + speed);
    let lead = 200.0;
    let edges: Vec<u64> = cells
        .iter()
        .enumerate()
        .filter(|&(_, &b)| b)
        .map(|(i, _)| {
            let j = (rng(&mut s) % (2 * jitter + 1)) as f64 - jitter as f64;
            (lead + i as f64 * cell + j) as u64
        })
        .collect();
    let pin = |t: u64| edges.partition_point(|&e| e <= t) % 2 == 0;
    let total = (lead + cells.len() as f64 * cell) as u64 + 64 * 32;
    pio_sim::run(&prog, total, pin)
}

fn check_writes() {
    let mut s = 0xBEEFu64;
    // The firmware's clocks: 8,181,818 Hz capture for 3.911 µs cells is 32.0 cycles per cell.
    let per_cell = 8_181_818.0 * 3.9111e-6;
    assert!((per_cell - 32.0f64).abs() < 0.01, "{per_cell}");
    // A sector write through the separator, decoded: no address field, data intact.
    for (speed, jitter) in [(0.0, 2), (-0.02, 3), (0.02, 3), (0.0, 5)] {
        let data = random_sector(&mut s);
        let got = capture(&a2_write_cells(&data), speed, jitter, rng(&mut s));
        let mut fields = vec![];
        writes::a2_fields(&Bools(got.clone()), got.len(), a2::Encoding::SixAndTwo, |a, _, d| fields.push((a, *d)));
        assert_eq!(fields, vec![(None, data)], "speed {speed} jitter {jitter}");
    }
    let img = random_image(1, &mut s);
    let mut buf = [0u32; MAX_TRACK_WORDS];
    a2::build_track(a2::Encoding::SixAndTwo, 0, 254, |p| image_sector(&img, a2::Order::Dos, 0, p), &mut buf);
    let cells = buf_cells(&buf, a2::TRACK_CELLS);
    // A format (DOS INIT writes a whole track): its address fields name every sector.
    let got = capture(&cells, 0.01, 3, 99);
    let mut found = [false; 16];
    writes::a2_fields(&Bools(got.clone()), got.len(), a2::Encoding::SixAndTwo, |a, _, d| {
        let (t, sec) = a.expect("address field");
        assert_eq!(t, 0);
        assert_eq!(d, image_sector(&img, a2::Order::Dos, 0, sec));
        found[sec as usize] = true;
    });
    assert!(found.iter().all(|&f| f));
    println!("writes: sector writes decode at ±2% and ±5 cycles, a whole-track format decodes");
}

fn check_woz_writes() {
    let file = disks::woz525(&disks::dos33_image(), false, 51_200, 32);
    let h: [u8; woz::HEADER_BYTES] = file[..woz::HEADER_BYTES].try_into().unwrap();
    let w = woz::Woz::parse(&h, &[0; woz::QUARTERS], None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    let mut s = 0xC0DEu64;
    for t in [0u8, 17, 34] {
        let trk = w.trk(w.track(t * 4)).unwrap();
        let start = trk.start as usize * BLOCK;
        let mut stored = Stored { bytes: file[start..start + trk.blocks as usize * BLOCK].to_vec(), n: trk.bits as usize };
        let before = decode_track(&stored.cells());
        // Rewrite each sector in turn, as DOS would: the gate opens after its address field, in
        // the played (rotated) loop's coordinates.
        let rot = moof::find_rotation(|i| stored.bytes[i], trk.bits) as usize;
        let mut want: Vec<(u8, [u8; 256])> = before.iter().map(|f| (f.1, f.3)).collect();
        want.sort_by_key(|f| f.0);
        want.dedup_by_key(|f| f.0);
        for sec in 0..16u8 {
            let data = random_sector(&mut s);
            let cells = stored.cells();
            // Find sector `sec`'s address field end in stored coordinates.
            let mut at = None;
            let (mut sh, mut recent) = (0u8, 0u32);
            let mut nibs: Vec<(u8, usize)> = vec![];
            for i in 0..2 * cells.len() {
                sh = sh << 1 | cells[i % cells.len()] as u8;
                if sh & 0x80 != 0 {
                    nibs.push((sh, i));
                    recent = recent << 8 | sh as u32;
                    sh = 0;
                }
            }
            for k in 3..nibs.len() - 12 {
                if (nibs[k - 3].0, nibs[k - 2].0, nibs[k - 1].0) == (0xD5, 0xAA, 0x96) && a2::decode44(nibs[k + 4].0, nibs[k + 5].0) == sec {
                    at = Some(nibs[k + 10].1 + 1);
                    break;
                }
            }
            let _ = recent;
            let stored_pos = at.expect("address field") % cells.len();
            // The firmware gets the position in the rotated loop and adds the rotation back.
            let played_pos = (stored_pos + cells.len() - rot) % cells.len();
            let pos = (played_pos + rot) % cells.len();
            let got = capture(&a2_write_cells(&data), 0.0, 2, rng(&mut s));
            writes::splice_gcr(&mut stored, pos + 20, &Bools(got.clone()), got.len()).expect("splice");
            want[sec as usize].1 = data;
        }
        let mut after: Vec<(u8, [u8; 256])> = decode_track(&stored.cells()).iter().map(|f| (f.1, f.3)).collect();
        after.sort_by_key(|f| f.0);
        after.dedup_by_key(|f| f.0);
        assert_eq!(after.len(), 16, "track {t}");
        assert!(after == want, "track {t}: rewritten sectors");
        assert_eq!(stored.n, trk.bits as usize);
        // A format fitted back into the stored length.
        let img = random_image(1, &mut s);
        let mut buf = [0u32; MAX_TRACK_WORDS];
        a2::build_track_cells(a2::Encoding::SixAndTwo, t, 254, |p| image_sector(&img, a2::Order::Dos, 0, p), stored.n, &mut buf).expect("fits");
        writes::copy_track(&mut stored, &buf);
        let found = decode_track(&stored.cells());
        assert_eq!(found.len(), 16);
        assert!(found.iter().all(|f| f.0 == t && &f.3 == image_sector(&img, a2::Order::Dos, 0, f.1)));
    }
    println!("WOZ writes: all 16 sectors of tracks 0, 17, 34 rewritten by splice; a format re-encoded to the stored length");
}

fn check_other_woz() {
    // WOZs play at their tracks' length, a revolution in 200 ms, not INFO's optimal bit timing:
    // Border Zone's (one of Applesauce's WOZ test images) is 3.5 µs, but its tracks hold ~50,400
    // bits, 3.97 µs on the disk.
    {
        let file = disks::woz525(&disks::dos33_image(), false, 50_400, 28);
        let h: [u8; woz::HEADER_BYTES] = file[..woz::HEADER_BYTES].try_into().unwrap();
        let w = woz::Woz::parse(&h, &[0; woz::QUARTERS], None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
        assert_eq!(w.bit_timing, 28);
        let hz = w.cell_hz();
        assert!((3_960_000..4_080_000).contains(&hz), "Border Zone at {hz} Hz");
        println!("WOZ: Border Zone (optimal bit timing 3.5 µs) plays at {:.3} µs cells", 16e6 / hz as f64);
    }
    // A 3.5" WOZ is refused cleanly; WOZ 1 parses with its records' bit counts.
    let mut h = [0u8; woz::HEADER_BYTES];
    h[..8].copy_from_slice(b"WOZ2\xFF\x0A\x0D\x0A");
    h[12..16].copy_from_slice(b"INFO");
    h[80..84].copy_from_slice(b"TMAP");
    h[248..252].copy_from_slice(b"TRKS");
    h[20] = 2;
    h[21] = 2;
    h[59] = 16;
    h[88..248].fill(0xFF);
    assert!(woz::Woz::parse(&h, &[0; 160], None, 100, MAX_TRACK_BLOCKS).is_err());
    // WOZ 1: 35 records, TMAP standard.
    let mut v1 = h;
    v1[..4].copy_from_slice(b"WOZ1");
    v1[20] = 1;
    v1[21] = 1;
    v1[252..256].copy_from_slice(&(35u32 * 6656).to_le_bytes());
    for q in 0..160usize {
        v1[88 + q] = if q % 4 == 2 || (q + 1) / 4 >= 35 { 0xFF } else { ((q + 1) / 4) as u8 };
    }
    let mut bits = [0u16; 160];
    for (j, b) in bits.iter_mut().enumerate().take(35) {
        *b = 50_000 + j as u16;
    }
    let blocks = (256 + 35 * 6656usize).div_ceil(512);
    let w = woz::Woz::parse(&v1, &bits, None, blocks, MAX_TRACK_BLOCKS).expect("WOZ 1");
    assert_eq!(w.version, 1);
    assert!(!w.writable());
    let trk = w.trk(w.track(4 * 34)).unwrap();
    assert_eq!((trk.start, trk.bits), (34 * 13, 50_034));
    // The loader finds each record's bit count in block 13j + 13 at byte 248.
    for j in 0..35usize {
        assert_eq!(woz::Woz::v1_bit_count_at(13 * j + 13), Some((j, 248)));
        let at = 256 + j * 6656 + 6648;
        assert_eq!((at / 512, at % 512), (13 * j + 13, 248));
    }
    assert_eq!(woz::Woz::v1_bit_count_at(14), None);
    assert!(woz::Woz::parse(&v1, &bits, None, blocks - 1, MAX_TRACK_BLOCKS).is_err(), "last record past the end");
    // An 800K WOZ (The Oregon Trail): refused as 3.5".
    {
        let (_, f, _) = disks::oregon();
        let h: [u8; woz::HEADER_BYTES] = f[..woz::HEADER_BYTES].try_into().unwrap();
        let e = woz::Woz::parse(&h, &[0; 160], None, f.len().div_ceil(512), MAX_TRACK_BLOCKS).err();
        assert_eq!(e, Some("WOZ: 3.5\" disk (use mode = apple35)"));
    }
    println!("WOZ: 3.5\" refused, WOZ 1 records and bit counts located, short file refused");
}

/// A sector image's track as the firmware holds it while loaded (`Cylinder`: cells at the
/// write cell, `a2::TRACK_CELLS`), with core 1's view of the head and core 0's delay in applying
/// captures, driven the way DOS 3.3's RWTS drives a Disk II.
/// `drive::A2_CAPTURE_TAIL_US` in cells.
const A2_TAIL: u64 = 36;

/// A capture core 1 is running: where the gate first opened, the cells so far, how many of them
/// are queued, and when the gate last closed.
struct Run {
    pos: usize,
    cells: Vec<bool>,
    queued: usize,
    close: u64,
}

struct Dos {
    /// 16 sectors (DOS 3.3) or 13 (DOS 3.2).
    enc: a2::Encoding,
    track: Stored,
    /// Absolute time in cells (3.911 µs, the cell the track plays at and the Apple II writes);
    /// the head is over cell `t % n`.
    t: u64,
    /// IWM framing of what's read.
    shifter: u8,
    /// Captures queued but not yet applied by core 0: (when applied, gate position, cells, the
    /// rest of the one before).
    pending: Vec<(u64, usize, Vec<bool>, bool)>,
    /// Most captures queued at once (core 1's queue holds 16).
    most_pending: usize,
    /// The capture core 1 is running (`run_a2`), if a gate is open or in its tail.
    run: Option<Run>,
    /// Core 0's delay from a gate closing to the track republished with it.
    latency: u64,
    /// The track is playing from this time on (core 0 reading and building it after a step).
    playing_from: u64,
    last: Option<writes::A2Write>,
    s: u64,
    retries: u32,
    /// Track number and sector data decoded after each capture, as `Cylinder::apply` does.
    tnum: u8,
    sectors: [Option<[u8; 256]>; 16],
}

impl Dos {
    fn new(latency: u64, enc: a2::Encoding) -> Self {
        Dos {
            enc,
            track: Stored { bytes: vec![], n: a2::TRACK_CELLS },
            t: 0,
            shifter: 0,
            pending: vec![],
            most_pending: 0,
            run: None,
            latency,
            playing_from: 0,
            last: None,
            s: 0x1234_5678,
            retries: 0,
            tnum: 0,
            sectors: [None; 16],
        }
    }

    /// Step to track `t` and load it from a blank image, playing `load` after the head gets
    /// there. DOS 3.3's SEEKABS for one track, in its DELAY units (~100 µs): the first half step
    /// on for ONTABLE[0] (1) and off for OFFTABLE[0] (112); the second on: the head is there,
    /// 11.3 ms in. Then ONTABLE[1] (48) and OFFTABLE[1] (44), and DELAY(0) (256) to settle.
    fn load(&mut self, t: u8, load: u64) {
        let ms = |x: f64| (x * 1000.0 / 3.9111) as u64;
        let (arrive, seek) = (ms(11.3), ms(11.3 + 4.8 + 4.4 + 25.6));
        self.flush();
        let zero = [0u8; 256];
        let mut buf = [0u32; MAX_TRACK_WORDS];
        let words = a2::build_track(self.enc, t, 254, |_| &zero, &mut buf);
        self.track.bytes = buf[..words].iter().flat_map(|w| w.to_be_bytes()).collect();
        self.playing_from = self.t + arrive + load;
        self.t += seek;
        self.last = None;
        self.tnum = t;
        self.sectors = [None; 16];
    }

    /// Apply captures core 0 would have applied by now, and finish core 1's once its tail is over.
    fn catch_up(&mut self) {
        if let Some(run) = &self.run
            && self.t >= run.close + A2_TAIL
        {
            self.end_run();
        }
        while let Some(k) = self.pending.iter().position(|p| p.0 <= self.t) {
            let (_, pos, cap, rest) = self.pending.remove(k);
            self.apply(pos, &cap, rest);
        }
    }

    /// Everything written applied, and the sectors decoded from the track (`Cylinder::settle`).
    fn flush(&mut self) {
        if self.run.is_some() {
            self.end_run();
        }
        while !self.pending.is_empty() {
            let (_, pos, cap, rest) = self.pending.remove(0);
            self.apply(pos, &cap, rest);
        }
        if self.track.bytes.is_empty() {
            return; // nothing loaded yet
        }
        let (tnum, enc) = (self.tnum, self.enc);
        let sectors = &mut self.sectors;
        writes::a2_fields(&self.track, a2::TRACK_CELLS + 4_000, enc, |a, kind, d| {
            if let Some((t, s)) = a
                && t == tnum
                && kind == enc
                && s < enc.sectors()
            {
                sectors[s as usize] = Some(*d);
            }
        });
    }

    fn apply(&mut self, pos: usize, cap: &[bool], rest: bool) {
        let last = self.last.take();
        self.last = writes::splice_a2(&mut self.track, pos, last, &Bools(cap.to_vec()), cap.len(), false, rest).ok();
    }

    /// Core 1 queues what the separator has pushed (whole words) of the capture it's running.
    fn queue(&mut self, at: u64) {
        let run = self.run.as_mut().unwrap();
        let to = run.cells.len() / 32 * 32;
        let cap = run.cells[run.queued..to].to_vec();
        self.pending.push((at + self.latency, run.pos, cap, run.queued > 0));
        run.queued = to;
        self.most_pending = self.most_pending.max(self.pending.len());
    }

    /// The tail is over: the last of the capture goes, and the separator stops.
    fn end_run(&mut self) {
        let run = self.run.as_mut().unwrap();
        run.cells.extend([false; A2_TAIL as usize]);
        let close = run.close;
        self.queue(close + A2_TAIL);
        self.run = None;
    }

    fn cell(&mut self) -> bool {
        self.catch_up();
        let b = self.t >= self.playing_from && self.track.bit((self.t % self.track.n as u64) as usize);
        self.t += 1;
        b
    }

    fn nibble(&mut self) -> u8 {
        loop {
            self.shifter = self.shifter << 1 | self.cell() as u8;
            if self.shifter & 0x80 != 0 {
                let n = self.shifter;
                self.shifter = 0;
                return n;
            }
        }
    }

    /// One write, gate opened now. Core 1 notes the head's word (good to half a word), captures
    /// on for 36 cells past the gate, and core 0 applies each capture `latency` after it's queued.
    /// A gate that opens again within those 36 cells carries on with the same capture: what's
    /// been pushed goes at once, and the rest follows on. A gate that opens before the track
    /// plays is captured from then on.
    fn write(&mut self, cells: &[bool]) {
        self.catch_up();
        let open = self.t.max(self.playing_from);
        let skip = (open - self.t) as usize;
        let n = self.track.n as u64;
        if skip < cells.len() {
            let pos = ((open % n) / 32 * 32 + 16) as usize;
            // Now and then the separator sees WR's first edge as a 1.
            let stray = rng(&mut self.s) % 4 == 0;
            let mut new = cells[skip..].to_vec();
            if let Some(run) = self.run.as_mut() {
                // Write mode was off since `close`: WR held still, so 0s.
                run.cells.extend(vec![false; (open - run.close) as usize]);
                if stray {
                    *run.cells.last_mut().unwrap() = true;
                }
                self.queue(open);
                let run = self.run.as_mut().unwrap();
                run.pos = pos;
                run.cells.extend(new);
                run.close = open + (cells.len() - skip) as u64;
            } else {
                if stray {
                    new.insert(0, true);
                    new.pop();
                }
                self.run = Some(Run { pos, cells: new, queued: 0, close: open + (cells.len() - skip) as u64 });
            }
        }
        self.t += cells.len() as u64;
        self.shifter = 0;
    }

    /// Sync nibbles: 10 cells (DOS 3.3), or 9 (DOS 3.2).
    fn sync(&self, cells: &mut Vec<bool>, n: usize) {
        for _ in 0..n {
            cells.extend([true; 8]);
            cells.extend(if self.enc == a2::Encoding::SixAndTwo { &[false, false][..] } else { &[false][..] });
        }
    }

    fn bytes(cells: &mut Vec<bool>, bytes: &[u8]) {
        for &b in bytes {
            cells.extend((0..8).rev().map(|k| b >> k & 1 != 0));
        }
    }

    /// RWTS WRITADR: `syncs` sync nibbles and an address field.
    fn write_address(&mut self, track: u8, sector: u8, syncs: usize) {
        let mut c = vec![];
        self.sync(&mut c, syncs);
        Self::bytes(&mut c, &[0xD5, 0xAA, self.enc.address_mark()]);
        Self::bytes(&mut c, &a2::address_field(254, track, sector));
        Self::bytes(&mut c, &[0xDE, 0xAA, 0xEB]);
        self.write(&c);
        self.t += 11; // write mode off and on again: ~45 cycles
    }

    /// RWTS WRITESEC: 5 syncs, the data field, one more `FF`.
    fn write_data(&mut self, data: &[u8; 256]) {
        let c = a2_write_cells_as(self.enc, data);
        self.write(&c);
        self.t += 11;
    }

    /// RWTS RDADDR: (track, sector) of the next address field, within 772 nibbles.
    fn read_address(&mut self) -> Option<(u8, u8)> {
        for _ in 0..772 {
            if self.nibble() != 0xD5 {
                continue;
            }
            if self.nibble() != 0xAA || self.nibble() != self.enc.address_mark() {
                continue;
            }
            let mut f = [0u8; 4];
            for v in &mut f {
                let (a, b) = (self.nibble(), self.nibble());
                *v = a2::decode44(a, b);
            }
            if f[0] ^ f[1] ^ f[2] != f[3] || self.nibble() != 0xDE || self.nibble() != 0xAA {
                return None;
            }
            return Some((f[1], f[2]));
        }
        None
    }

    /// RWTS READ16: the data field that follows, within 32 nibbles.
    fn read_data(&mut self) -> Option<[u8; 256]> {
        let mut tries = 32;
        loop {
            tries -= 1;
            if tries == 0 {
                return None;
            }
            if self.nibble() == 0xD5 && self.nibble() == 0xAA && self.nibble() == 0xAD {
                break;
            }
        }
        let mut v = [0u8; a2::DATA_NIBBLES_53];
        let v = &mut v[..self.enc.data_nibbles()];
        for n in v.iter_mut() {
            *n = self.nibble();
        }
        let d = match self.enc {
            a2::Encoding::SixAndTwo => a2::decode_data((&*v).try_into().unwrap())?,
            a2::Encoding::FiveAndThree => a2::decode_data53((&*v).try_into().unwrap())?,
        };
        (self.nibble() == 0xDE && self.nibble() == 0xAA).then_some(d)
    }

    /// RWTS FORMAT for one track (FORMATRK and its verify), `syncntr` adjusted as RWTS does.
    /// DOS 3.2's is modelled on it: its sectors numbered in its skew, sector 0 wanted right
    /// after the last one written.
    fn format_track(&mut self, track: u8, syncntr: &mut usize) -> bool {
        let zero = [0u8; 256];
        let (n, enc) = (self.enc.sectors(), self.enc);
        let last = enc.sector_at(n - 1);
        loop {
            // FORMATRK
            self.write_address(track, 0, 128);
            self.write_data(&zero);
            for k in 1..n {
                self.write_address(track, enc.sector_at(k), *syncntr);
                self.write_data(&zero);
            }
            // Verify: let some sync pass, then sector 0 must be next.
            self.t += *syncntr as u64 * 50 / 4;

            let mut readcntr = 48u8;
            let mut verified = [false; 16];
            let mut left = n;
            let mut reformat = false;
            let first = self.read_address();
            match first {
                Some((_, 0)) => {}
                Some(_) => {
                    *syncntr -= if *syncntr > 16 { 2 } else { 1 };
                    if *syncntr < 5 {
                        return false;
                    }
                    reformat = true;
                }
                None => reformat = true,
            }
            if !reformat {
                // RDNXTDAT with sector 0, then RDNXTADR...
                let mut sec = 0u8;
                loop {
                    let ok = self.read_data().is_some() && !verified[sec as usize];
                    if ok {
                        verified[sec as usize] = true;
                        left -= 1;
                        if left == 0 {
                            break;
                        }
                    } else {
                        readcntr -= 1;
                        if readcntr == 0 {
                            reformat = true;
                            break;
                        }
                    }
                    loop {
                        match self.read_address() {
                            Some((_, s)) => {
                                sec = s;
                                break;
                            }
                            None => {
                                readcntr -= 1;
                                if readcntr == 0 {
                                    reformat = true;
                                    break;
                                }
                            }
                        }
                    }
                    if reformat {
                        break;
                    }
                }
            }
            if !reformat {
                if track == 0 && *syncntr >= 16 {
                    *syncntr -= 2;
                }
                return true;
            }
            // REREADDR: reformat once sector 15 has been read.
            self.retries += 1;
            loop {
                match self.read_address() {
                    Some((_, s)) if s == last && self.read_data().is_some() => break,
                    _ => {
                        // NOTLAST (from BADREAD's 0 this wraps to 255 more tries, as on the 6502).
                        readcntr = readcntr.wrapping_sub(1);
                        if readcntr == 0 {
                            return false;
                        }
                    }
                }
            }
        }
    }

    /// After FORMATRK: find sector 0 and read it (48 tries).
    fn read_check(&mut self) -> bool {
        for _ in 0..47 {
            if let Some((_, 0)) = self.read_address()
                && self.read_data().is_some()
            {
                return true;
            }
        }
        false
    }

    /// RWTS write of sector `sec`: find its address field, write the data field at once.
    fn write_sector(&mut self, sec: u8, data: &[u8; 256]) -> bool {
        for _ in 0..48 {
            if let Some((_, s)) = self.read_address()
                && s == sec
            {
                self.t += 4; // DOS turns write mode on after the epilogue's AA
                self.write_data(data);
                return true;
            }
        }
        false
    }

    fn read_sector(&mut self, sec: u8) -> Option<[u8; 256]> {
        for _ in 0..48 {
            if let Some((_, s)) = self.read_address()
                && s == sec
            {
                return self.read_data();
            }
        }
        None
    }
}

/// DOS 3.3's INIT (RWTS FORMAT) against sector-image tracks held and
/// written as the firmware does, then sector writes on the formatted tracks.
fn check_dos_init() {
    let ms = |x: f64| (x * 1000.0 / 3.9111) as u64;
    // Core 0's delay applying a capture, and from the head arriving to the track playing (one
    // step in eight slower: an SD write stalling).
    for (latency, load, slow) in [(0.0, 5.0, 5.0), (3.0, 5.0, 5.0), (10.0, 5.0, 5.0), (3.0, 20.0, 20.0), (25.0, 30.0, 30.0), (3.0, 5.0, 60.0)] {
        let mut d = Dos::new(ms(latency), a2::Encoding::SixAndTwo);
        let mut syncntr = 40usize;
        let mut passes = vec![];
        for t in 0..35u8 {
            d.load(t, ms(if t % 8 == 5 { slow } else { load }));
            let before = d.retries;
            assert!(d.format_track(t, &mut syncntr), "latency {latency} ms, load {load} ms: I/O error on track {t}, sync {syncntr}");
            assert!(d.read_check(), "track {t}: sector 0 after the format");
            passes.push(d.retries - before);
            // What core 0 would save: all 16 sectors, zeros.
            d.flush();
            assert!(d.sectors.iter().all(|s| *s == Some([0; 256])), "track {t}: sectors decoded {:?}", d.sectors.iter().map(|s| s.is_some()).collect::<Vec<_>>());
            // Then DOS writes into the formatted track (VTOC, catalog) before stepping on.
            if t == 17 || t == 0 {
                for sec in [0u8, 15, 7, 8] {
                    let data = random_sector(&mut d.s);
                    assert!(d.write_sector(sec, &data), "track {t} sector {sec}: address not found");
                    d.flush();
                    assert_eq!(d.sectors[sec as usize], Some(data), "track {t} sector {sec} after a write");
                    assert_eq!(d.read_sector(sec), Some(data), "track {t} sector {sec} read back");
                }
                assert_eq!(d.sectors.iter().filter(|s| s.is_some()).count(), 16);
            }
        }
        assert!(d.most_pending <= 16, "{} captures queued at once", d.most_pending);
        println!(
            "DOS 3.3 INIT, core 0 {latency} ms behind, tracks playing {load} ms after a step ({slow} ms on 1 in 8): gap 3 down to {syncntr} syncs, reformats per track {:?}, up to {} captures queued",
            passes, d.most_pending
        );
    }
}

/// The Disk II playback program (`src/disk2.pio`) with its DMA, as `drive::Stream` and
/// `Player::play_a2` drive it: a revolution is exactly the track's cells, every cell 16 cycles
/// with a 1's pulse at cycles 12–15, from any cell a track is started at.
fn check_playback() {
    let source = std::fs::read_to_string("src/disk2.pio").unwrap();
    let prog = pio_play::parse(&source, "disk2");
    let mut s = 0x9E37_79B9u64;
    let mut runs = 0;
    for bits in [51_136u32, 50_999, 51_015, 49_865, 51_200, 33, 63, 64, 65, 95, 2_048 * 32 - 1] {
        let track: Vec<bool> = (0..bits).map(|_| rng(&mut s) % 3 == 0).collect();
        let words = bits as usize / 32 + 1;
        let mut buf = vec![0u32; words];
        for (i, &b) in track.iter().enumerate() {
            if b {
                buf[i / 32] |= 0x8000_0000 >> (i % 32);
            }
        }
        // Junk past the last cell, which must never play.
        let tail = bits as usize % 32;
        buf[words - 1] |= if tail == 0 { u32::MAX } else { u32::MAX >> tail };
        let starts = [0, 1, 31, 32, 33, bits / 2, bits.saturating_sub(33), bits - 1];
        for &cell in starts.iter().filter(|&&c| c < bits) {
            let mut p = pio_play::Player::new(&prog);
            p.play(&buf, bits, cell);
            let cells = (3 * bits as usize).min(200_000);
            for k in 0..cells {
                let bit = track[(cell as usize + k) % bits as usize];
                for c in 0..16 {
                    let rd = p.cycle();
                    assert!(!p.stalled(), "bits {bits} from {cell}: stalled at cell {k}");
                    assert_eq!(rd, !(bit && (12..16).contains(&c)), "bits {bits} from {cell}: cell {k} cycle {c}");
                }
            }
            runs += 1;
        }
    }
    println!("disk2.pio: {runs} starts, tracks of 33 to 65,535 cells play exactly their cells a revolution, 16 cycles each");
}

/// 5-and-3 (`a2::encode_data53`) 
fn check_codec53() {
    // 32 nibbles: top bit set, never two 0 bits in a row, not the marks' D5 and AA.
    let mut want = vec![];
    for b in 0x80..=0xFFu8 {
        if (0..7).all(|k| b >> k & 3 != 0) && b != 0xD5 && b != 0xAA {
            want.push(b);
        }
    }
    assert_eq!(want, a2::ENCODE53.to_vec());
    assert_eq!(a2::ENCODE53, disks::DISK_BYTES_53);
    for b in 0..=255u8 {
        assert_eq!(a2::decode5(b), a2::ENCODE53.iter().position(|&x| x == b).map(|v| v as u8));
    }
    let mut s = 0x5353u64;
    for round in 0..2_000 {
        let data = match round {
            0 => [0u8; 256],
            1 => [0xFF; 256],
            2 => core::array::from_fn(|i| i as u8),
            _ => random_sector(&mut s),
        };
        let mut nibbles = vec![];
        a2::encode_data53(&data, |n| nibbles.push(n));
        assert_eq!(nibbles.len(), a2::DATA_NIBBLES_53);
        assert_eq!(nibbles[..], disks::encode53(&data)[..], "encoder mismatch, round {round}");
        assert_eq!(a2_decode::decode53(&nibbles), Some(data), "decoder");
        let arr: [u8; 411] = nibbles.clone().try_into().unwrap();
        assert_eq!(a2::decode_data53(&arr), Some(data));
        // A changed nibble is caught (by the checksum, or as a non-5-and-3 nibble).
        let mut bad = arr;
        let k = (rng(&mut s) % 411) as usize;
        bad[k] = if rng(&mut s) % 4 == 0 {
            0x96 // a 6-and-2 nibble, not a 5-and-3 one
        } else {
            a2::ENCODE53[(a2::decode5(bad[k]).unwrap() as usize + 1 + (rng(&mut s) % 30) as usize) % 32]
        };
        assert_eq!(a2::decode_data53(&bad), None);
    }
    // DOS 3.2's FORMAT numbers sectors 10 apart: 0, 10, 7, 4, 1, …
    let skew: Vec<u8> = (0..13).map(|k| a2::Encoding::FiveAndThree.sector_at(k)).collect();
    assert_eq!(skew, disks::DOS32_SKEW.to_vec());
    let mut sorted = skew.clone();
    sorted.sort();
    assert_eq!(sorted, (0..13).collect::<Vec<_>>());
    assert!((0..16).all(|k| a2::Encoding::SixAndTwo.sector_at(k) == k));
    assert!((0..13).all(|p| a2::file_sector(a2::Order::Dos32, p) == p as usize));
    println!("5-and-3: table, damage caught; DOS 3.2 skew {skew:?}");
}

/// A `.d13` image's sector numbered `sec` of track `track`.
fn d13_sector(img: &[u8], track: u8, sec: u8) -> &[u8; 256] {
    img[(track as usize * 13 + sec as usize) * 256..][..256].try_into().unwrap()
}

/// 13-sector tracks as `Cylinder` builds a `.d13`'s: every sector decodes (independently) in its
/// place in DOS 3.2's skew, and nothing a 13-sector P6 PROM couldn't read (two 0s in a row).
fn check_d13_tracks() {
    let mut s = 0xD13u64;
    let enc = a2::Encoding::FiveAndThree;
    for tracks in [35usize, 40] {
        let img: Vec<u8> = (0..tracks * a2::TRACK_BYTES_13).map(|_| rng(&mut s) as u8).collect();
        assert_eq!(img.len(), [116_480, 133_120][(tracks == 40) as usize]);
        for t in 0..tracks as u8 {
            let mut buf = [0u32; MAX_TRACK_WORDS];
            let words = a2::build_track(enc, t, 254, |p| d13_sector(&img, t, a2::file_sector(a2::Order::Dos32, p) as u8), &mut buf);
            assert_eq!(words * 32, a2::TRACK_CELLS);
            let cells = buf_cells(&buf, a2::TRACK_CELLS);
            let found = decode_track13(&cells);
            assert_eq!(found.len(), 13, "track {t}");
            for (k, &(trk, sec, vol, d)) in found.iter().enumerate() {
                assert_eq!((trk, sec, vol), (t, disks::DOS32_SKEW[k], 254), "track {t}");
                assert_eq!(&d, d13_sector(&img, t, sec));
            }
            // No 16-sector fields, and never two 0 cells in a row, across the wrap too.
            assert!(decode_track(&cells).is_empty());
            let n = cells.len();
            assert!((0..n).all(|i| cells[i] || cells[(i + 1) % n]), "track {t}: 00 in the cells");
            let mut w = weak::WeakList::new();
            w.find_runs(&buf, a2::TRACK_CELLS, weak::A2_RUN);
            assert_eq!(w.len(), 0);
            // The firmware's own decoder finds them too, as 5-and-3, by default either way.
            for default in [a2::Encoding::SixAndTwo, enc] {
                let mut got = 0;
                writes::a2_fields(&Bools(cells.iter().chain(&cells[..4_000]).copied().collect()), n + 4_000, default, |a, kind, d| {
                    let (tt, sec) = a.expect("address field");
                    assert_eq!((tt, kind), (t, enc));
                    assert_eq!(d, d13_sector(&img, t, sec));
                    got |= 1u16 << sec;
                });
                assert_eq!(got, 0x1FFF, "track {t}");
            }
        }
    }
    // Fitted to other lengths (13-sector WOZ tracks being reformatted).
    let img: Vec<u8> = (0..a2::TRACK_BYTES_13).map(|_| rng(&mut s) as u8).collect();
    for cells in (47_000..56_000).step_by(41).chain([50_000, 51_024, 51_200]) {
        let mut buf = [0u32; MAX_TRACK_WORDS];
        a2::build_track_cells(enc, 0, 254, |p| d13_sector(&img, 0, p), cells, &mut buf).expect("fits");
        assert_eq!(decode_track13(&buf_cells(&buf, cells)).len(), 13, "{cells} cells");
    }
    assert!(a2::build_track_cells(enc, 0, 254, |p| d13_sector(&img, 0, p), 46_000, &mut [0; MAX_TRACK_WORDS]).is_none());
    println!("13-sector tracks: 35/40-track .d13, every sector decodes in DOS 3.2's skew, no 00 cells; fitted lengths 47,000-56,000");
}

/// DOS 3.2 writes through the separator: a sector write (no address field) is 5-and-3 by the
/// disk's kind, a format's by its `D5 AA B5` address fields; and a 16-sector one still decodes on
/// a 13-sector disk's default.
fn check_writes53() {
    let mut s = 0x3232u64;
    let enc = a2::Encoding::FiveAndThree;
    for (speed, jitter) in [(0.0, 2), (-0.02, 3), (0.02, 3), (0.0, 5)] {
        let data = random_sector(&mut s);
        let got = capture(&a2_write_cells_as(enc, &data), speed, jitter, rng(&mut s));
        let mut fields = vec![];
        writes::a2_fields(&Bools(got.clone()), got.len(), enc, |a, kind, d| fields.push((a, kind, *d)));
        assert_eq!(fields, vec![(None, enc, data)], "speed {speed} jitter {jitter}");
        // On a 16-sector disk's default it's no 6-and-2 field.
        let mut six = 0;
        writes::a2_fields(&Bools(got.clone()), got.len(), a2::Encoding::SixAndTwo, |_, _, _| six += 1);
        assert_eq!(six, 0);
    }
    let img: Vec<u8> = (0..a2::TRACK_BYTES_13).map(|_| rng(&mut s) as u8).collect();
    let mut buf = [0u32; MAX_TRACK_WORDS];
    a2::build_track(enc, 3, 254, |p| d13_sector(&img, 0, p), &mut buf);
    let got = capture(&buf_cells(&buf, a2::TRACK_CELLS), 0.01, 3, 77);
    let mut found = 0u16;
    writes::a2_fields(&Bools(got.clone()), got.len(), a2::Encoding::SixAndTwo, |a, kind, d| {
        let (t, sec) = a.expect("address field");
        assert_eq!((t, kind), (3, enc));
        assert_eq!(d, d13_sector(&img, 0, sec));
        found |= 1 << sec;
    });
    assert_eq!(found, 0x1FFF);
    // A 16-sector format captured, decoded on a 13-sector disk's default: its address fields win.
    let img16 = random_image(1, &mut s);
    a2::build_track(a2::Encoding::SixAndTwo, 0, 254, |p| image_sector(&img16, a2::Order::Dos, 0, p), &mut buf);
    let got = capture(&buf_cells(&buf, a2::TRACK_CELLS), 0.0, 2, 78);
    let mut n16 = 0;
    writes::a2_fields(&Bools(got.clone()), got.len(), enc, |a, kind, _| {
        assert!(a.is_some() && kind == a2::Encoding::SixAndTwo);
        n16 += 1;
    });
    assert_eq!(n16, 16);
    // A capture with a D5 AA B5 address field is a formatter's: copied at the head.
    let mut c = vec![];
    for _ in 0..20 {
        c.extend([true; 8]);
        c.push(false);
    }
    for b in [0xD5u8, 0xAA, 0xB5].into_iter().chain(a2::address_field(254, 5, 7)).chain([0xDE, 0xAA, 0xEB]) {
        c.extend((0..8).rev().map(|k| b >> k & 1 != 0));
    }
    let mut track = Stored { bytes: vec![0xFF; a2::TRACK_CELLS / 8], n: a2::TRACK_CELLS };
    let w = writes::splice_a2(&mut track, 1_000, None, &Bools(c.clone()), c.len(), false, false).expect("address write");
    assert_eq!(w.end, 1_000 + c.len() - c.iter().rev().position(|&b| b).unwrap());
    assert!((0..c.len()).all(|i| track.bit(1_000 + i) == c[i]));
    // A sector write whose gate edge the separator took for a 1, just ahead of the first sync:
    // with 9-cell syncs the framing wouldn't recover before the data mark, so `splice_a2` drops
    // it, and the sector lands in place. Every sector, held as a `.d13`'s track is.
    a2::build_track(enc, 3, 254, |p| d13_sector(&img, 0, p), &mut buf);
    let mut held = Stored { bytes: buf[..a2::TRACK_CELLS / 32].iter().flat_map(|w| w.to_be_bytes()).collect(), n: a2::TRACK_CELLS };
    let mut want: Vec<[u8; 256]> = (0..13).map(|k| *d13_sector(&img, 0, k)).collect();
    for sec in 0..13u8 {
        let nibs = disks::frame(&held.cells());
        let k = (3..nibs.len() - 12)
            .find(|&k| (nibs[k - 3].0, nibs[k - 2].0, nibs[k - 1].0) == (0xD5, 0xAA, 0xB5) && a2::decode44(nibs[k + 4].0, nibs[k + 5].0) == sec)
            .expect("address field");
        let pos = nibs[k + 10].1 + 30;
        let data = random_sector(&mut s);
        let mut cap = vec![false; 7];
        cap.push(true); // the gate's edge
        cap.extend(a2_write_cells_as(enc, &data));
        cap.extend([false; 40]);
        writes::splice_a2(&mut held, pos, None, &Bools(cap.clone()), cap.len(), false, false).expect("sector write");
        want[sec as usize] = data;
    }
    let mut after: Vec<(u8, [u8; 256])> = decode_track13(&held.cells()).iter().map(|f| (f.1, f.3)).collect();
    after.sort_by_key(|f| f.0);
    assert_eq!(after.len(), 13);
    assert!(after.iter().all(|(k, d)| *d == want[*k as usize]), "sector writes with a gate edge");
    println!("DOS 3.2 writes: sector writes decode at ±2% and ±5 cycles as 5-and-3, formats by their D5 AA B5 fields, a D5 AA B5 write copied at the head, a gate edge before 9-cell sync dropped");
}

/// DOS 3.2's INIT (its FORMAT modelled on DOS 3.3's RWTS: field-at-a-time writes, the verify
/// that wants sector 0 after the last one written) on 13-sector tracks held as the firmware
/// holds a `.d13`'s, then sector writes into them.
fn check_dos32_init() {
    let ms = |x: f64| (x * 1000.0 / 3.9111) as u64;
    for (latency, load) in [(0.0, 5.0), (3.0, 5.0), (10.0, 20.0), (25.0, 30.0)] {
        let mut d = Dos::new(ms(latency), a2::Encoding::FiveAndThree);
        let mut syncntr = 40usize;
        let mut passes = vec![];
        for t in 0..35u8 {
            d.load(t, ms(load));
            let before = d.retries;
            assert!(d.format_track(t, &mut syncntr), "latency {latency} ms: I/O error on track {t}, sync {syncntr}");
            assert!(d.read_check(), "track {t}: sector 0 after the format");
            passes.push(d.retries - before);
            d.flush();
            assert!(d.sectors[..13].iter().all(|s| *s == Some([0; 256])), "track {t}");
            assert!(d.sectors[13..].iter().all(|s| s.is_none()));
            if t == 17 || t == 0 {
                for sec in [0u8, 12, 3, 10] {
                    let data = random_sector(&mut d.s);
                    assert!(d.write_sector(sec, &data), "track {t} sector {sec}: address not found");
                    d.flush();
                    assert_eq!(d.sectors[sec as usize], Some(data), "track {t} sector {sec} after a write");
                    assert_eq!(d.read_sector(sec), Some(data), "track {t} sector {sec} read back");
                }
            }
        }
        println!("DOS 3.2 INIT, core 0 {latency} ms behind, tracks playing {load} ms after a step: gap down to {syncntr} syncs, reformats per track {passes:?}");
    }
}

/// A 13-sector WOZ: tracks rotated into a gap (`moof::find_rotation` knows `D5 AA B5`), every
/// sector found by `writes::a2_fields` as by the independent decoder, the catalog chain on track
/// 17, sector writes spliced (also over `damaged`'s data field, whose mark is broken), a format
/// re-encoded as 13 sectors.
fn check_woz13(name: &str, file: &[u8], img: &[u8], damaged: Option<(u8, u8)>) {
    let h: [u8; woz::HEADER_BYTES] = file[..woz::HEADER_BYTES].try_into().unwrap();
    let w = woz::Woz::parse(&h, &[0; woz::QUARTERS], None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    let mut s = 0x1313u64;
    let mut lengths = (usize::MAX, 0);
    for t in 0..35u8 {
        let trk = w.trk(w.track(t * 4)).unwrap();
        let start = trk.start as usize * BLOCK;
        let mut stored = Stored { bytes: file[start..start + trk.blocks as usize * BLOCK].to_vec(), n: trk.bits as usize };
        let n = stored.n;
        lengths = (lengths.0.min(n), lengths.1.max(n));
        let bad = damaged.filter(|d| d.0 == t).map(|d| d.1);
        let whole = 13 - bad.is_some() as usize;
        // The loop wraps between a data field and the next address field: one pass from the
        // rotation, not looping, holds every sector whole.
        let rot = moof::find_rotation(|i| stored.bytes[i], trk.bits) as usize;
        let mut once: Vec<bool> = (0..n).map(|i| stored.bit(rot + i)).collect();
        once.extend(vec![false; n]);
        let found = decode_track13(&once);
        assert_eq!(found.len(), whole, "{name} track {t}: sectors whole from the rotation");
        for &(trk_n, sec, _, d) in &found {
            assert_eq!(trk_n, t);
            assert_ne!(Some(sec), bad);
            assert_eq!(&d, d13_sector(img, t, sec));
        }
        // The firmware's decoder (a format re-encoded, `a2_reformat`) finds the same, whatever its
        // default.
        for default in [a2::Encoding::SixAndTwo, a2::Encoding::FiveAndThree] {
            let mut by_fw = 0u16;
            writes::a2_fields(&Bools(once[..n].to_vec()), n, default, |a, kind, d| {
                let (trk_n, sec) = a.expect("address field");
                assert_eq!((trk_n, kind), (t, a2::Encoding::FiveAndThree));
                assert_eq!(d, d13_sector(img, t, sec), "{name} track {t} sector {sec}: a2_fields");
                by_fw |= 1 << sec;
            });
            assert_eq!(by_fw.count_ones() as usize, whole, "{name} track {t}: a2_fields");
        }
        if t == 17 {
            // VTOC: catalog at 17/12, DOS 3.2's 13 sectors a track; the catalog links down.
            let v = d13_sector(img, 17, 0);
            assert_eq!((v[1], v[2], v[3], v[0x35]), (17, 12, 2, 13));
            let sec = |k: u8| found.iter().find(|f| f.1 == k).unwrap().3;
            let mut at = 12;
            while at > 1 {
                let c = sec(at);
                assert_eq!((c[1], c[2]), (17, at - 1));
                at -= 1;
            }
        }
        if ![0u8, 17, 34].contains(&t) && bad.is_none() {
            continue;
        }
        // Rewrite each sector, the gate opening after its address field.
        let mut want: Vec<[u8; 256]> = (0..13).map(|k| *d13_sector(img, t, k)).collect();
        for sec in 0..13u8 {
            let data = random_sector(&mut s);
            // Twice around: a track may start anywhere, even in an address field.
            let nibs = disks::frame(&[stored.cells(), stored.cells()].concat());
            let k = (3..nibs.len() - 12)
                .find(|&k| (nibs[k - 3].0, nibs[k - 2].0, nibs[k - 1].0) == (0xD5, 0xAA, 0xB5) && a2::decode44(nibs[k + 4].0, nibs[k + 5].0) == sec)
                .expect("address field");
            let pos = nibs[k + 10].1 % n;
            let got = capture(&a2_write_cells_as(a2::Encoding::FiveAndThree, &data), 0.0, 2, rng(&mut s));
            writes::splice_gcr(&mut stored, pos + 20, &Bools(got.clone()), got.len()).expect("splice");
            want[sec as usize] = data;
        }
        let mut after: Vec<(u8, [u8; 256])> = decode_track13(&stored.cells()).iter().map(|f| (f.1, f.3)).collect();
        after.sort_by_key(|f| f.0);
        assert_eq!(after.len(), 13, "{name} track {t}: every sector after the writes");
        assert!(after.iter().all(|(k, d)| *d == want[*k as usize]), "{name} track {t}: rewritten sectors");
        // A format fitted back into the stored length, as `a2_reformat` does for 13 sectors.
        let fresh: Vec<u8> = (0..a2::TRACK_BYTES_13).map(|_| rng(&mut s) as u8).collect();
        let mut buf = [0u32; MAX_TRACK_WORDS];
        a2::build_track_cells(a2::Encoding::FiveAndThree, t, 254, |p| d13_sector(&fresh, 0, p), n, &mut buf).expect("fits");
        writes::copy_track(&mut stored, &buf);
        let found = decode_track13(&stored.cells());
        assert_eq!(found.len(), 13);
        assert!(found.iter().all(|f| f.0 == t && &f.3 == d13_sector(&fresh, 0, f.1)));
    }
    let also = damaged.map_or(String::new(), |(t, sec)| format!(", track {t} (sector {sec}'s data mark broken)"));
    println!("{name}: tracks of {}-{} cells wrap in a gap, a2_fields finds every sector, catalog chain on track 17; tracks 0, 17, 34{also} rewritten by splice and reformatted to their length", lengths.0, lengths.1);
}

fn main() {
    check_playback();
    check_codec();
    check_codec53();
    check_d13_tracks();
    check_stepper();
    check_sector_tracks();
    check_nib();
    check_writes();
    check_dos_init();
    check_writes53();
    check_dos32_init();
    let img = disks::dos32_image();
    check_woz13("13-sector WOZ", &disks::woz525_13(&img, 51_000), &img, None);
    check_woz13("DOS 3.2 WOZ as real disks read", &disks::dos32_woz(&img, Some((2, 12))), &img, Some((2, 12)));
    check_blank_woz("DOS 3.3 WOZ", &disks::woz525(&disks::dos33_image(), false, 51_200, 32));
    check_blank_woz("ProDOS WOZ", &disks::woz525(&disks::prodos_image(), true, 51_000, 32));
    check_woz_writes();
    check_other_woz();
    println!("a2_check: all passed");
}
