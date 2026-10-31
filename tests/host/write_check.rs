//! Host-side check of the write path: `src/capture.pio` (run by the `pio_sim` interpreter
//! against simulated Mac write waveforms), `src/writes.rs` (decoding and MOOF splicing), and
//! MFM MOOF reading. The images are made at test time (`disks.rs`).
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/write_check.rs -o target/write_check && target/write_check

#[path = "../../src/a2.rs"]
mod a2;
mod disks;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/mfm.rs"]
mod mfm;
#[path = "../../src/moof.rs"]
mod moof;
mod pio_sim;
#[path = "../../src/track.rs"]
mod track;
#[path = "../../src/writes.rs"]
mod writes;

use track::*;
use writes::{CellTrack, Cells};

struct Capture(Vec<bool>);

impl Cells for Capture {
    fn bit(&self, i: usize) -> bool {
        self.0[i]
    }
}

/// A MOOF-style stored track: `n` cells, MSB first.
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

fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

fn bytes_to_cells(bytes: &[u8]) -> Vec<bool> {
    bytes.iter().flat_map(|&b| (0..8).rev().map(move |k| b >> k & 1 != 0)).collect()
}

/// What the Mac puts on WR for `cells`, run through a capture program. GCR: NRZI, a level change
/// per 1. MFM: a 250 ns low pulse per 1. `speed` scales the cell, `jitter` (PIO cycles) moves
/// each edge. Returns the captured cells.
fn capture(program: &str, cells: &[bool], mfm: bool, speed: f64, jitter: u64, seed: u64) -> Vec<bool> {
    let src = std::fs::read_to_string("src/capture.pio").unwrap();
    let prog = pio_sim::parse(&src, program);
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
    let pulse = 8; // 250 ns at 32 cycles per µs
    let pin = |t: u64| {
        let k = edges.partition_point(|&e| e <= t);
        if mfm { !(k > 0 && t < edges[k - 1] + pulse) } else { k % 2 == 0 }
    };
    // Run past the end like the firmware does (80 µs of 0s), so the last cells get pushed.
    let total = (lead + cells.len() as f64 * cell) as u64 + 64 * 32;
    pio_sim::run(&prog, total, pin)
}

/// Position of `needle` in `hay`.
fn find(hay: &[bool], needle: &[bool]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

fn check_separators() {
    let mut s = 0x1234u64;
    for (program, mfm) in [("gcr_write", false), ("mfm_write", true)] {
        let mut bits = vec![];
        for _ in 0..40 {
            bits.extend([true; 8]);
            bits.extend([false; 2]);
        }
        while bits.len() < 6_000 {
            let zeros = if mfm { 1 + rng(&mut s) % 3 } else { rng(&mut s) % 3 };
            bits.extend(std::iter::repeat_n(false, zeros as usize));
            bits.push(true);
        }
        // Realistic envelope: crystal clocks on both sides (speed error far below 1%), edges on
        // FCLK boundaries, MFM precompensation ±2 FCLK ≈ ±4 cycles. Test ±3% with ±3 cycles,
        // and ±5 cycles at nominal speed.
        let cases = [(-0.03, 3), (0.0, 3), (0.03, 3), (0.0, 5), (-0.01, 4), (0.01, 4)];
        for (speed, jitter) in cases {
            let got = capture(program, &bits, mfm, speed, jitter, rng(&mut s));
            let probe = &bits[400..800];
            let at = find(&got, probe).unwrap_or_else(|| panic!("{program} {speed} ±{jitter}: no alignment"));
            let start = at - 400;
            let n = bits.len() - 400;
            assert_eq!(&got[start + 400..start + n], &bits[400..n], "{program} {speed} ±{jitter}");
        }
    }
    println!("separators: GCR and MFM capture exactly within ±3% speed and ±3–5 cycle jitter");
}

/// The cells of a GCR data-field write, as the ROM's `WrData` sends them: one sync group,
/// `D5 AA AD`, sector nibble, 703 data/checksum nibbles, `DE AA FF FF`.
fn gcr_write_cells(sector: u8, data: &[u8; 512]) -> Vec<bool> {
    let mut nib = vec![0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF, 0xD5, 0xAA, 0xAD];
    nib.push(gcr::address_field(0, sector, 0, 0)[1]); // sector nibble = ENCODE[sector]
    gcr::encode_sector(&[0; 12], data, |n| nib.push(n));
    nib.extend([0xDE, 0xAA, 0xFF, 0xFF]);
    bytes_to_cells(&nib)
}

/// MFM cells for bytes, marks for the `A1`s flagged in `marks`.
fn mfm_cells(bytes: &[(u8, bool)]) -> Vec<bool> {
    let mut prev = false;
    let mut out = vec![];
    for &(b, mark) in bytes {
        for k in (0..8).rev() {
            let d = b >> k & 1 != 0;
            let mut c = !(prev || d);
            if mark && k == 2 {
                c = false; // A1: drop the clock before data bit 2
            }
            out.push(c);
            out.push(d);
            prev = d;
        }
    }
    out
}

/// An MFM data-field write as `mWrData` sends it: 12 × `00`, `A1 A1 A1 FB`, data, CRC, 4 × `4E`.
fn mfm_write_cells(data: &[u8; 512]) -> Vec<bool> {
    let crc = mfm::crc16(mfm::crc16(0xFFFF, &[0xA1, 0xA1, 0xA1, 0xFB]), data);
    let mut bytes: Vec<(u8, bool)> = vec![(0, false); 12];
    bytes.extend([(0xA1, true); 3]);
    bytes.push((0xFB, false));
    bytes.extend(data.iter().map(|&b| (b, false)));
    bytes.extend(crc.to_be_bytes().map(|b| (b, false)));
    bytes.extend([(0x4E, false); 4]);
    mfm_cells(&bytes)
}

fn random_block(s: &mut u64) -> [u8; 512] {
    core::array::from_fn(|_| rng(s) as u8)
}

fn track_cells(buf: &TrackBuf, words: usize) -> Vec<bool> {
    (0..words * 32).map(|i| buf[i / 32] >> (31 - i % 32) & 1 != 0).collect()
}

fn check_raw_writes() {
    let mut s = 0xBEEFu64;
    // GCR sector writes, every sector number.
    for sector in 0..12u8 {
        let data = random_block(&mut s);
        let got = capture("gcr_write", &gcr_write_cells(sector, &data), false, 0.0, 2, rng(&mut s));
        let mut found = vec![];
        writes::gcr_data_fields(&Capture(got.clone()), got.len(), |sec, d| found.push((sec, d[12..].to_vec())));
        assert_eq!(found, vec![(sector, data.to_vec())], "GCR sector {sector}");
    }
    // GCR format: 200 sync groups, then a whole track of zeroed sectors.
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    let zero = [0u8; BLOCK];
    let words = gcr::build_track(20, 1, gcr::format(2, false), |_| &zero, &mut buf);
    let mut cells = bytes_to_cells(&[0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF].repeat(200));
    cells.extend(track_cells(&buf, words));
    let got = capture("gcr_write", &cells, false, 0.0, 2, 7);
    let mut sectors = vec![];
    writes::gcr_data_fields(&Capture(got.clone()), got.len(), |sec, d| {
        assert!(d.iter().all(|&b| b == 0));
        sectors.push(sec);
    });
    sectors.sort();
    assert_eq!(sectors, (0..11).collect::<Vec<_>>(), "GCR format, zone 1");

    // MFM sector writes; the sector number comes from the position.
    for _ in 1..=mfm::MAX_SECTORS {
        let data = random_block(&mut s);
        let got = capture("mfm_write", &mfm_write_cells(&data), true, 0.0, 3, rng(&mut s));
        let mut found = vec![];
        writes::mfm_fields(&Capture(got.clone()), got.len(), |f, _| match f {
            writes::MfmField::Data(d) => found.push(d.to_vec()),
            writes::MfmField::Id(_) => panic!("no ID in a sector write"),
        });
        assert_eq!(found, vec![data.to_vec()]);
    }
    for format in [mfm::Format::Hd, mfm::Format::Dd] {
        let spt = format.sectors() as usize;
        // Where the gate opens: about 22 bytes after the ID CRC, give or take a couple of words.
        let blank = [0u8; BLOCK];
        let words = mfm::build_track(format, 7, 0, |_| &blank, &mut buf);
        let t = track_cells(&buf, words);
        let mut ids = vec![];
        writes::mfm_fields(&Capture(t.clone()), t.len(), |f, at| {
            if let writes::MfmField::Id(id) = f {
                ids.push((id[2], at));
            }
        });
        assert_eq!(ids.len(), spt, "{format:?}");
        for &(sector, at) in &ids {
            let id_end = at + (4 + 6) * 16;
            for err in [-64i64, 0, 64] {
                let pos = (id_end as i64 + 22 * 16 + err) as usize;
                assert_eq!(mfm::sector_at(format, pos), sector, "{format:?} sector_at near sector {sector}, error {err}");
            }
        }
        // MFM format: a whole track captured, IDs and data.
        let got = capture("mfm_write", &t, true, 0.0, 3, 9);
        let (mut n_id, mut n_data) = (0, 0);
        writes::mfm_fields(&Capture(got.clone()), got.len(), |f, _| match f {
            writes::MfmField::Id(_) => n_id += 1,
            writes::MfmField::Data(_) => n_data += 1,
        });
        assert_eq!((n_id, n_data), (spt, spt), "{format:?} format capture");
        // The whole format fits the capture ring.
        assert!(got.len() / 32 < 8_192, "{format:?} format: {} words", got.len() / 32);
    }
    println!("raw writes: GCR sectors and format, MFM sectors, sector_at and format decode (1.44 MB and 720K)");
}

/// Every GCR sector in a stored track: (sector, data) over two revolutions, deduplicated.
fn gcr_track_sectors(t: &Stored) -> Vec<(u8, Vec<u8>)> {
    let mut v = vec![];
    writes::gcr_data_fields(t, 2 * t.n, |s, d| v.push((s, d[12..].to_vec())));
    v.sort();
    v.dedup();
    v
}

fn mfm_track_sectors(t: &Stored) -> Vec<(u8, Vec<u8>)> {
    let mut v = vec![];
    let mut sector = None;
    writes::mfm_fields(t, 2 * t.n, |f, _| match f {
        writes::MfmField::Id(id) => sector = Some(id[2]),
        writes::MfmField::Data(d) => {
            if let Some(s) = sector.take() {
                v.push((s, d.to_vec()));
            }
        }
    });
    v.sort();
    v.dedup();
    v
}

fn load_stored(file: &[u8], m: &moof::Moof, track: u8, side: u8) -> Stored {
    let (_, trk) = m.track(track, side).unwrap();
    let start = trk.start as usize * BLOCK;
    Stored { bytes: file[start..start + trk.blocks as usize * BLOCK].to_vec(), n: trk.bits as usize }
}

fn check_moof_writes() {
    let mut s = 0xC0FFEEu64;

    // GCR: rewrite one sector of a Pascal track in place.
    let file = disks::moof(&disks::pascal());
    let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    for track in [0u8, 5, 40, 79] {
        let mut t = load_stored(&file, &m, track, 0);
        let mut before = gcr_track_sectors(&t);
        // Position under the head: just after each sector's address field, as when the ROM's
        // RdAddr returns, with a couple of words of error either way.
        let mut headers = vec![];
        let mut recent = 0u32;
        let n = t.n;
        let bits = |i: usize| t.bit(i);
        let mut shifter = 0u8;
        for i in 0..n {
            shifter = shifter << 1 | bits(i) as u8;
            if shifter & 0x80 != 0 {
                recent = recent << 8 | u32::from(shifter);
                shifter = 0;
                if recent & 0xFF_FFFF == 0xD5_AA96 {
                    headers.push(i + 1); // end of the mark
                }
            }
        }
        for (k, &mark_end) in headers.iter().enumerate().take(3) {
            let pos = mark_end + 7 * 8 + [-48i64, 0, 48][k % 3] as usize;
            // Which sector is that header? Decode the next address field nibble.
            let mut snapshot = Stored { bytes: t.bytes.clone(), n };
            let sector = {
                let mut nibbles = vec![];
                let mut sh = 0u8;
                for i in mark_end..mark_end + 16 {
                    sh = sh << 1 | snapshot.bit(i) as u8;
                    if sh & 0x80 != 0 {
                        nibbles.push(sh);
                        sh = 0;
                    }
                }
                gcr::decode6(nibbles[1]).unwrap()
            };
            let data = random_block(&mut s);
            let got = capture("gcr_write", &gcr_write_cells(sector, &data), false, 0.0, 2, rng(&mut s));
            writes::splice_gcr(&mut snapshot, pos % n, &Capture(got.clone()), got.len()).unwrap();
            let after = gcr_track_sectors(&snapshot);
            let want: Vec<_> = before
                .iter()
                .map(|(sec, d)| (*sec, if *sec == sector { data.to_vec() } else { d.clone() }))
                .collect();
            let mut want_all = want.clone();
            if !want.iter().any(|(sec, _)| *sec == sector) {
                want_all.push((sector, data.to_vec())); // was undecodable before the write
                want_all.sort();
            }
            if after != want_all {
                let summary = |v: &[(u8, Vec<u8>)]| v.iter().map(|(sec, d)| format!("{sec}:{:02X}", d[0])).collect::<Vec<_>>().join(" ");
                panic!(
                    "Pascal track {track} sector {sector} pos {pos} (mark end {mark_end}):\n before {}\n after  {}\n want   {}",
                    summary(&before), summary(&after), summary(&want_all)
                );
            }
            before = after;
            t = snapshot;
        }
    }
    println!("MOOF GCR splice: rewritten sectors decode, the rest unchanged");

    // MFM MOOF: reading. A 1.44 MB MOOF (192,992 half-cells a track, as `Install.moof`) holds
    // its image.
    let img = disks::random_bytes(1_474_560, 0x1440);
    let file = disks::moof(&disks::mfm_disk(&img));
    let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    assert!(m.mfm && m.sides == 2);
    let mut c = moof::Crc32::new();
    c.update(&file[12..]);
    assert_eq!(c.finish(), m.stored_crc);
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    let (mut matched, mut total) = (0, 0);
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let (_, trk) = m.track(track, side).unwrap();
            let data = &file[trk.start as usize * BLOCK..][..trk.blocks as usize * BLOCK];
            let words = moof::load_track(|i| data[i], trk.bits, 0, moof::Pad::Mfm, &mut buf);
            let cells = Capture(track_cells(&buf, words));
            let mut sector = None;
            let mut seen = 0;
            writes::mfm_fields(&cells, cells.0.len(), |f, _| match f {
                writes::MfmField::Id(id) => {
                    assert_eq!((id[0], id[1]), (track, side));
                    sector = Some(id[2]);
                }
                writes::MfmField::Data(d) => {
                    let sec = sector.take().expect("data after ID") as usize;
                    let blk = mfm::Format::Hd.first_block(track, side) + sec - 1;
                    total += 1;
                    matched += (&img[blk * BLOCK..][..BLOCK] == &d[..]) as usize;
                    seen += 1;
                }
            });
            assert_eq!(seen, 18, "1.44 MB MOOF track {track} side {side}");
        }
    }
    assert_eq!(matched, total, "sectors equal to the image");
    println!("1.44 MB MOOF: all {total} MFM sectors decode from the played loop, identical to the image");

    // MFM MOOF: rewrite sectors in place.
    for (track, side) in [(0u8, 0u8), (3, 1), (79, 1)] {
        let mut t = load_stored(&file, &m, track, side);
        let before = mfm_track_sectors(&t);
        assert_eq!(before.len(), 18);
        let mut ids = vec![];
        writes::mfm_fields(&t, t.n, |f, at| {
            if let writes::MfmField::Id(id) = f {
                ids.push((id[2], at));
            }
        });
        for &(sector, at) in ids.iter().step_by(5) {
            let pos = at + 10 * 16 + 22 * 16;
            let data = random_block(&mut s);
            let got = capture("mfm_write", &mfm_write_cells(&data), true, 0.0, 3, rng(&mut s));
            let pos = pos % t.n;
            writes::splice_mfm(&mut t, pos, &Capture(got.clone()), got.len()).unwrap();
            let after = mfm_track_sectors(&t);
            for (sec, d) in &after {
                if *sec == sector {
                    assert_eq!(d, &data.to_vec(), "1.44 MB MOOF track {track} side {side} sector {sector}");
                }
            }
            assert_eq!(after.len(), 18);
        }
        // The untouched sectors are as before.
        let after = mfm_track_sectors(&t);
        let changed = before.iter().zip(&after).filter(|(a, b)| a != b).count();
        assert_eq!(changed, ids.iter().step_by(5).count(), "only the rewritten sectors change");
    }
    println!("MOOF MFM splice: rewritten sectors decode, the rest unchanged");

    // Format: decode the captured sectors and re-encode them to the stored track's length.
    let fresh = random_block(&mut s);
    let words = mfm::build_track(mfm::Format::Hd, 10, 0, |_| &fresh, &mut buf);
    let mut cells = track_cells(&buf, words);
    cells.extend(track_cells(&buf, words)[..20_000].to_vec());
    let got = capture("mfm_write", &cells, true, 0.0, 3, 11);
    let mut sectors = vec![[0u8; BLOCK]; 18];
    let mut sector = None;
    writes::mfm_fields(&Capture(got.clone()), got.len(), |f, _| match f {
        writes::MfmField::Id(id) => sector = Some(id[2]),
        writes::MfmField::Data(d) => sectors[sector.take().unwrap() as usize - 1] = *d,
    });
    let mut t = load_stored(&file, &m, 10, 0);
    mfm::build_track_cells(mfm::Format::Hd, 10, 0, |s| &sectors[s as usize - 1], t.n, &mut buf).expect("fits");
    writes::copy_track(&mut t, &buf);
    let after = mfm_track_sectors(&t);
    assert_eq!(after.len(), 18);
    assert!(after.iter().all(|(_, d)| d == &fresh.to_vec()));

    let pascal = disks::moof(&disks::pascal());
    let pm = moof::Moof::parse(pascal[..moof::HEADER_BYTES].try_into().unwrap(), None, pascal.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    for track in [0u8, 79] {
        let mut t = load_stored(&pascal, &pm, track, 0);
        let zero = [0u8; BLOCK];
        gcr::build_track_cells(track, 0, gcr::format(1, false), |_| &zero, t.n, &mut buf).expect("fits");
        writes::copy_track(&mut t, &buf);
        let after = gcr_track_sectors(&t);
        assert_eq!(after.len(), gcr::sectors_per_track(track) as usize, "Pascal track {track} reformatted");
        assert!(after.iter().all(|(_, d)| d.iter().all(|&b| b == 0)));
    }
    println!("MOOF format: sectors re-encoded to the stored length (MFM 192,992 cells, GCR {} and {})", pm.track(0, 0).unwrap().1.bits, pm.track(79, 0).unwrap().1.bits);

    // An Apple II 3.5" WOZ (read as a MOOF) reformatted by a IIgs: re-encoded at its 4:1
    // interleave, address fields saying so.
    let (_, woz, _) = disks::oregon();
    let wm = moof::Moof::parse(woz[..moof::HEADER_BYTES].try_into().unwrap(), None, woz.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).unwrap();
    assert!(wm.woz);
    for (track, side) in [(0u8, 0u8), (40, 1), (79, 1)] {
        let mut t = load_stored(&woz, &wm, track, side);
        let zero = [0u8; BLOCK];
        gcr::build_track_cells(track, side, gcr::format(2, true), |_| &zero, t.n, &mut buf).expect("fits");
        writes::copy_track(&mut t, &buf);
        let after = gcr_track_sectors(&t);
        assert_eq!(after.len(), gcr::sectors_per_track(track) as usize, "WOZ track {track} side {side} reformatted");
        assert!(after.iter().all(|(_, d)| d.iter().all(|&b| b == 0)));
        // The address fields: D5 AA 96, then track, sector, side, format $24.
        let (mut sh, mut recent, mut formats) = (0u8, 0u64, vec![]);
        for i in 0..t.n {
            sh = sh << 1 | t.bit(i) as u8;
            if sh & 0x80 != 0 {
                recent = recent << 8 | u64::from(sh);
                sh = 0;
                if recent >> 32 & 0xFF_FFFF == 0xD5_AA96 {
                    formats.push(gcr::decode6(recent as u8).unwrap());
                }
            }
        }
        assert!(!formats.is_empty() && formats.iter().all(|&f| f == 0x24), "WOZ track {track}: format bytes {formats:?}");
    }
    println!("3.5\" WOZ format: re-encoded to the stored length at 4:1, format $24");
}

fn main() {
    check_separators();
    check_raw_writes();
    check_moof_writes();
    println!("all write checks passed");
}
