//! Host-side check of weak bits (`src/weak.rs`): finding no-flux runs in a track buffer and
//! re-randomizing them, on hand-made buffers and on MOOFs with the traits of real ones (made at
//! test time, `disks.rs`), built the way `Cylinder::build` does (rotated, padded). Fuzzy
//! reversals from flux tracks are checked in `flux_check.rs`.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/weak_check.rs -o target/weak_check && target/weak_check

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
use weak::{Spot, WeakList};

const ENC: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

fn bit(buf: &[u32], i: usize) -> bool {
    buf[i / 32] >> (31 - i % 32) & 1 != 0
}

fn set(buf: &mut [u32], i: usize, v: bool) {
    let m = 0x8000_0000 >> (i % 32);
    if v { buf[i / 32] |= m } else { buf[i / 32] &= !m }
}

/// Buffers made from a cell pattern: runs found, and only their cells change.
fn check_basics() {
    // 1s every 2 cells, with zero runs of 7 (too short), 8, 40 (across words) and 20 at the end.
    let cells: usize = 300;
    let mut buf = vec![0u32; cells.div_ceil(32)];
    let mut ones = vec![];
    let mut i = 0;
    for gap in [1, 1, 7, 1, 8, 1, 40, 1, 1, 1] {
        set(&mut buf, i, true);
        ones.push(i);
        i += gap + 1;
    }
    let last_one = i - 2;
    let (min, keep) = weak::GCR_RUN;
    let mut list = WeakList::new();
    list.find_runs(&buf, last_one + 1 + 20, (min, keep));
    let mut want = vec![];
    for w in ones.windows(2) {
        let zeros = w[1] - w[0] - 1;
        if zeros >= min {
            want.push(Spot::Run { start: w[0] + 1 + keep, len: zeros - keep });
        }
    }
    want.push(Spot::Run { start: last_one + 1 + keep, len: 20 - keep });
    assert_eq!(list.iter().collect::<Vec<_>>(), want);

    // Randomizing changes run cells only, and does change them.
    let before = buf.clone();
    let mut rng = weak::Rng::new(7);
    let mut changed = vec![false; buf.len() * 32];
    for _ in 0..50 {
        list.randomize(&mut buf, &mut rng);
        for c in 0..buf.len() * 32 {
            if bit(&buf, c) != bit(&before, c) {
                changed[c] = true;
            }
        }
    }
    let inside = |c: usize| want.iter().any(|s| matches!(*s, Spot::Run { start, len } if (start..start + len).contains(&c)));
    for c in 0..buf.len() * 32 {
        assert!(!changed[c] || inside(c), "cell {c} changed outside a run");
    }
    assert!(want.iter().all(|s| matches!(*s, Spot::Run { start, len } if (start..start + len).any(|c| changed[c]))));

    // Fuzzy: one of the two cells, never both, never neither.
    let mut buf = vec![0u32; 2];
    set(&mut buf, 10, true);
    let mut stored = WeakList::new();
    stored.push_fuzzy(10, true);
    let mut list = WeakList::new();
    list.add_fuzzy(&stored, 64, 0, &buf);
    let mut seen = [false; 2];
    for _ in 0..50 {
        list.randomize(&mut buf, &mut rng);
        assert!(bit(&buf, 10) != bit(&buf, 11));
        seen[bit(&buf, 11) as usize] = true;
    }
    assert_eq!(seen, [true, true]);
    println!("WeakList: runs found (min {min}, first {keep} kept), only their cells change; fuzzy picks one of two cells");
}

/// Frame a GCR loop and return (sector, decodes?) for every address field that has a data field.
fn gcr_sectors(cells: &dyn Fn(usize) -> bool, n: usize) -> Vec<(u8, bool, usize)> {
    let mut nib = vec![];
    let mut sh = 0u8;
    for i in 0..2 * n {
        sh = sh << 1 | cells(i % n) as u8;
        if sh & 0x80 != 0 {
            nib.push((sh, i));
            sh = 0;
        }
    }
    let mut out = vec![];
    let mut sector = None;
    for k in 0..nib.len().saturating_sub(3) {
        match (nib[k].0, nib[k + 1].0, nib[k + 2].0) {
            (0xD5, 0xAA, 0x96) if k + 5 < nib.len() => sector = ENC.iter().position(|&e| e == nib[k + 4].0).map(|s| (s as u8, nib[k].1)),
            (0xD5, 0xAA, 0xAD) if k + 4 + 703 <= nib.len() => {
                if let Some((s, at)) = sector.take() {
                    let body: Vec<u8> = nib[k + 4..k + 4 + 703].iter().map(|x| x.0).collect();
                    out.push((s, gcr::decode_sector(body[..].try_into().unwrap()).is_some(), at % n));
                }
            }
            _ => {}
        }
    }
    out.sort_by_key(|x| x.0);
    out.dedup_by_key(|x| x.0);
    out
}

/// Every MOOF: the weak runs the firmware would find (only on the protection tracks), and that
/// randomizing them leaves every sector without one as readable as before.
fn check_images() {
    let files = [
        ("Macintosh Pascal", disks::moof(&disks::pascal())),
        ("Skyfox", disks::moof(&disks::skyfox())),
        ("Dollars and Sense", disks::moof(&disks::dollars())),
        ("SimCity", disks::moof(&disks::simcity())),
        ("MacWars", disks::moof(&disks::macwars())),
        ("The Ancient Art of War", disks::moof(&disks::aaow())),
        ("Oids", disks::oids().1),
        ("LabelWriter II", disks::labelwriter().1),
        ("The Oregon Trail", disks::oregon().1),
        ("1.44 MB", disks::moof(&disks::mfm_disk(&disks::random_bytes(1_474_560, 0x1440)))),
    ];
    let mut rng = weak::Rng::new(3);
    let (mut images, mut with_runs) = (0, 0);
    let mut found = vec![];
    for (name, file) in &files {
        let header: &[u8; moof::HEADER_BYTES] = file[..moof::HEADER_BYTES].try_into().unwrap();
        let chunk = moof::Moof::flux_block(header).map(|b| {
            let mut c = [0u8; BLOCK];
            let tail = &file[b * BLOCK..(b * BLOCK + BLOCK).min(file.len())];
            c[..tail.len()].copy_from_slice(tail);
            c
        });
        let m = moof::Moof::parse(header, chunk.as_ref(), file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).expect(name);
        images += 1;
        let mut report = vec![];
        for track in 0..TRACKS {
            for side in 0..2u8 {
                let Some((_, trk)) = m.track(track, side) else { continue };
                if trk.flux {
                    continue; // flux_check.rs
                }
                let data = &file[trk.start as usize * BLOCK..];
                let byte = |i: usize| data[i];
                let n = trk.bits as usize;
                let rot = if m.mfm { 0 } else { moof::find_rotation(byte, trk.bits) };
                let pad = if m.mfm { moof::Pad::Mfm } else { moof::Pad::Gcr };
                let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
                let words = moof::load_track(byte, trk.bits, rot, pad, &mut buf);
                let mut list = WeakList::new();
                list.find_runs(&buf[..], n, if m.mfm { weak::MFM_RUN } else { weak::GCR_RUN });
                if list.len() == 0 {
                    continue;
                }
                found.push((*name, track, side));
                let longest = list.iter().map(|s| if let Spot::Run { len, .. } = s { len } else { 0 }).max().unwrap();
                report.push(format!("t{track}s{side}: {} (longest {})", list.len(), longest + if m.mfm { weak::MFM_RUN.1 } else { weak::GCR_RUN.1 }));
                if m.mfm {
                    continue;
                }
                // Sectors before and after a few randomized passes. A sector whose data field holds a
                // weak run may change; every other one must read exactly as before.
                let before = gcr_sectors(&|i| bit(&buf[..], i), words * 32);
                let weak_at = |at: usize| {
                    list.iter().any(|s| matches!(s, Spot::Run { start, len } if start + len > at && start < at + 6_000))
                };
                for _ in 0..5 {
                    list.randomize(&mut buf[..], &mut rng);
                    let after = gcr_sectors(&|i| bit(&buf[..], i), words * 32);
                    for b in &before {
                        if b.1 && !weak_at(b.2) {
                            assert!(after.iter().any(|a| a.0 == b.0 && a.1), "{name} t{track}s{side}: sector {} lost", b.0);
                        }
                    }
                }
            }
        }
        if !report.is_empty() {
            with_runs += 1;
            println!("  {name}: weak runs on {} sides: {}", report.len(), report.join(", "));
        }
    }
    assert_eq!(found, disks::WEAK_SIDES, "weak runs only on the protection tracks");
    println!("{images} MOOFs, {with_runs} with weak runs, only on the protection tracks; randomizing them never lost a sector without one");
}

fn main() {
    check_basics();
    check_images();
    println!("all weak-bit checks passed");
}
