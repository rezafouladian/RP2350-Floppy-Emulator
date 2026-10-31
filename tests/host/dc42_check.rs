//! Host-side check of DiskCopy 4.2 support (`src/dc42.rs`) against images of every size, with
//! and without tags, made at test time (`disks.rs`, with a checksum of its own): size detection,
//! header parsing, the checksum as `image::load` computes it block by block, every cylinder
//! rebuilt the way `Cylinder::read` does it (one extra block, shifted by the header), and writes
//! saved the way `Cylinder::flush_shifted` and `checksum_step` do them.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/dc42_check.rs -o target/dc42_check && target/dc42_check

#[path = "../../src/dc42.rs"]
mod dc42;
mod disks;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/mfm.rs"]
mod mfm;
#[path = "../../src/track.rs"]
mod track;

use track::{BLOCK, TRACKS};

/// (name, DiskCopy file, its sector data): 400K and 800K with tags and without, 720K, 1.44 MB.
fn images() -> Vec<(&'static str, Vec<u8>, Vec<u8>)> {
    let mut out = vec![];
    for (name, size, tags) in [
        ("400K", 409_600, false),
        ("400K with tags", 409_600, true),
        ("800K", 819_200, false),
        ("800K with tags", 819_200, true),
        ("720K", 737_280, false),
        ("1.44 MB", 1_474_560, false),
    ] {
        let raw = disks::random_bytes(size, size as u64 + 2 * tags as u64);
        let t = tags.then(|| disks::random_bytes(size / BLOCK * 12, 7));
        out.push((name, disks::diskcopy(&raw, t.as_deref(), name), raw));
    }
    out
}

fn main() {
    let mut failures = 0;
    let mut check = |ok: bool, what: String| {
        println!("{} {what}", if ok { "ok  " } else { "FAIL" });
        failures += usize::from(!ok);
    };

    for size in dc42::DATA_SIZES {
        check(dc42::data_size_for(size as u64).is_none(), format!("raw size {size} is not taken for DiskCopy"));
    }

    let images = images();
    for (file, bytes, raw) in &images {
        let size = bytes.len() as u64;
        let detected = dc42::data_size_for(size);
        check(detected.is_some(), format!("{file}: {size} bytes detected as DiskCopy ({detected:?})"));
        let header: &[u8; dc42::HEADER_BYTES] = bytes[..dc42::HEADER_BYTES].try_into().unwrap();
        let h = match dc42::Header::parse(header, size) {
            Ok(h) => h,
            Err(e) => {
                check(false, format!("{file}: header: {e}"));
                continue;
            }
        };
        check(Some(h.data_size) == detected, format!("{file}: header data size {}", h.data_size));

        // File blocks as the block map sees them; the last may be partial.
        let blocks: Vec<[u8; BLOCK]> = bytes
            .chunks(BLOCK)
            .map(|c| {
                let mut b = [0u8; BLOCK];
                b[..c.len()].copy_from_slice(c);
                b
            })
            .collect();

        // Checksum fed block by block, as in `load_candidate`.
        let mut sum = dc42::Checksum::new();
        for (i, c) in bytes.chunks(BLOCK).enumerate() {
            let at = i * BLOCK;
            let start = at.max(dc42::HEADER_BYTES);
            let end = (at + c.len()).min(dc42::HEADER_BYTES + h.data_size);
            if start < end {
                sum.update(&c[start - at..end - at]);
            }
        }
        check(sum.finish() == h.data_checksum, format!("{file}: checksum {:08X}", h.data_checksum));

        // Every cylinder, as `Cylinder::read` assembles it.
        let data = &bytes[dc42::HEADER_BYTES..dc42::HEADER_BYTES + h.data_size];
        let mut bad = 0;
        for track in 0..TRACKS {
            let (first, n) = match h.data_size {
                409_600 => (gcr::first_block(track, 1), gcr::sectors_per_track(track) as usize),
                819_200 => (gcr::first_block(track, 2), 2 * gcr::sectors_per_track(track) as usize),
                737_280 => (mfm::Format::Dd.first_block(track, 0), 2 * mfm::Format::Dd.sectors() as usize),
                _ => (mfm::Format::Hd.first_block(track, 0), 2 * mfm::Format::Hd.sectors() as usize),
            };
            let mut cyl: Vec<[u8; BLOCK]> = blocks[first..first + n + 1].to_vec();
            for i in 0..n {
                let (this, rest) = cyl.split_at_mut(i + 1);
                dc42::shift_sector(&mut this[i], &rest[0], dc42::HEADER_BYTES);
            }
            for (k, sector) in cyl[..n].iter().enumerate() {
                let at = (first + k) * BLOCK;
                if sector[..] != data[at..at + BLOCK] {
                    bad += 1;
                }
            }
        }
        check(bad == 0, format!("{file}: all 80 cylinders reassemble ({bad} bad sectors)"));

        check(data == &raw[..], format!("{file}: data matches the raw image"));
    }

    // Writes, saved as the firmware does, on a copy of each.
    for (file, bytes, _) in &images {
        let (ok, what) = check_writes(bytes);
        check(ok, format!("{file}: {what}"));
    }

    // A header whose format byte disagrees with its size is refused.
    let mut header = [0u8; dc42::HEADER_BYTES];
    header[0x40..0x44].copy_from_slice(&819_200u32.to_be_bytes());
    header[0x50] = 3;
    check(dc42::Header::parse(&header, (dc42::HEADER_BYTES + 819_200) as u64).is_err(), "800K data with format 3 is refused".into());
    header[0x40..0x44].copy_from_slice(&737_280u32.to_be_bytes());
    check(dc42::Header::parse(&header, (dc42::HEADER_BYTES + 737_280) as u64).is_err(), "720K data with format 3 is refused".into());
    header[0x50] = 2;
    check(dc42::Header::parse(&header, (dc42::HEADER_BYTES + 737_280) as u64).is_ok(), "720K data with format 2 is accepted".into());

    if failures > 0 {
        println!("{failures} FAILED");
        std::process::exit(1);
    }
    println!("all DiskCopy checks passed");
}

/// File block `i` of `file`, zero-padded past the end.
fn file_block(file: &[u8], i: usize) -> [u8; BLOCK] {
    let mut b = [0u8; BLOCK];
    if i * BLOCK < file.len() {
        let end = (i * BLOCK + BLOCK).min(file.len());
        b[..end - i * BLOCK].copy_from_slice(&file[i * BLOCK..end]);
    }
    b
}

fn put_block(file: &mut [u8], i: usize, b: &[u8; BLOCK]) {
    let end = (i * BLOCK + BLOCK).min(file.len());
    file[i * BLOCK..end].copy_from_slice(&b[..end - i * BLOCK]);
}

/// Write runs of new sectors into a copy of a DiskCopy image the way `Cylinder::flush_shifted`
/// saves them, then redo the checksum the way `checksum_step` does. Checks the data, that nothing
/// else changed, the checksum against a full recompute, the loader's header check, and the tag
/// checksum.
fn check_writes(orig: &[u8]) -> (bool, String) {
    let header: &[u8; dc42::HEADER_BYTES] = orig[..dc42::HEADER_BYTES].try_into().unwrap();
    let h = dc42::Header::parse(header, orig.len() as u64).unwrap();
    let size = h.data_size;
    let off = dc42::HEADER_BYTES;
    let mut file = orig.to_vec();
    let mut want = orig[off..off + size].to_vec();
    // (first sector of the cylinder, sectors in it) for every cylinder.
    let cylinders: Vec<(usize, usize)> = (0..TRACKS)
        .map(|t| match size {
            409_600 => (gcr::first_block(t, 1), gcr::sectors_per_track(t) as usize),
            819_200 => (gcr::first_block(t, 2), 2 * gcr::sectors_per_track(t) as usize),
            737_280 => (mfm::Format::Dd.first_block(t, 0), 18),
            _ => (mfm::Format::Hd.first_block(t, 0), 36),
        })
        .collect();
    // Dirty runs (cylinder, first, end): a whole first cylinder (its first block holds the
    // header), one sector, two runs on one cylinder, a whole last cylinder (its last block runs
    // into the tags or the end of the file).
    let last = cylinders[TRACKS as usize - 1].1;
    let runs = [(0usize, 0usize, cylinders[0].1), (5, 3, 4), (40, 0, 2), (40, 5, 9), (79, 0, last)];
    let mut seed = 0x1234_5678u32;
    for &(cyl, k0, k1) in &runs {
        let (first, _) = cylinders[cyl];
        let mut sectors = vec![[0u8; BLOCK]; k1];
        for s in &mut sectors[k0..k1] {
            for b in s.iter_mut() {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                *b = seed as u8;
            }
        }
        // As flush_shifted: file blocks first+k0 ..= first+k1, the two at the ends from the card.
        let count = k1 - k0 + 1;
        let mut stage = vec![[0u8; BLOCK]; count];
        stage[0] = file_block(&file, first + k0);
        stage[count - 1] = file_block(&file, first + k1);
        for j in k0..k1 {
            let (this, next) = stage[j - k0..].split_at_mut(1);
            dc42::place_sector(&mut this[0], &mut next[0], &sectors[j], off);
        }
        for (i, b) in stage.iter().enumerate() {
            put_block(&mut file, first + k0 + i, b);
        }
        for j in k0..k1 {
            want[(first + j) * BLOCK..][..BLOCK].copy_from_slice(&sectors[j]);
        }
    }
    if file[off..off + size] != want[..] {
        return (false, "written sectors don't read back".into());
    }
    if file[..off] != orig[..off] || file[off + size..] != orig[off + size..] {
        return (false, "header or tags changed".into());
    }
    // As checksum_step: 32 file blocks at a time, then into the header.
    let mut job = dc42::ChecksumJob::new(size);
    while let Some(next) = job.next_block() {
        let total = (off + size).div_ceil(BLOCK);
        for i in next..(next + 32).min(total) {
            job.feed(&file_block(&file, i));
        }
    }
    let sum = job.finish();
    let mut full = dc42::Checksum::new();
    full.update(&file[off..off + size]);
    if sum != full.finish() {
        return (false, format!("chunked checksum {sum:08X} != {:08X}", full.finish()));
    }
    file[dc42::DATA_CHECKSUM].copy_from_slice(&sum.to_be_bytes());
    let header: &[u8; dc42::HEADER_BYTES] = file[..off].try_into().unwrap();
    let after = dc42::Header::parse(header, file.len() as u64).expect("still a valid header");
    if after.data_checksum != full.finish() {
        return (false, "header checksum not updated".into());
    }
    // Tags: untouched, so DiskCopy's tag checksum (the first sector's 12 bytes left out) still holds.
    let tags = &file[off + size..];
    let tag_note = if tags.is_empty() {
        "no tags".to_string()
    } else {
        let mut t = dc42::Checksum::new();
        t.update(&tags[12..]);
        let stored = u32::from_be_bytes(file[0x4C..0x50].try_into().unwrap());
        if t.finish() != stored {
            return (false, format!("tag checksum {:08X} != stored {stored:08X}", t.finish()));
        }
        format!("{} tag bytes untouched, tag checksum still valid", tags.len())
    };
    let changed = runs.iter().map(|r| r.2 - r.1).sum::<usize>();
    (true, format!("{changed} sectors written as the firmware saves them, the rest unchanged, checksum {:08X} -> {sum:08X} (chunked = full), {tag_note}", h.data_checksum))
}
