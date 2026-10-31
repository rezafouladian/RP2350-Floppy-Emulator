//! Turns a raw sector image into the other formats the emulator plays, using the firmware's own
//! encoders: a DiskCopy 4.2 image (any size) and a MOOF (400K/800K GCR, 1.44 MB MFM; MOOF has no
//! 720K type). Each MOOF is decoded again and checked against the raw image.
//!
//! Build and run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tools/make_images.rs -o target/make_images
//!   target/make_images "images/Blank/Blank 800K.dsk"
//! writes `Blank 800K.image` and `Blank 800K.moof` next to it.
//!
//! MOOF layout: header, INFO (not write-protected, creator
//! "rp2350-floppy make_images"), TMAP, TRKS, then each track side from block 3 at its exact cell
//! count: GCR as `gcr::build_track` lays it out (a real revolution's length), MFM a full 200 ms
//! revolution from the index.

#[path = "../src/dc42.rs"]
mod dc42;
#[path = "../src/gcr.rs"]
mod gcr;
#[path = "../src/mfm.rs"]
mod mfm;
#[path = "../src/moof.rs"]
mod moof;
#[path = "../src/track.rs"]
mod track;

use std::path::Path;
use track::{BLOCK, MAX_TRACK_BLOCKS, MAX_TRACK_WORDS, TRACKS};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("usage: make_images <raw image>...");
        std::process::exit(2);
    }
    for path in &args {
        let raw = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
        let path = Path::new(path);
        let name = path.file_stem().unwrap().to_string_lossy().into_owned();
        let dc = diskcopy(&raw, &name);
        std::fs::write(path.with_extension("image"), &dc).unwrap();
        print!("{name}: {} bytes -> .image ({} bytes)", raw.len(), dc.len());
        if let Some(m) = moof_image(&raw) {
            std::fs::write(path.with_extension("moof"), &m).unwrap();
            print!(", .moof ({} bytes, decodes back)", m.len());
        }
        println!();
    }
}

/// DiskCopy 4.2: 84-byte header, the sectors, no tags.
fn diskcopy(raw: &[u8], name: &str) -> Vec<u8> {
    let format = match raw.len() {
        409_600 => 0,
        819_200 => 1,
        737_280 => 2,
        1_474_560 => 3,
        n => panic!("{n} bytes isn't a floppy size"),
    };
    let mut h = vec![0u8; dc42::HEADER_BYTES];
    let name = &name.as_bytes()[..name.len().min(63)];
    h[0] = name.len() as u8;
    h[1..1 + name.len()].copy_from_slice(name);
    h[0x40..0x44].copy_from_slice(&(raw.len() as u32).to_be_bytes());
    let mut sum = dc42::Checksum::new();
    sum.update(raw);
    h[0x48..0x4C].copy_from_slice(&sum.finish().to_be_bytes());
    h[0x50] = format;
    h[0x51] = if format == 0 { 0x12 } else { 0x22 };
    h[0x52..0x54].copy_from_slice(&0x0100u16.to_be_bytes());
    let mut out = h;
    out.extend_from_slice(raw);
    // Check it the way the loader does.
    let header: &[u8; dc42::HEADER_BYTES] = out[..dc42::HEADER_BYTES].try_into().unwrap();
    let parsed = dc42::Header::parse(header, out.len() as u64).expect("valid DiskCopy header");
    assert_eq!(parsed.data_size, raw.len());
    out
}

/// A MOOF of the raw image, or `None` for 720K.
fn moof_image(raw: &[u8]) -> Option<Vec<u8>> {
    let (disk_type, sides, mfm) = match raw.len() {
        409_600 => (1u8, 1u8, false),
        819_200 => (2, 2, false),
        1_474_560 => (3, 2, true),
        _ => return None,
    };
    let mut out = vec![0u8; moof::HEADER_BYTES];
    out[..8].copy_from_slice(b"MOOF\xFF\x0A\x0D\x0A");
    out[12..16].copy_from_slice(b"INFO");
    out[16..20].copy_from_slice(&60u32.to_le_bytes());
    out[20] = 1; // INFO version
    out[21] = disk_type;
    out[22] = 0; // writable
    out[23] = 0; // not synchronized
    out[24] = if mfm { 8 } else { 16 }; // bit timing, 125 ns units
    let creator = b"rp2350-floppy make_images";
    out[25..57].fill(b' ');
    out[25..25 + creator.len()].copy_from_slice(creator);
    out[80..84].copy_from_slice(b"TMAP");
    out[84..88].copy_from_slice(&160u32.to_le_bytes());
    out[88..248].fill(0xFF);
    out[248..252].copy_from_slice(b"TRKS");
    let mut largest = 0u16;
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    for t in 0..TRACKS {
        for side in 0..sides {
            let bits = if mfm {
                let f = mfm::Format::Hd;
                let first = f.first_block(t, side);
                let words = mfm::build_track(f, t, side, |s| raw[(first + s as usize - 1) * BLOCK..][..BLOCK].try_into().unwrap(), &mut buf);
                words * 32
            } else {
                let spt = gcr::sectors_per_track(t) as usize;
                let first = gcr::first_block(t, sides) + side as usize * spt;
                gcr::build_track(t, side, gcr::format(sides, false), |s| raw[(first + s as usize) * BLOCK..][..BLOCK].try_into().unwrap(), &mut buf);
                spt * 6220 // the exact cell count, not word-padded
            };
            let bytes: Vec<u8> = buf.iter().flat_map(|w| w.to_be_bytes()).take(bits.div_ceil(8)).collect();
            let start = out.len() / BLOCK;
            let nblocks = bytes.len().div_ceil(BLOCK);
            out.extend(&bytes);
            out.resize((start + nblocks) * BLOCK, 0);
            let idx = t as usize * 2 + side as usize;
            out[88 + idx] = idx as u8;
            let r = 256 + idx * 8;
            out[r..r + 2].copy_from_slice(&(start as u16).to_le_bytes());
            out[r + 2..r + 4].copy_from_slice(&(nblocks as u16).to_le_bytes());
            out[r + 4..r + 8].copy_from_slice(&(bits as u32).to_le_bytes());
            largest = largest.max(nblocks as u16);
        }
    }
    out[58..60].copy_from_slice(&largest.to_le_bytes());
    let trks_size = (out.len() - 256) as u32;
    out[252..256].copy_from_slice(&trks_size.to_le_bytes());
    let mut c = moof::Crc32::new();
    c.update(&out[12..]);
    let crc = c.finish();
    out[8..12].copy_from_slice(&crc.to_le_bytes());

    // Parse it the way the loader does, and decode every sector back.
    let m = moof::Moof::parse(out[..moof::HEADER_BYTES].try_into().unwrap(), None, out.len() / BLOCK, MAX_TRACK_BLOCKS)
        .expect("valid MOOF");
    assert_eq!((m.sides, m.mfm), (sides, mfm));
    assert_eq!(decode(&out, &m, sides, mfm), raw, "MOOF decodes back to the raw image");
    Some(out)
}

/// Every sector of a MOOF made by `moof_image`, in raw image order.
fn decode(file: &[u8], m: &moof::Moof, sides: u8, mfm: bool) -> Vec<u8> {
    let size = if mfm { 1_474_560 } else { 409_600 * sides as usize };
    let mut raw = vec![0u8; size];
    let mut found = 0;
    for t in 0..TRACKS {
        for side in 0..sides {
            let (_, trk) = m.track(t, side).unwrap();
            let data = &file[trk.start as usize * BLOCK..];
            let n = trk.bits as usize;
            let bit = |i: usize| data[i % n / 8] >> (7 - i % n % 8) & 1;
            if mfm {
                let f = mfm::Format::Hd;
                let raw16 = |i: usize| (0..16).fold(0u16, |v, k| v << 1 | bit(i + k) as u16);
                let byte_at = |i: usize| (0..8).fold(0u8, |v, k| v << 1 | bit(i + 2 * k + 1));
                let mut id = None;
                let mut i = 0;
                while i < n {
                    if raw16(i) == 0x4489 && raw16(i + 16) == 0x4489 && raw16(i + 32) == 0x4489 {
                        let p = i + 48;
                        match byte_at(p) {
                            0xFE => id = Some(byte_at(p + 16 + 32)),
                            0xFB => {
                                let s = id.take().expect("data after ID") as usize;
                                let at = (f.first_block(t, side) + s - 1) * BLOCK;
                                for k in 0..BLOCK {
                                    raw[at + k] = byte_at(p + 16 + k * 16);
                                }
                                found += 1;
                                i = p + 16 * 515;
                                continue;
                            }
                            _ => {}
                        }
                        i = p + 16;
                    } else {
                        i += 1;
                    }
                }
            } else {
                // Frame as an IWM would and decode each data field.
                let mut nib = vec![];
                let mut sh = 0u8;
                for i in 0..2 * n {
                    sh = sh << 1 | bit(i);
                    if sh & 0x80 != 0 {
                        nib.push(sh);
                        sh = 0;
                    }
                }
                let spt = gcr::sectors_per_track(t) as usize;
                let first = gcr::first_block(t, sides) + side as usize * spt;
                let mut sector = None;
                let mut seen = vec![false; spt];
                for k in 0..nib.len().saturating_sub(3) {
                    match (nib[k], nib[k + 1], nib[k + 2]) {
                        (0xD5, 0xAA, 0x96) => sector = Some(nib[k + 4]),
                        (0xD5, 0xAA, 0xAD) if k + 4 + 703 <= nib.len() => {
                            let Some(s) = sector.take() else { continue };
                            let s = ENC.iter().position(|&e| e == s).unwrap();
                            if s < spt && !seen[s] {
                                let d = gcr::decode_sector(nib[k + 4..k + 4 + 703].try_into().unwrap()).expect("sector decodes");
                                raw[(first + s) * BLOCK..][..BLOCK].copy_from_slice(&d[12..]);
                                seen[s] = true;
                                found += 1;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    assert_eq!(found, size / BLOCK, "every sector found");
    raw
}

const ENC: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];
