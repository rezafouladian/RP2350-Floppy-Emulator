//! Host-side check of the track builders (`src/track.rs`, `gcr.rs`, `mfm.rs`, `moof.rs`); the
//! firmware crate itself only builds for the RP2350.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/track_check.rs -o target/track_check && target/track_check
//!
//! - GCR: reference vectors, random sector round trips
//!   through an independent decoder, then every track of 400K and 800K images read back the way
//!   an IWM would (byte complete when bit 7 is set) and compared with the image.
//! - MFM: CRC vectors, the MFM clock rule, and every sector of a 1.44 MB and a 720K (DiskCopy)
//!   image decoded and compared.
//! - MOOF: header and CRC, rotated track loading (bit-exact, wrap in a gap, whole-nibble fill)
//!   and every sector against the data of MOOFs with the traits of real ones (Macintosh Pascal,
//!   Skyfox, Dollars and Sense, SimCity, MacWars: see `disks.rs`), and an 800K MOOF built from
//!   the firmware's own tracks, decoded back.
//! - The Ancient Art of War's protection (track 78 to 79 alignment and nibble counts), run on
//!   the tracks as the firmware plays them.
//! - Apple II 3.5": tracks built with the IIgs's format byte (`$24`, 4:1 interleave), also from
//!   a `.2mg`; the physical sector order of both interleaves in every zone (the 4:1 one as on a
//!   real IIgs disk, The Oregon Trail); and a 3.5" WOZ 2 of a IIgs disk, read as a MOOF.
//!
//! The images are made at test time (`disks.rs`).

mod disks;
#[path = "../../src/gcr.rs"]
mod gcr;
#[path = "../../src/mfm.rs"]
mod mfm;
#[path = "../../src/moof.rs"]
mod moof;
#[path = "../../src/track.rs"]
mod track;
use gcr::*;
use track::*;

const ENC: [u8; 64] = [
    0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6, 0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
    0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
    0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
    0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

fn hex(v: &[u8]) -> String {
    v.iter().map(|b| format!("{:02X}", b)).collect::<Vec<_>>().join(" ")
}

/// Independent decoder, written separately from the firmware's.
fn decode(inp: &[u8]) -> Result<Vec<u8>, i32> {
    let mut dec = [-1i32; 256];
    for (i, &e) in ENC.iter().enumerate() {
        dec[e as usize] = i as i32;
    }
    if inp.len() != 703 || inp.iter().any(|&b| dec[b as usize] < 0) {
        return Err(-1);
    }
    let d = |b: u8| dec[b as usize] as u32;
    let (mut c1, mut c2, mut c3) = (0u32, 0u32, 0u32);
    let mut i = 0;
    let mut out = vec![];
    while out.len() < 524 {
        let last = out.len() + 2 == 524;
        let hi = d(inp[i]);
        let mut a = d(inp[i + 1]) | ((hi << 2) & 0xC0);
        let mut b = d(inp[i + 2]) | ((hi << 4) & 0xC0);
        i += 3;
        let mut x = c3 >> 7;
        c3 = ((c3 << 1) | x) & 0xFF;
        a ^= c3;
        c1 += a + x;
        x = c1 >> 8;
        c1 &= 0xFF;
        out.push(a as u8);
        b ^= c1;
        c2 += b + x;
        x = c2 >> 8;
        c2 &= 0xFF;
        out.push(b as u8);
        if !last {
            let mut c = d(inp[i]) | ((hi << 6) & 0xC0);
            i += 1;
            c ^= c2;
            c3 += c + x;
            c3 &= 0xFF;
            out.push(c as u8);
        }
    }
    let hi = d(inp[i]);
    let k1 = d(inp[i + 1]) | ((hi << 2) & 0xC0);
    let k2 = d(inp[i + 2]) | ((hi << 4) & 0xC0);
    let k3 = d(inp[i + 3]) | ((hi << 6) & 0xC0);
    if (k1, k2, k3) == (c1, c2, c3) { Ok(out) } else { Err(-2) }
}

fn enc(tags: &[u8; 12], data: &[u8]) -> Vec<u8> {
    let mut v = vec![];
    encode_sector(tags, data, |n| v.push(n));
    v
}

fn main() {
    check_gcr();
    check_mfm();
    check_moof();
    check_aaow();
    check_woz35();
    println!("all checks passed");
}

fn check_gcr() {
    // Reference vectors.
    assert_eq!(hex(&address_field(0, 0, 0, 0x22)), "96 96 96 D9 D9");
    assert_eq!(hex(&address_field(79, 7, 1, 0x22)), "B3 A6 D7 D9 AD");
    let z = enc(&[0; 12], &[0; 512]);
    assert_eq!(z.len(), 703);
    assert!(z.iter().all(|&b| b == 0x96));
    let all: Vec<u8> = (0..524).map(|i| i as u8).collect();
    let p = enc(all[..12].try_into().unwrap(), &all[12..]);
    assert_eq!(hex(&p[..8]), "96 96 97 9B 96 A6 A6 96");
    assert_eq!(hex(&p[695..]), "EA CD FA A7 F4 9B EC ED");

    let mut s = 0x1234_5678u32;
    for _ in 0..2000 {
        let v: Vec<u8> = (0..524)
            .map(|_| {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                s as u8
            })
            .collect();
        assert_eq!(decode(&enc(v[..12].try_into().unwrap(), &v[12..])).unwrap(), v);
    }
    println!("codec vectors and random round trips OK");

    let ss: Vec<usize> = [16, 32, 48, 64, 80].iter().map(|&t| first_block(t, 1)).collect();
    assert_eq!(ss, [192, 368, 528, 672, 800]);
    let ds: Vec<usize> = [16, 32, 48, 64, 80].iter().map(|&t| first_block(t, 2)).collect();
    assert_eq!(ds, [384, 736, 1056, 1344, 1600]);

    // The Mac's 2:1 interleave, and the IIgs's 4:1 one.
    assert_eq!((format(1, false), format(2, false), format(1, true), format(2, true)), (0x02, 0x22, 0x04, 0x24));
    let (r400, r800) = (disks::random_bytes(409_600, 400), disks::random_bytes(819_200, 800));
    let (_, _, two_mg) = disks::oregon();
    let images = [
        ("400K image", &r400[..], 1u8, false, 0usize),
        ("800K image", &r800[..], 2, false, 0),
        ("800K image", &r800[..], 2, true, 0),
        ("The Oregon Trail .2mg", &two_mg[..], 2, true, 64), // blocks after its 64-byte header
    ];
    for (file, img, sides, apple2, offset) in images {
        let img = &img[offset..][..sides as usize * 409_600];
        let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
        let mut recovered = vec![0u8; img.len()];
        let fmt = format(sides, apple2);
        for track in 0..TRACKS {
            for side in 0..sides {
                let spt = sectors_per_track(track) as usize;
                let first = first_block(track, sides) + side as usize * spt;
                let n = build_track(
                    track,
                    side,
                    fmt,
                    |s| img[(first + s as usize) * BLOCK..][..BLOCK].try_into().unwrap(),
                    &mut buf,
                );
                // IWM framing over two revolutions, so a sector across the wrap is still seen.
                let mut nibbles = vec![];
                let mut sh = 0u8;
                for i in 0..n * 64 {
                    let i = i % (n * 32);
                    sh = sh << 1 | (buf[i / 32] >> (31 - i % 32) & 1) as u8;
                    if sh & 0x80 != 0 {
                        nibbles.push(sh);
                        sh = 0;
                    }
                }
                let mut seen = vec![false; spt];
                let mut order = vec![];
                let mut i = 0;
                while i + 720 < nibbles.len() {
                    if nibbles[i..i + 3] != [0xD5, 0xAA, 0x96] {
                        i += 1;
                        continue;
                    }
                    let h = &nibbles[i + 3..i + 8];
                    let sec = ENC.iter().position(|&e| e == h[1]).unwrap();
                    assert_eq!(h, &address_field(track, sec as u8, side, fmt));
                    assert_eq!(nibbles[i + 8..i + 10], [0xDE, 0xAA]);
                    // The ROM wants the data mark within 48 nibbles of the address trailer.
                    let j = (i + 10..i + 58)
                        .find(|&j| nibbles[j..j + 3] == [0xD5, 0xAA, 0xAD])
                        .expect("data mark");
                    assert_eq!(nibbles[j + 3], h[1]);
                    let d = decode(&nibbles[j + 4..j + 707]).unwrap();
                    assert_eq!(nibbles[j + 707..j + 709], [0xDE, 0xAA]);
                    let blk = first + sec;
                    recovered[blk * BLOCK..][..BLOCK].copy_from_slice(&d[12..]);
                    seen[sec] = true;
                    order.push(sec as u8);
                    i = j + 709;
                }
                assert!(seen.iter().all(|&s| s), "{file} track {track} side {side}: missing sectors");
                assert_eq!(order[..spt], *want_order(spt, fmt & 0xF), "{file} track {track}: physical order");
                if side == 0 && track % 16 == 0 {
                    println!(
                        "  track {track:2}: {n} words, {:.1} ms/rev",
                        n as f64 * 32.0 * 2.0425 / 1000.0
                    );
                }
            }
        }
        assert_eq!(recovered, img, "{file}");
        println!("{file}: all {} blocks round-trip, format ${fmt:02X}, interleave as wanted", img.len() / BLOCK);
    }
}

/// Physical sector order for `spt` sectors at `factor`:1 interleave. 2:1 is the Mac's table;
/// 4:1 as found on an Apple IIgs disk (The Oregon Trail, by Applesauce) in every zone.
fn want_order(spt: usize, factor: u8) -> &'static [u8] {
    match (factor, spt) {
        (2, 12) => &[0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11],
        (2, 11) => &[0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5],
        (2, 10) => &[0, 5, 1, 6, 2, 7, 3, 8, 4, 9],
        (2, 9) => &[0, 5, 1, 6, 2, 7, 3, 8, 4],
        (2, 8) => &[0, 4, 1, 5, 2, 6, 3, 7],
        (4, 12) => &[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11],
        (4, 11) => &[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8],
        (4, 10) => &[0, 5, 3, 8, 1, 6, 4, 9, 2, 7],
        (4, 9) => &[0, 7, 5, 3, 1, 8, 6, 4, 2],
        (4, 8) => &[0, 2, 4, 6, 1, 3, 5, 7],
        _ => panic!("no table for {spt} sectors at {factor}:1"),
    }
}

/// An Apple II 3.5" WOZ 2 read as a MOOF (`Moof::parse`): header, CRC, every sector of every
/// track side as the firmware loads it, its address fields' format byte and 4:1 order, and the
/// data against the same disk's `.2mg`. Also what's refused.
fn check_woz35() {
    let (_, file, two) = disks::oregon();
    let blocks = file.len().div_ceil(BLOCK);
    let header: &[u8; moof::HEADER_BYTES] = file[..moof::HEADER_BYTES].try_into().unwrap();
    let m = moof::Moof::parse(header, None, blocks, MAX_TRACK_BLOCKS).expect("3.5\" WOZ parses");
    assert!(m.woz && !m.mfm && m.sides == 2 && !m.has_flux);
    let mut c = moof::Crc32::new();
    c.update(&file[12..]);
    assert_eq!(c.finish(), m.stored_crc, "WOZ CRC");
    let mut recovered = vec![0u8; 819_200];
    let mut decoded = 0;
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let spt = sectors_per_track(track) as usize;
            let first = first_block(track, 2) + side as usize * spt;
            let sectors = check_moof_track(&file, &m, track, side).expect("side present");
            let order: Vec<u8> = sectors.iter().map(|(h, _)| ENC.iter().position(|&e| e == h[1]).unwrap() as u8).collect();
            // One pass from the rotated start: the 4:1 order from somewhere in it.
            let want = want_order(spt, 4);
            let at = want.iter().position(|&s| s == order[0]).unwrap();
            assert!(order.iter().enumerate().all(|(i, &s)| s == want[(at + i) % spt]), "track {track} side {side}: order {order:?}");
            for (h, d) in sectors {
                let sec = ENC.iter().position(|&e| e == h[1]).unwrap();
                assert_eq!(h, address_field(track, sec as u8, side, 0x24));
                // Block 8's data field fails its checksum in the image itself (its nibbles are
                // all legal and its trailer is right), as on the real disk's WOZ: played
                // bit-exact, it fails on a IIgs too.
                let Some(d) = d else {
                    assert_eq!((track, side, sec), (0, 0, 8), "only track 0 sector 8 is bad in the image");
                    continue;
                };
                recovered[(first + sec) * BLOCK..][..BLOCK].copy_from_slice(&d[12..]);
                decoded += 1;
            }
        }
    }
    assert_eq!(decoded, 1599);
    let same = (0..1600).filter(|&b| recovered[b * BLOCK..][..BLOCK] == two[64 + b * BLOCK..][..BLOCK]).count();
    assert_eq!(same, 1599, "blocks equal to the .2mg's");
    println!("The Oregon Trail 3.5\" WOZ: CRC OK, 1599 of 1600 sectors decode (block 8 is bad in the image), format $24 at 4:1, the rest equal the .2mg's");

    // Without INFO version 2's sides field, a side 1 in the TMAP means two sides.
    let mut v1 = header.to_vec();
    v1[20] = 1;
    v1[57] = 0;
    assert_eq!(moof::Moof::parse(v1[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).unwrap().sides, 2);
    // WOZ 2.1's FLUX block is at INFO + 46, read only from INFO version 3.
    let mut flux = header.to_vec();
    flux[66..70].copy_from_slice(&[9, 0, 1, 0]);
    assert_eq!(moof::Moof::flux_block(&flux), None, "INFO version 2 has no FLUX block");
    flux[20] = 3;
    assert_eq!(moof::Moof::flux_block(&flux), Some(9));
    assert!(moof::Moof::parse(flux[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).is_err(), "FLUX named but not given");
    // A 5.25" WOZ, and a 3.5" one with bad sides or timing.
    let mut wrong = header.to_vec();
    wrong[21] = 1;
    let e = moof::Moof::parse(wrong[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).err();
    assert_eq!(e, Some("WOZ: 5.25\" disk (use mode = apple2)"));
    let mut wrong = header.to_vec();
    wrong[57] = 3;
    assert!(moof::Moof::parse(wrong[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).is_err());
    let mut wrong = header.to_vec();
    wrong[59] = 32;
    assert!(moof::Moof::parse(wrong[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).is_err());
    println!("3.5\" WOZ: sides inferred without INFO v2, FLUX block located, 5.25\" and bad headers refused");
}

/// IWM-style framing of `bits` cells of a looped word buffer, starting at bit `from`.
/// Returns (nibble, bit index just after it).
fn frame(buf: &[u32], words: usize, from: usize, bits: usize) -> Vec<(u8, usize)> {
    let n = words * 32;
    let mut out = vec![];
    let mut sh = 0u8;
    for i in from..from + bits {
        let j = i % n;
        sh = sh << 1 | (buf[j / 32] >> (31 - j % 32) & 1) as u8;
        if sh & 0x80 != 0 {
            out.push((sh, i + 1));
            sh = 0;
        }
    }
    out
}

/// Every GCR address field in a nibble stream, with its sector data if the data field is
/// there and decodes (data mark within 48 nibbles, checksum, trailers).
fn gcr_sectors(nibbles: &[u8]) -> Vec<([u8; 5], Option<Vec<u8>>)> {
    let mut out = vec![];
    let mut i = 0;
    while i + 10 <= nibbles.len() {
        if nibbles[i..i + 3] != [0xD5, 0xAA, 0x96] || nibbles[i + 8..i + 10] != [0xDE, 0xAA] {
            i += 1;
            continue;
        }
        let h: [u8; 5] = nibbles[i + 3..i + 8].try_into().unwrap();
        let data = (i + 10..i + 58)
            .find(|&j| j + 709 <= nibbles.len() && nibbles[j..j + 3] == [0xD5, 0xAA, 0xAD])
            .filter(|&j| nibbles[j + 3] == h[1] && nibbles[j + 707..j + 709] == [0xDE, 0xAA])
            .and_then(|j| decode(&nibbles[j + 4..j + 707]).ok());
        out.push((h, data));
        i += 10;
    }
    out
}

fn check_mfm() {
    use mfm::*;
    assert_eq!(crc16(0xFFFF, &[0xA1, 0xA1, 0xA1]), 0xCDB4);
    assert_eq!(crc16(0xFFFF, &[0xA1, 0xA1, 0xA1, 0xFE, 0, 0, 1, 2]), 0xCA6F);
    assert_eq!(crc16(0xFFFF, &[0xA1, 0xA1, 0xA1, 0xFE, 0, 0, 1, 2, 0xCA, 0x6F]), 0);

    let img = disks::random_bytes(1_474_560, 1440);
    check_mfm_image("1.44 MB image", &img, Format::Hd, 6_250);

    // 720K: the sector data of a DiskCopy 4.2 image (84-byte header, no tags).
    let file = "720K DiskCopy image";
    let dc42 = disks::diskcopy(&disks::random_bytes(737_280, 720), None, "PSLPwAAS");
    assert_eq!(dc42.len(), 84 + 737_280);
    assert_eq!(u32::from_be_bytes(dc42[0x40..0x44].try_into().unwrap()), 737_280);
    assert_eq!(dc42[0x50], 2, "DiskCopy format 2 = 720K");
    check_mfm_image(file, &dc42[84..], Format::Dd, 3_125);
}

/// Encode every track side of the raw MFM image `img`, check the clock rule and the index mark,
/// and decode every sector back. `words` is one revolution: 200 ms (HD) or 100 ms (DD) at 1 µs
/// half-cells.
fn check_mfm_image(file: &str, img: &[u8], format: mfm::Format, words_per_track: usize) {
    use mfm::*;
    assert_eq!(img.len(), 160 * format.sectors() as usize * BLOCK);
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    let mut recovered = vec![0u8; img.len()];
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let first = format.first_block(track, side);
            let words = build_track(
                format,
                track,
                side,
                |s| img[(first + s as usize - 1) * BLOCK..][..BLOCK].try_into().unwrap(),
                &mut buf,
            );
            assert_eq!(words, words_per_track, "one revolution");
            let n = words * 32;
            let bit = |i: usize| (buf[i % n / 32] >> (31 - i % n % 32) & 1) as u8;
            // MFM clock rule around the whole loop, wrap included: no two adjacent reversals,
            // never more than three empty half-cells.
            let mut zeros = 0;
            for i in 0..n + 8 {
                if bit(i) == 1 {
                    assert!(zeros >= 1 || i == 0, "adjacent reversals at {i}");
                    zeros = 0;
                } else {
                    zeros += 1;
                    assert!(zeros <= 3, "run of zeros at track {track} side {side} bit {i}");
                }
            }
            // Find A1 A1 A1 (raw 4489 x3) at any bit offset, then read data bits.
            let raw16 = |i: usize| (0..16).fold(0u16, |v, k| v << 1 | bit(i + k) as u16);
            let byte_at = |i: usize| (0..8).fold(0u8, |v, k| v << 1 | bit(i + 2 * k + 1));
            let mut sectors = vec![];
            let mut i = 0;
            let mut pending_id: Option<[u8; 4]> = None;
            while i < n {
                if raw16(i) != 0x4489 || raw16(i + 16) != 0x4489 || raw16(i + 32) != 0x4489 {
                    i += 1;
                    continue;
                }
                let p = i + 48;
                let mark = byte_at(p);
                match mark {
                    0xFE => {
                        let f: Vec<u8> = (0..6).map(|k| byte_at(p + 16 + k * 16)).collect();
                        let mut crc_in = vec![0xA1, 0xA1, 0xA1, 0xFE];
                        crc_in.extend(&f);
                        assert_eq!(crc16(0xFFFF, &crc_in), 0, "ID CRC");
                        pending_id = Some(f[..4].try_into().unwrap());
                        i = p + 16 * 7;
                    }
                    0xFB => {
                        let id = pending_id.take().expect("data mark without ID");
                        let d: Vec<u8> = (0..514).map(|k| byte_at(p + 16 + k * 16)).collect();
                        let mut crc_in = vec![0xA1, 0xA1, 0xA1, 0xFB];
                        crc_in.extend(&d);
                        assert_eq!(crc16(0xFFFF, &crc_in), 0, "data CRC");
                        assert_eq!(id, [track, side, id[2], 2]);
                        let blk = first + id[2] as usize - 1;
                        recovered[blk * BLOCK..][..BLOCK].copy_from_slice(&d[..512]);
                        sectors.push(id[2]);
                        i = p + 16 * 515;
                    }
                    m => panic!("unexpected mark {m:02X}"),
                }
            }
            assert_eq!(sectors, (1..=format.sectors()).collect::<Vec<_>>(), "track {track} side {side}");
        }
    }
    // Index mark: C2 C2 C2 as raw 5224 near the start of every track.
    let words = build_track(format, 0, 0, |s| img[(s as usize - 1) * BLOCK..][..BLOCK].try_into().unwrap(), &mut buf);
    let n = words * 32;
    let bit = |i: usize| (buf[i % n / 32] >> (31 - i % n % 32) & 1) as u16;
    assert!((0..2000).any(|i| (0..16).fold(0u16, |v, k| v << 1 | bit(i + k)) == 0x5224));
    assert!(recovered == img, "{file}: decoded blocks differ from the image");
    println!("{file}: all {} blocks round-trip (MFM {format:?})", img.len() / BLOCK);
}

/// Marks in three revolutions of a GCR stream, framed as an IWM would: `D` data mark and `A`
/// address mark (at their first bit), `T` the end of a data field's `DE AA` trailer.
fn gcr_events(src: &impl Fn(usize) -> u32, n: usize) -> Vec<(usize, char)> {
    let (mut sh, mut recent, mut in_data) = (0u8, 0u32, false);
    let mut events = vec![];
    for i in 0..3 * n {
        sh = sh << 1 | src(i % n) as u8;
        if sh & 0x80 == 0 {
            continue;
        }
        recent = recent << 8 | u32::from(sh);
        sh = 0;
        if recent & 0xFF_FFFF == 0xD5_AAAD {
            in_data = true;
            events.push((i + 1 - 24, 'D'));
        } else if recent & 0xFFFF == 0xDEAA && in_data {
            in_data = false;
            events.push((i + 1, 'T'));
        } else if recent & 0xFF_FFFF == 0xD5_AA96 {
            in_data = false;
            events.push((i + 1 - 24, 'A'));
        }
    }
    events
}

/// The fill after a GCR loop's `n` cells is whole `FF` nibbles with at most two 0s in a row,
/// and the wrap is a nibble boundary: framed on across loops, the second loop's nibbles are the
/// first's, which started with a fresh framer at word 0.
fn check_fill(buf: &[u32], words: usize, n: usize, what: &str) {
    let len = words * 32;
    let cell = |k: usize| buf[k % len / 32] >> (31 - k % 32) & 1;
    let zeros = (n..len).fold((0, 0), |(run, max), k| if cell(k) == 0 { (run + 1, max.max(run + 1)) } else { (0, max) });
    assert!(zeros.1 <= 2, "{what}: fill has {} 0s in a row", zeros.1);
    let two = frame(buf, words, 0, 2 * len);
    let first: Vec<_> = two.iter().filter(|&&(_, end)| end <= len).copied().collect();
    let second: Vec<_> = two.iter().filter(|&&(_, end)| end > len).map(|&(b, end)| (b, end - len)).collect();
    assert!(second.starts_with(&first), "{what}: framing changes at the wrap");
    assert!(first.iter().filter(|&&(_, end)| end > n).all(|&(b, _)| b == 0xFF), "{what}: fill isn't FF nibbles");
}

/// Load one MOOF track side the way the firmware does and check it.
fn check_moof_track(file: &[u8], m: &moof::Moof, track: u8, side: u8) -> Option<Vec<([u8; 5], Option<Vec<u8>>)>> {
    let (_, trk) = m.track(track, side)?;
    let data = &file[trk.start as usize * BLOCK..][..trk.blocks as usize * BLOCK];
    let byte = |i: usize| data[i];
    let rot = moof::find_rotation(byte, trk.bits);
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    let words = moof::load_track(byte, trk.bits, rot, moof::Pad::Gcr, &mut buf);
    let n = trk.bits as usize;
    assert!(words == n.div_ceil(32) || words == n.div_ceil(32) + 1, "track {track} side {side}: {words} words");
    // Bit exact: output bit k is source bit (k + rot) mod n, then the fill.
    let src = |i: usize| (data[i / 8] >> (7 - i % 8) & 1) as u32;
    for k in 0..n {
        assert_eq!(buf[k / 32] >> (31 - k % 32) & 1, src((k + rot as usize) % n), "track {track} side {side} bit {k}");
    }
    check_fill(&buf[..], words, n, &format!("track {track} side {side}"));
    // One revolution from word 0, with a fresh framer, holds every address field whole: the
    // loop wraps (and every stream restart begins) in a gap.
    let one: Vec<u8> = frame(&buf[..], words, 0, words * 32).iter().map(|&(b, _)| b).collect();
    let sectors = gcr_sectors(&one);
    assert_eq!(sectors.len(), sectors_per_track(track) as usize, "track {track} side {side}: headers in one pass");
    // The wrap, where the padding goes and every restart begins, is in a gap: in the original
    // stream, the last mark before the rotation point is a data field's trailer and the next is
    // an address mark, each at least 40 cells away. (Gap filler varies: FF sync on most disks,
    // AA on Dollars and Sense, a repeating FD EA AF DE AA on SimCity.)
    let events = gcr_events(&src, n);
    if events.iter().any(|&(_, kind)| kind == 'T') {
        let at = n + rot as usize;
        let prev = events.iter().filter(|&&(p, _)| p <= at).last().copied();
        let next = events.iter().find(|&&(p, _)| p > at).copied();
        let ok = matches!((prev, next), (Some((p, 'T')), Some((q, 'A'))) if at - p >= 40 && q - at >= 40);
        assert!(ok, "track {track} side {side}: wrap at {rot} not in a gap ({prev:?} .. {next:?})");
    }
    // The same sectors decode as from the original, unrotated stream (two revolutions, so the
    // sector across its wrap is seen whole too).
    let mut sh = 0u8;
    let mut orig = vec![];
    for i in 0..2 * n {
        sh = sh << 1 | src(i % n) as u8;
        if sh & 0x80 != 0 {
            orig.push(sh);
            sh = 0;
        }
    }
    let good = |v: &[([u8; 5], Option<Vec<u8>>)]| {
        let mut g: Vec<_> = v.iter().filter_map(|(h, d)| d.as_ref().map(|_| h[1])).collect();
        g.sort();
        g.dedup();
        g
    };
    assert_eq!(good(&sectors), good(&gcr_sectors(&orig)), "track {track} side {side}: decodable sectors");
    Some(sectors)
}

fn check_moof() {
    // CRC-32 check value.
    let mut c = moof::Crc32::new();
    c.update(b"123456789");
    assert_eq!(c.finish(), 0xCBF4_3926);

    // GCR MOOFs: sides, and how many data fields decode. Pascal has one damaged data mark in the
    // image. Skyfox is a copy-protected original: its track 1 has a sector with no data field,
    // whose place holds timing records with 32 µs no-flux gaps; MacWars has such records too, and
    // a sector with a no-flux stretch in place of its data field. Dollars and Sense fills its
    // gaps with AA rather than FF sync (and has one missing data field), SimCity with a repeating
    // FD EA AF DE AA.
    for (name, disk, want_decoded) in [
        ("Macintosh Pascal", disks::pascal(), 799),
        ("Skyfox", disks::skyfox(), 799),
        ("Dollars and Sense", disks::dollars(), 799),
        ("SimCity", disks::simcity(), 1600),
        ("MacWars", disks::macwars(), 798),
    ] {
        let sides = disk.sides;
        let file = disks::moof(&disk);
        let blocks = file.len().div_ceil(BLOCK);
        let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).unwrap();
        assert_eq!(m.sides, sides, "{name}");
        let mut c = moof::Crc32::new();
        c.update(&file[12..]);
        assert_eq!(c.finish(), m.stored_crc, "{name} CRC");
        let (mut headers, mut decoded) = (0, 0);
        let format = if sides == 1 { 0x02 } else { 0x22 };
        for track in 0..TRACKS {
            for side in 0..sides {
                let sectors = check_moof_track(&file, &m, track, side).expect("side present");
                let first = first_block(track, sides) + side as usize * sectors_per_track(track) as usize;
                for (h, d) in &sectors {
                    let sec = ENC.iter().position(|&e| e == h[1]).unwrap() as u8;
                    assert_eq!(h, &address_field(track, sec, side, format));
                    headers += 1;
                    if let Some(d) = d {
                        assert!(d[12..] == disk.raw[(first + sec as usize) * BLOCK..][..BLOCK], "{name} track {track} sector {sec}: data");
                        decoded += 1;
                    }
                }
            }
            if sides == 1 {
                assert!(m.track(track, 1).is_none(), "single-sided: side 1 blank");
            }
        }
        assert_eq!((headers, decoded), (800 * sides as usize, want_decoded), "{name}");
        println!("{name}: CRC OK, {headers} headers, {decoded} data fields decode to the data, same as unrotated, every wrap in a gap");
    }

    // Rejections: bad signature, Twiggy. (MFM MOOFs are read in write_check.rs.)
    let file = disks::moof(&disks::pascal());
    let blocks = file.len().div_ceil(BLOCK);
    let mut twiggy = file[..moof::HEADER_BYTES].to_vec();
    twiggy[21] = 4;
    assert!(moof::Moof::parse(twiggy[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).is_err());
    let mut bad = file[..moof::HEADER_BYTES].to_vec();
    bad[4] = 0;
    assert!(moof::Moof::parse(bad[..].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).is_err());

    // An 800K MOOF built from a raw image with the firmware's tracks.
    let raw = disks::random_bytes(819_200, 0x800);
    let mut out = vec![0u8; moof::HEADER_BYTES];
    out[..8].copy_from_slice(b"MOOF\xFF\x0A\x0D\x0A");
    out[12..16].copy_from_slice(b"INFO");
    out[16..20].copy_from_slice(&60u32.to_le_bytes());
    out[20] = 1; // version
    out[21] = 2; // 800K
    out[22] = 1; // write protected
    out[24] = 16; // bit timing
    out[25..57].copy_from_slice(&[b' '; 32]);
    out[80..84].copy_from_slice(b"TMAP");
    out[84..88].copy_from_slice(&160u32.to_le_bytes());
    out[88..248].fill(0xFF);
    out[248..252].copy_from_slice(b"TRKS");
    let mut largest = 0u16;
    let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let spt = sectors_per_track(track) as usize;
            let first = first_block(track, 2) + side as usize * spt;
            build_track(track, side, 0x22, |s| raw[(first + s as usize) * BLOCK..][..BLOCK].try_into().unwrap(), &mut buf);
            let bits = spt * 6220; // exact cell count, not word-padded
            let bytes: Vec<u8> = buf.iter().flat_map(|w| w.to_be_bytes()).take(bits.div_ceil(8)).collect();
            let start = out.len() / BLOCK;
            let nblocks = bytes.len().div_ceil(BLOCK);
            out.extend(&bytes);
            out.resize((start + nblocks) * BLOCK, 0);
            let idx = track as usize * 2 + side as usize;
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

    let m = moof::Moof::parse(out[..moof::HEADER_BYTES].try_into().unwrap(), None, out.len() / BLOCK, MAX_TRACK_BLOCKS).unwrap();
    assert_eq!(m.sides, 2);
    let mut recovered = vec![0u8; raw.len()];
    for track in 0..TRACKS {
        for side in 0..2u8 {
            let spt = sectors_per_track(track) as usize;
            let first = first_block(track, 2) + side as usize * spt;
            for (h, d) in check_moof_track(&out, &m, track, side).unwrap() {
                let d = d.expect("synthetic sector decodes");
                let sec = ENC.iter().position(|&e| e == h[1]).unwrap();
                assert_eq!(h, address_field(track, sec as u8, side, 0x22));
                recovered[(first + sec) * BLOCK..][..BLOCK].copy_from_slice(&d[12..]);
            }
        }
    }
    assert_eq!(recovered, raw);
    println!("synthetic 800K MOOF ({} blocks): all 1600 blocks round-trip", out.len() / BLOCK);
}

/// The Ancient Art of War (`CODE` 12, decrypted): with interrupts off it reads address fields on
/// track 78 until sector 0's, steps to 79 at once, waits 41.8 ms (VIA timer 2: 9,360 + 23,400
/// ticks), then counts nibbles up to `AA FF FF` three times. The first count must be 512–895,
/// the next two (one revolution each) 5,376–5,807. Track 79 has no sectors; the first count
/// only works with 79 at the angle it was imaged at relative to 78 (the MOOF is synchronized).
fn check_aaow() {
    let name = "The Ancient Art of War";
    let file = disks::moof(&disks::aaow());
    let blocks = file.len().div_ceil(BLOCK);
    let m = moof::Moof::parse(file[..moof::HEADER_BYTES].try_into().unwrap(), None, blocks, MAX_TRACK_BLOCKS).unwrap();
    assert!(m.synchronized, "{name}: imaged with cross-track sync");
    for track in 0..79 {
        let sectors = check_moof_track(&file, &m, track, 0).expect("side present");
        assert!(sectors.iter().all(|(_, d)| d.is_some()), "{name} track {track}");
    }
    // Tracks 78 and 79 as `Cylinder::build` makes them: (buffer, words, rotation in words).
    let load = |track: u8| {
        let (_, trk) = m.track(track, 0).unwrap();
        let data = &file[trk.start as usize * BLOCK..][..trk.blocks as usize * BLOCK];
        let rot = moof::find_rotation(|i| data[i], trk.bits);
        let mut buf = Box::new([0u32; MAX_TRACK_WORDS]);
        let words = moof::load_track(|i| data[i], trk.bits, rot, moof::Pad::Gcr, &mut buf);
        check_fill(&buf[..], words, trk.bits as usize, &format!("{name} track {track}"));
        (buf, words, rot as usize / 32)
    };
    let (t78, t79) = (load(78), load(79));
    let cell = |t: &(Box<[u32; MAX_TRACK_WORDS]>, usize, usize), i: usize| t.0[i / 32 % t.1] >> (31 - i % 32) & 1;
    let cells = |ms: f64| (ms * 1000.0 / 2.0425) as usize;
    let mut tried = 0;
    let mut first = (usize::MAX, 0);
    for phase in (0..t78.1).step_by(61) {
        // Track 78 playing from word `phase` at cell 0. The end of sector 0's address field:
        let (mut sh, mut recent, mut i) = (0u8, 0u128, phase * 32);
        let t0 = loop {
            sh = sh << 1 | cell(&t78, i) as u8;
            i += 1;
            if sh & 0x80 != 0 {
                recent = recent << 8 | sh as u128;
                sh = 0;
                // D5 AA 96, track, sector 0, side, format, checksum, DE AA.
                if recent >> 56 & 0xFF_FFFF == 0xD5_AA96 && recent >> 40 & 0xFF == 0x96 && recent & 0xFFFF == 0xDEAA {
                    break i - phase * 32;
                }
            }
            assert!(i < (phase + 2 * t78.1) * 32, "{name}: no sector 0 on track 78");
        };
        let step = t0 + cells(0.05);
        for publish_ms in [2.0, 10.0, 20.0, 30.0, 38.0] {
            // Core 1 switches when core 0 publishes 79, at the same angle (`Player::spin_start`).
            let switch = step + cells(publish_ms);
            let next = ((phase * 32 + switch) / 32 + 1) % t78.1;
            let at = (next + t78.2) % t78.1;
            let start = (at * t79.1 / t78.1 + t79.1 - t79.2 % t79.1) % t79.1;
            let played = |i: usize| if i < switch { cell(&t78, phase * 32 + i) } else { cell(&t79, start * 32 + i - switch) };
            let from = step + cells(41.9);
            let (mut sh, mut nibbles) = (0u8, vec![]);
            for i in t0..from + 3 * 12_000 * 10 {
                sh = sh << 1 | played(i) as u8;
                if sh & 0x80 != 0 {
                    if i >= from {
                        nibbles.push(sh);
                    }
                    sh = 0;
                }
            }
            // As at CODE 12 + $3E6: count to AA FF FF (at most 12,000), then read 3 more.
            let mut p = 0;
            let mut count = || {
                let mut n = 0;
                while n < 12_000 {
                    p += 1;
                    if nibbles[p - 1] == 0xAA && { p += 1; nibbles[p - 1] == 0xFF } && { p += 1; nibbles[p - 1] == 0xFF } {
                        break;
                    }
                    n += 1;
                }
                p += 3;
                n
            };
            let c = [count(), count(), count()];
            assert!(
                (512..896).contains(&c[0]) && c[1..].iter().all(|n| (5376..5808).contains(n)),
                "{name}: protection counts {c:?} (start word {phase}, 79 published after {publish_ms} ms)"
            );
            first = (first.0.min(c[0]), first.1.max(c[0]));
            tried += 1;
        }
    }
    println!("{name}: tracks 0-78 decode, protection passes in {tried} runs (first count {}..{} of 512..895)", first.0, first.1);
}
