//! Makes the benchmark image set: a numbered folder per emulator mode, every image type the mode
//! plays. The images are blank volumes (HFS for
//! the Mac, ProDOS for the Apple II), so the write and file tests have room and nothing is lost;
//! the read-only types (flux MOOF, `.nib`, WOZ 1, A2R, WOZ 2.1 flux) get read tests only.
//!
//!   rustc --edition 2024 -O -A dead_code tools/bench_images.rs -o target/bench_images
//!   target/bench_images [--out images/Benchmarks] [--blank images/Blank]
//!     [--floppybench tools/floppybench/build/FloppyBench.dsk] [--a2bench tools/a2bench/build/BENCH.SYSTEM]
//!     [--boot <ProDOS image for boot block 0>] [--gsos ~/Downloads/gsos-6.0.2.zip]
//!
//! Inputs:
//! - `--blank`: the Mac blanks (`Blank 400K.dsk` … `Blank HD20 100MB.hda`), copied.
//! - `--floppybench`, `--a2bench`: the built programs (FloppyBench's `cmake --build`, A2Bench's `make`).
//! - `--boot`: blocks 0-1 of a ProDOS disk, the boot code (default the IIgs System Disk 1.1 copy in
//!   `images/Apple IIgs test/`; ProDOS's block 0 is the same on every disk seen).
//! - `--gsos`: the GS/OS 6.0.2 32 MB SmartPort image (a zip holding `smart0.po`, or the `.po`): it
//!   becomes SmartPort unit 1 with A2Bench added, and its `SYSTEM/P8` (ProDOS 8 2.0.3) the
//!   `PRODOS` of the bootable A2Bench disks.
//! - hfsutils' `hformat` (Retro68's toolchain) formats the partitioned HD20 image.
//!
//! The Apple II tracks, WOZ, A2R and MOOF encoders are the host tests' (`tests/host/disks.rs`),
//! the ProDOS volumes `tools/prodos.rs`'s.

#[path = "../tests/host/disks.rs"]
mod disks;
#[path = "../src/gcr.rs"]
mod gcr;
#[path = "prodos.rs"]
mod prodos;
#[path = "../src/track.rs"]
mod track;

use disks::{At, Fill, Kind, Odd, Trk};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const BLOCK: usize = 512;

fn home(p: &str) -> String {
    p.replacen('~', &std::env::var("HOME").unwrap_or_default(), 1)
}

fn put(dir: &Path, name: &str, data: &[u8]) {
    fs::write(dir.join(name), data).unwrap_or_else(|e| panic!("{name}: {e}"));
    println!("  {name} ({} KB)", data.len() / 1024);
}

fn copy(dir: &Path, name: &str, from: &Path) -> bool {
    match fs::read(from) {
        Ok(d) => {
            put(dir, name, &d);
            true
        }
        Err(e) => {
            println!("  ({name} skipped: {}: {e})", from.display());
            false
        }
    }
}

/// An empty ProDOS volume.
fn volume(blocks: usize, name: &str) -> Vec<u8> {
    prodos::new_volume(blocks, name)
}

/// A ProDOS volume that boots ProDOS 8 into A2Bench: `boot`'s blocks 0-1, `p8` as PRODOS, then
/// BENCH.SYSTEM (its first .SYSTEM file).
fn bootable(blocks: usize, name: &str, boot: &[u8], p8: &[u8], bench: &(Vec<u8>, u8, u16)) -> Vec<u8> {
    let mut v = volume(blocks, name);
    v[..2 * BLOCK].copy_from_slice(&boot[..2 * BLOCK]);
    let mut img = prodos::Image::from_bytes(v, name);
    img.put("PRODOS", 0xFF, 0, p8);
    img.put("BENCH.SYSTEM", bench.1, bench.2, &bench.0);
    img.bytes
}

/// Sector `phys` of track `t` of a ProDOS-order 5.25" image.
fn sector(po: &[u8], t: usize, phys: u8) -> [u8; 256] {
    let l = disks::PRODOS_PHYSICAL.iter().position(|&p| p == phys).unwrap();
    po[(t * 16 + l) * 256..][..256].try_into().unwrap()
}

/// The same disk in DOS 3.3 sector order (`.dsk`, `.do`).
fn dos_order(po: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; po.len()];
    for t in 0..po.len() / 4096 {
        for p in 0..16u8 {
            let l = disks::DOS_PHYSICAL.iter().position(|&x| x == p).unwrap();
            out[(t * 16 + l) * 256..][..256].copy_from_slice(&sector(po, t, p));
        }
    }
    out
}

/// A track's cells, as `disks::woz525` lays it out (51,200 cells, 200 ms at 3.906 µs).
fn a2_cells(po: &[u8], t: usize) -> Vec<bool> {
    disks::a2_track(t as u8, 254, 51_200, |p| sector(po, t, p))
}

/// `.nib`: 6,656 nibbles a track, syncs as plain `FF`s.
fn nib(po: &[u8]) -> Vec<u8> {
    let mut out = vec![];
    for t in 0..35u8 {
        let mut n = vec![0xFF; 40];
        for p in 0..16u8 {
            n.extend_from_slice(&[0xD5, 0xAA, 0x96]);
            for b in [254, t, p, 254 ^ t ^ p] {
                n.extend_from_slice(&[b >> 1 | 0xAA, b | 0xAA]);
            }
            n.extend_from_slice(&[0xDE, 0xAA, 0xEB]);
            n.extend_from_slice(&[0xFF; 6]);
            n.extend_from_slice(&[0xD5, 0xAA, 0xAD]);
            n.extend_from_slice(&disks::encode62(&sector(po, t as usize, p)));
            n.extend_from_slice(&[0xDE, 0xAA, 0xEB]);
            n.extend_from_slice(&[0xFF; 16]);
        }
        assert!(n.len() <= 6656);
        n.resize(6656, 0xFF);
        out.extend(n);
    }
    out
}

/// WOZ 1: 35 fixed 6,656-byte records after a 256-byte header.
fn woz1(po: &[u8]) -> Vec<u8> {
    let mut f = vec![0u8; 256];
    f[..8].copy_from_slice(b"WOZ1\xFF\x0A\x0D\x0A");
    f[12..16].copy_from_slice(b"INFO");
    f[16..20].copy_from_slice(&60u32.to_le_bytes());
    f[20] = 1; // version
    f[21] = 1; // 5.25"
    f[25..57].fill(b' ');
    let creator = b"rp2350-floppy bench_imgs";
    f[25..25 + creator.len()].copy_from_slice(creator);
    f[80..84].copy_from_slice(b"TMAP");
    f[84..88].copy_from_slice(&160u32.to_le_bytes());
    f[88..248].fill(0xFF);
    for t in 0..35usize {
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q >= 0 {
                f[88 + q as usize] = t as u8;
            }
        }
    }
    f[248..252].copy_from_slice(b"TRKS");
    f[252..256].copy_from_slice(&(35u32 * 6656).to_le_bytes());
    for t in 0..35 {
        let cells = a2_cells(po, t);
        let mut r = disks::pack(&cells);
        r.resize(6646, 0);
        r.extend_from_slice(&(cells.len().div_ceil(8) as u16).to_le_bytes());
        r.extend_from_slice(&(cells.len() as u16).to_le_bytes());
        r.extend_from_slice(&0xFFFFu16.to_le_bytes()); // no splice point
        r.extend_from_slice(&[0, 0, 0, 0]);
        f.extend(r);
    }
    let crc = disks::crc32(&f[12..]);
    f[8..12].copy_from_slice(&crc.to_le_bytes());
    f
}

/// A2R 3: each track captured 1.6 revolutions from a random angle (62.5 ns ticks, a drive
/// within 0.5% of speed, ±2 ticks), at its quarter track and one either side, the index the loop.
/// (Its 3.906 µs cells are 62.5 ticks.)
fn a2r(po: &[u8]) -> Vec<u8> {
    let mut s = 0xA2Au64;
    let mut f = b"A2R3\xFF\x0A\x0D\x0A".to_vec();
    let mut info = vec![1u8];
    let creator = b"rp2350-floppy bench_images";
    info.extend_from_slice(creator);
    info.extend_from_slice(&[b' '; 32][creator.len()..]);
    info.extend_from_slice(&[1, 0, 1, 0]); // 5.25", not write-protected, synchronized
    f.extend_from_slice(b"INFO");
    f.extend_from_slice(&(info.len() as u32).to_le_bytes());
    f.extend_from_slice(&info);
    let mut rwcp = vec![1u8];
    rwcp.extend_from_slice(&62_500u32.to_le_bytes());
    rwcp.extend_from_slice(&[0; 11]);
    for t in 0..35usize {
        let cells = a2_cells(po, t);
        let ticks = 62.5 * (1.0 + ((disks::rng(&mut s) % 101) as f64 - 50.0) / 10_000.0);
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q < 0 {
                continue;
            }
            let from = disks::rng(&mut s) as usize % cells.len();
            let flux = disks::flux(&cells, from, cells.len() * 8 / 5, ticks, false, &mut s, |_| At::Grid { shift: 0.0, jitter: 2 });
            rwcp.extend_from_slice(&[b'C', 1]);
            rwcp.extend_from_slice(&(q as u16).to_le_bytes());
            rwcp.push(1);
            rwcp.extend_from_slice(&((cells.len() as f64 * ticks) as u32).to_le_bytes());
            rwcp.extend_from_slice(&(flux.len() as u32).to_le_bytes());
            rwcp.extend_from_slice(&flux);
        }
    }
    rwcp.push(b'X');
    f.extend_from_slice(b"RWCP");
    f.extend_from_slice(&(rwcp.len() as u32).to_le_bytes());
    f.extend_from_slice(&rwcp);
    f
}

/// WOZ 2.1 with every track flux (125 ns ticks, one revolution, ±1 tick).
fn woz_flux(po: &[u8]) -> Vec<u8> {
    let mut s = 0xF1u64;
    let mut map = [0xFFu8; 160];
    let mut trks = vec![];
    for t in 0..35usize {
        let cells = a2_cells(po, t);
        trks.push(Trk::Flux(disks::flux(&cells, 0, cells.len(), 31.25, true, &mut s, |_| At::Grid { shift: 0.0, jitter: 1 })));
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q >= 0 {
                map[q as usize] = t as u8;
            }
        }
    }
    disks::container(Kind::Woz525(32), false, false, &map, &trks)
}

/// A Mac 800K MOOF of `raw` with every track side flux (125 ns ticks, ±1 tick): read-only, the
/// firmware converting each track as it's read.
fn mac_flux_moof(raw: &[u8]) -> Vec<u8> {
    let d = disks::mac_disk(raw, false, Fill::Sync, |_, _| 300, |_, _, _| Odd::Normal);
    let mut s = 0xF2u64;
    let mut map = [0xFFu8; 160];
    let trks: Vec<Trk> = d
        .tracks
        .iter()
        .map(|c| Trk::Flux(disks::flux(c, 0, c.len(), 16.34, true, &mut s, |_| At::Grid { shift: 0.0, jitter: 1 })))
        .collect();
    for (i, m) in map.iter_mut().enumerate() {
        *m = i as u8;
    }
    disks::container(Kind::Moof(2), true, false, &map, &trks)
}

/// A hard disk with an Apple partition map: the map (blocks 1-63) and one HFS partition after it,
/// formatted by hfsutils.
fn apm_hfs(blocks: usize, label: &str, path: &Path) -> bool {
    let mut d = vec![0u8; blocks * BLOCK];
    d[..2].copy_from_slice(b"ER");
    d[2..4].copy_from_slice(&512u16.to_be_bytes());
    d[4..8].copy_from_slice(&(blocks as u32).to_be_bytes());
    let mut entry = |i: usize, start: usize, n: usize, name: &[u8], kind: &[u8], status: u32| {
        let e = &mut d[i * BLOCK..(i + 1) * BLOCK];
        e[..2].copy_from_slice(b"PM");
        e[4..8].copy_from_slice(&2u32.to_be_bytes());
        e[8..12].copy_from_slice(&(start as u32).to_be_bytes());
        e[12..16].copy_from_slice(&(n as u32).to_be_bytes());
        e[16..16 + name.len()].copy_from_slice(name);
        e[48..48 + kind.len()].copy_from_slice(kind);
        e[84..88].copy_from_slice(&(n as u32).to_be_bytes());
        e[88..92].copy_from_slice(&status.to_be_bytes());
    };
    entry(1, 1, 63, b"Apple", b"Apple_partition_map", 0x37);
    entry(2, 64, blocks - 64, b"MacOS", b"Apple_HFS", 0x33);
    fs::write(path, &d).unwrap();
    let hformat = home("~/Retro68-build/toolchain/bin/hformat");
    let ok = Command::new(&hformat).args(["-l", label]).arg(path).arg("1").output().is_ok_and(|o| o.status.success());
    if ok {
        println!("  {} ({} KB)", path.file_name().unwrap().to_string_lossy(), d.len() / 1024);
    } else {
        println!("  ({} skipped: {hformat} failed)", path.display());
        let _ = fs::remove_file(path);
    }
    ok
}

/// The GS/OS image: a `.po`, or the zip holding `smart0.po`.
fn gsos(path: &str) -> Option<Vec<u8>> {
    if path.ends_with(".zip") {
        let o = Command::new("unzip").args(["-p", path, "smart0.po"]).output().ok()?;
        (o.status.success() && o.stdout.len() > 1024).then_some(o.stdout)
    } else {
        fs::read(path).ok()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str, default: &str| args.iter().position(|a| a == name).map_or(home(default), |i| home(&args[i + 1]));
    let out = PathBuf::from(arg("--out", "images/Benchmarks"));
    let blank = PathBuf::from(arg("--blank", "images/Blank"));
    let fbench = PathBuf::from(arg("--floppybench", "tools/floppybench/build/FloppyBench.dsk"));
    let a2bench = arg("--a2bench", "tools/a2bench/build/BENCH.SYSTEM");
    let boot_src = arg("--boot", "images/Apple IIgs test/31 IIgs System Disk 1.1.po");
    let gsos_src = arg("--gsos", "~/Downloads/gsos-6.0.2.zip");

    let bench = fs::read(&a2bench).ok().map(|f| prodos::apple_single(&f).unwrap_or((f, 0xFF, 0x2000)));
    let boot = fs::read(&boot_src).ok().filter(|b| b.len() >= 2 * BLOCK);
    let gs = gsos(&gsos_src);
    let p8 = gs.as_ref().and_then(|g| {
        let img = prodos::Image::from_bytes(g.clone(), "GS/OS image");
        img.find("SYSTEM/P8").map(|e| img.read(&e))
    });

    // ---- Mac floppies ----
    let dir = out.join("1 Mac floppy");
    fs::create_dir_all(&dir).unwrap();
    println!("{}:", dir.display());
    copy(&dir, "00 FloppyBench.dsk", &fbench);
    for (n, size, kinds) in [
        (10, "400K", &["dsk", "image", "moof"][..]),
        (20, "800K", &["dsk", "image", "moof"][..]),
        (30, "720K", &["dsk", "image"][..]),
        (40, "1.44MB", &["dsk", "image", "moof"][..]),
    ] {
        for (k, ext) in kinds.iter().enumerate() {
            copy(&dir, &format!("{} Blank {size}.{ext}", n + k), &blank.join(format!("Blank {size}.{ext}")));
        }
    }
    if let Ok(raw) = fs::read(blank.join("Blank 800K.dsk")) {
        put(&dir, "23 Blank 800K flux.moof", &mac_flux_moof(&raw));
    }

    // ---- HD20 ----
    let dir = out.join("2 Mac HD20");
    fs::create_dir_all(&dir).unwrap();
    println!("{}:", dir.display());
    copy(&dir, "01 Blank HD20 20MB.hda", &blank.join("Blank HD20 20MB.hda"));
    copy(&dir, "02 Blank HD20 100MB.hda", &blank.join("Blank HD20 100MB.hda"));
    apm_hfs(40_960, "HD20 APM", &dir.join("03 Blank HD20 20MB partitioned.hda"));

    // ---- Apple II 5.25" ----
    let dir = out.join("3 Apple II 5.25");
    fs::create_dir_all(&dir).unwrap();
    println!("{}:", dir.display());
    match (&boot, &p8, &bench) {
        (Some(b), Some(p), Some(x)) => put(&dir, "01 A2Bench boot 140K.po", &bootable(280, "A2BENCH", b, p, x)),
        _ => println!("  (01 A2Bench boot 140K.po skipped: needs --a2bench, --boot and --gsos)"),
    }
    let po = volume(280, "BENCH140");
    put(&dir, "10 Bench 140K.po", &po);
    put(&dir, "11 Bench 140K.dsk", &dos_order(&po));
    put(&dir, "12 Bench 140K.do", &dos_order(&po));
    put(&dir, "13 Bench 140K.2mg", &disks::two_mg(&po));
    put(&dir, "14 Bench 140K.woz", &disks::woz525(&po, true, 51_200, 32));
    put(&dir, "20 Bench 140K RO.nib", &nib(&po));
    put(&dir, "21 Bench 140K RO WOZ1.woz", &woz1(&po));
    put(&dir, "22 Bench 140K RO flux.woz", &woz_flux(&po));
    put(&dir, "23 Bench 140K RO.a2r", &a2r(&po));

    // ---- Apple II 3.5" ----
    let dir = out.join("4 Apple II 3.5");
    fs::create_dir_all(&dir).unwrap();
    println!("{}:", dir.display());
    match (&boot, &p8, &bench) {
        (Some(b), Some(p), Some(x)) => put(&dir, "01 A2Bench boot 800K.po", &bootable(1600, "A2BENCH", b, p, x)),
        _ => println!("  (01 A2Bench boot 800K.po skipped: needs --a2bench, --boot and --gsos)"),
    }
    let po = volume(1600, "BENCH800");
    put(&dir, "10 Bench 800K.po", &po);
    put(&dir, "11 Bench 800K.2mg", &disks::two_mg(&po));
    put(&dir, "12 Bench 800K.dc42", &disks::diskcopy(&po, None, "BENCH800"));
    let d = disks::mac_disk(&po, true, Fill::Sync, disks::varied_ends(11), |_, _, _| Odd::Normal);
    put(&dir, "13 Bench 800K.moof", &disks::moof(&d));
    put(&dir, "14 Bench 800K.woz", &disks::woz35(&d));
    put(&dir, "15 Bench 400K.po", &volume(800, "BENCH400"));

    // ---- SmartPort ----
    let dir = out.join("5 SmartPort");
    fs::create_dir_all(&dir).unwrap();
    println!("{}:", dir.display());
    match (&gs, &bench) {
        (Some(g), Some(x)) => {
            let mut img = prodos::Image::from_bytes(g.clone(), "GS/OS image");
            img.put("BENCH.SYSTEM", x.1, x.2, &x.0);
            put(&dir, "01 GSOS 6.0.2 with A2Bench.po", &img.bytes);
        }
        _ => println!("  (01 GSOS 6.0.2 with A2Bench.po skipped: needs --gsos and --a2bench)"),
    }
    match (&boot, &p8, &bench) {
        (Some(b), Some(p), Some(x)) => put(&dir, "02 A2Bench boot 800K.po", &bootable(1600, "A2BENCH", b, p, x)),
        _ => {}
    }
    put(&dir, "10 Bench 800K.po", &volume(1600, "BENCH800"));
    let big = volume(65_535, "BENCH32M");
    put(&dir, "11 Bench 32MB.po", &big);
    put(&dir, "12 Bench 32MB.hdv", &big);
    put(&dir, "13 Bench 32MB.2mg", &disks::two_mg(&big));

    println!("done: {}", out.display());
}
