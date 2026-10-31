//! Host-side check of HD20 (DCD) support: `src/dcd.rs` (7-to-8 groups, decoding the Mac's
//! transfers as `capture.pio` captures them, replies, the command session, finding the volume)
//! and `src/clusters.rs` (mapping a file through its FAT).
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/dcd_check.rs -o target/dcd_check && target/dcd_check

#[path = "../../src/clusters.rs"]
mod clusters;
#[path = "../../src/dcd.rs"]
mod dcd;
mod pio_sim;

use dcd::{BLOCK, MAX_ENCODED, MAX_PAYLOAD, Received, Session, Storage};

fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

// ---- 7-to-8 groups -----------------------------------------------------------------------

fn check_groups() {
    let data = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37];
    let sent = dcd::encode_group(&data);
    assert_eq!(sent, [0x98, 0x99, 0x99, 0x9A, 0x9A, 0x9B, 0x9B, 0xD5]);
    // The Mac sends the low bits first.
    let mut mac = [0u8; 8];
    mac[0] = sent[7];
    mac[1..].copy_from_slice(&sent[..7]);
    assert_eq!(dcd::decode_group(&mac), data);
    let mut s = 7u64;
    for _ in 0..10_000 {
        let d: [u8; 7] = core::array::from_fn(|_| rng(&mut s) as u8);
        let e = dcd::encode_group(&d);
        assert!(e.iter().all(|&b| b & 0x80 != 0));
        let mut m = [0u8; 8];
        m[0] = e[7];
        m[1..].copy_from_slice(&e[..7]);
        assert_eq!(dcd::decode_group(&m), d);
    }
    assert_eq!(dcd::checksum(&[1, 2, 3]), 0u8.wrapping_sub(6));
    println!("groups: ok");
}

// ---- The Mac's side ----------------------------------------------------------------------

/// A command transfer as the Mac's IWM writes it: `AA`, the group counts, the groups (low bits
/// first), with a holdoff after each group in `holdoffs`.
struct MacWrite {
    payload: Vec<u8>,
    reply_groups: usize,
    holdoffs: Vec<usize>,
    /// WR's level before the transfer, and whether the Mac brings it low before starting.
    starts_high: bool,
    pulls_low_first: bool,
}

/// PIO cycles per cell for the capture program (32x the cell rate).
const CELL: f64 = 32.0;

/// WR edges (PIO cycles) for `w`: NRZI, an edge per 1. `speed`/`jitter` as in write_check.
struct Wire {
    s: u64,
    cell: f64,
    jitter: u64,
    t: f64,
    level: bool,
    edges: Vec<u64>,
}

impl Wire {
    fn edge(&mut self) {
        let j = (rng(&mut self.s) % (2 * self.jitter + 1)) as f64 - self.jitter as f64;
        self.edges.push((self.t + j) as u64);
        self.level = !self.level;
    }

    fn bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            for k in (0..8).rev() {
                if b >> k & 1 != 0 {
                    self.edge();
                }
                self.t += self.cell;
            }
        }
    }
}

fn waveform(w: &MacWrite, speed: f64, jitter: u64, seed: u64) -> (bool, Vec<u64>, u64) {
    // The gate is open a while before the Mac writes.
    let mut wire = Wire { s: seed | 1, cell: CELL * (1.0 + speed), jitter, t: 300.0, level: w.starts_high, edges: vec![] };
    if w.starts_high && w.pulls_low_first {
        wire.edge();
        wire.t += 3.3 * wire.cell;
    }
    assert!(w.payload.len() % 7 == 0);
    let groups = w.payload.len() / 7;
    wire.bytes(&[0xAA, 0x80 | groups as u8, 0x80 | w.reply_groups as u8]);
    for g in 0..groups {
        let enc = dcd::encode_group(&w.payload[7 * g..7 * g + 7]);
        let mut raw = [0u8; 8];
        raw[0] = enc[7];
        raw[1..].copy_from_slice(&enc[..7]);
        wire.bytes(&raw);
        if w.holdoffs.contains(&g) && g + 1 < groups {
            // Serve an interrupt, bring WR low if it is high, then resume with AA.
            wire.t += (20 + rng(&mut wire.s) % 400) as f64 * wire.cell;
            if wire.level {
                wire.edge();
                wire.t += (1 + rng(&mut wire.s) % 6) as f64 * wire.cell + 0.4 * wire.cell;
            }
            wire.bytes(&[0xAA]);
        }
    }
    wire.edges.sort();
    (w.starts_high, wire.edges, wire.t as u64 + 40 * 32)
}

fn capture(w: &MacWrite, speed: f64, jitter: u64, seed: u64) -> Vec<bool> {
    let src = std::fs::read_to_string("src/capture.pio").unwrap();
    let prog = pio_sim::parse(&src, "gcr_write");
    let (high, edges, end) = waveform(w, speed, jitter, seed);
    let pin = |t: u64| {
        let k = edges.partition_point(|&e| e <= t);
        (k % 2 == 0) == high
    };
    pio_sim::run(&prog, end + 64 * 32, pin)
}

fn command_payload(id: u8, count: u8, block: u32, data: Option<&[u8; BLOCK]>) -> Vec<u8> {
    let mut p = vec![id, count, (block >> 16) as u8, (block >> 8) as u8, block as u8, 0];
    if let Some(d) = data {
        p.extend([0u8; 20]);
        p.extend_from_slice(d);
    }
    p.push(0);
    while p.len() % 7 != 0 {
        p.insert(p.len() - 1, 0);
    }
    let n = p.len();
    p[n - 1] = dcd::checksum(&p[..n - 1]);
    p
}

fn check_receive() {
    let mut s = 0xDCDu64;
    let mut cases = 0;
    let mut data = [0u8; BLOCK];
    for b in data.iter_mut() {
        *b = rng(&mut s) as u8;
    }
    for (payload, replies) in [(command_payload(0x00, 3, 0x01_2345, None), 77), (command_payload(0x01, 1, 77, Some(&data)), 1), (command_payload(0x03, 0, 0, None), 49)] {
        let groups = payload.len() / 7;
        let holdoff_sets: Vec<Vec<usize>> = if groups > 3 { vec![vec![], vec![0], vec![1, 5, 40, groups - 2], (0..groups).step_by(3).collect()] } else { vec![vec![], vec![0]] };
        for holdoffs in holdoff_sets {
            for (starts_high, pulls_low_first) in [(false, false), (true, true), (true, false)] {
                // Both ends are crystal clocks (far under 0.1% apart) and the IWM puts edges on FCLK, so
                // ±0.5% and ±3 PIO cycles is generous. DCD bytes have runs of up to 7 zeros (GCR 2), and
                // the separator spaces its 0s 0.97 cells apart, which is what limits the speed.
                for (speed, jitter) in [(0.0, 0), (-0.005, 2), (0.005, 2), (0.0, 3), (-0.01, 1), (0.01, 1)] {
                    let w = MacWrite { payload: payload.clone(), reply_groups: replies, holdoffs: holdoffs.clone(), starts_high, pulls_low_first };
                    let cells = capture(&w, speed, jitter, rng(&mut s));
                    let mut out = [0u8; MAX_PAYLOAD];
                    let rx = dcd::receive(&|i| cells[i], cells.len(), &mut out).expect("no command");
                    let want = Received { len: payload.len(), reply_groups: replies, ok: true };
                    assert_eq!(rx, want, "holdoffs {:?} high {} pull {} speed {} jitter {}", holdoffs, starts_high, pulls_low_first, speed, jitter);
                    assert_eq!(&out[..payload.len()], &payload[..]);
                    cases += 1;
                }
            }
        }
    }
    // A corrupted byte: decoded, but not ok (the drive NAKs it).
    let mut p = command_payload(0x00, 1, 5, None);
    p[2] ^= 0x10;
    let w = MacWrite { payload: p, reply_groups: 77, holdoffs: vec![], starts_high: false, pulls_low_first: false };
    let cells = capture(&w, 0.0, 2, 99);
    let mut out = [0u8; MAX_PAYLOAD];
    let rx = dcd::receive(&|i| cells[i], cells.len(), &mut out).unwrap();
    assert!(!rx.ok && rx.reply_groups == 77);
    // Cut short: the length bytes arrived, not every group.
    let w = MacWrite { payload: command_payload(0x01, 1, 77, Some(&data)), reply_groups: 1, holdoffs: vec![], starts_high: false, pulls_low_first: false };
    let cells = capture(&w, 0.0, 2, 98);
    let rx = dcd::receive(&|i| cells[i], cells.len() / 2, &mut out).unwrap();
    assert!(!rx.ok && rx.reply_groups == 1);
    // Nothing at all.
    assert!(dcd::receive(&|_| false, 5_000, &mut out).is_err());
    println!("receive: {} captured transfers decoded", cases);
}

/// What the Mac's IWM makes of a reply as core 1 sends it (`AA`, then the groups; a holdoff
/// before group `g` in `holdoffs` adds an `AA`): the payload, and its checksum holds.
fn mac_read(encoded: &[u8], holdoffs: &[usize]) -> Vec<u8> {
    let mut stream = vec![0xAA];
    for (g, group) in encoded.chunks(8).enumerate() {
        if holdoffs.contains(&g) {
            stream.push(0xAA);
        }
        stream.extend_from_slice(group);
    }
    // As cells, with idle zeros the IWM skips; then IWM framing.
    let mut cells = vec![false; 13];
    for b in &stream {
        cells.extend((0..8).rev().map(|k| b >> k & 1 != 0));
        cells.extend(std::iter::repeat_n(false, (*b as usize) % 3));
    }
    let mut bytes = vec![];
    let mut sh = 0u8;
    for c in cells {
        if sh == 0 && !c {
            continue;
        }
        sh = sh << 1 | c as u8;
        if sh & 0x80 != 0 {
            bytes.push(sh);
            sh = 0;
        }
    }
    assert_eq!(bytes[0], 0xAA);
    let mut payload = vec![];
    let mut i = 1;
    let mut g = 0;
    while i < bytes.len() {
        if holdoffs.contains(&g) {
            assert_eq!(bytes[i], 0xAA);
            i += 1;
        }
        let raw = &bytes[i..i + 8];
        let mut mac = [0u8; 8];
        mac[0] = raw[7];
        mac[1..].copy_from_slice(&raw[..7]);
        payload.extend(dcd::decode_group(&mac));
        i += 8;
        g += 1;
    }
    assert_eq!(payload.iter().fold(0u8, |s, &b| s.wrapping_add(b)), 0, "reply checksum");
    payload
}

// ---- A whole session ---------------------------------------------------------------------

struct Mem {
    data: Vec<u8>,
    writable: bool,
    fail_read: Option<u32>,
}

impl Storage for Mem {
    fn blocks(&self) -> u32 {
        (self.data.len() / BLOCK) as u32
    }
    fn writable(&self) -> bool {
        self.writable
    }
    fn read(&mut self, block: u32, out: &mut [u8; BLOCK]) -> Result<(), ()> {
        if self.fail_read == Some(block) {
            return Err(());
        }
        let at = block as usize * BLOCK;
        out.copy_from_slice(&self.data[at..at + BLOCK]);
        Ok(())
    }
    fn write(&mut self, block: u32, data: &[u8; BLOCK]) -> Result<(), ()> {
        let at = block as usize * BLOCK;
        self.data[at..at + BLOCK].copy_from_slice(data);
        Ok(())
    }
}

/// Send `payload` to the session as received (`ok` or not) and collect every reply it makes.
fn exchange(session: &mut Session, disk: &mut Mem, payload: &[u8], reply_groups: usize, ok: bool) -> Vec<Vec<u8>> {
    let rx = Received { len: payload.len(), reply_groups, ok };
    let mut out = [0u8; MAX_ENCODED];
    let mut replies = vec![];
    let n = session.command(&rx, payload, disk, &mut out);
    if n > 0 {
        assert!(!session.reading());
        replies.push(mac_read(&out[..n], &[]));
    }
    let mut k = 0;
    while let Some(n) = session.next_reply(disk, &mut out) {
        assert_eq!(n, 8 * reply_groups);
        // Holdoffs in some replies, as the Mac makes them.
        let h: Vec<usize> = if k % 2 == 1 { vec![1, 30, 76] } else { vec![] };
        replies.push(mac_read(&out[..n], &h));
        k += 1;
    }
    replies
}

fn check_session() {
    let mut s = 0x5E55u64;
    let blocks = 2_000;
    let mut disk = Mem { data: (0..blocks * BLOCK).map(|_| rng(&mut s) as u8).collect(), writable: true, fail_read: None };
    let original = disk.data.clone();
    let mut session = Session::new();

    // Controller status: size, characteristics, icon, location.
    let r = exchange(&mut session, &mut disk, &command_payload(0x03, 0, 0, None), 49, true);
    assert_eq!(r.len(), 1);
    let st = &r[0];
    assert_eq!(st.len(), 343);
    assert_eq!(st[0], 0x83);
    assert_eq!(&st[2..6], &[0, 0, 0, 0]);
    assert_eq!(&st[6..10], &[0, 1, 0, 1]);
    assert_eq!(st[10], 0xE6);
    assert_eq!(&st[11..14], &[0, (blocks >> 8) as u8, blocks as u8]);
    assert!(st[70..198].iter().any(|&b| b != 0) && st[198..326].iter().any(|&b| b != 0));
    for i in 0..128 {
        assert_eq!(st[70 + i] & !st[198 + i], 0, "icon pixel outside its mask");
    }
    assert_eq!(st[326] as usize, b"rp2350-floppy".len());
    assert_eq!(&st[327..340], b"rp2350-floppy");

    // Read three blocks: three replies counting down.
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 3, 1_234, None), 77, true);
    assert_eq!(r.len(), 3);
    for (i, rep) in r.iter().enumerate() {
        assert_eq!(rep.len(), 539);
        assert_eq!(&rep[..6], &[0x80, 3 - i as u8, 0, 0, 0, 0]);
        let at = (1_234 + i) * BLOCK;
        assert_eq!(&rep[26..538], &original[at..at + BLOCK]);
    }

    // Write two blocks: the first command has the address, the continuation follows it.
    let a: [u8; BLOCK] = core::array::from_fn(|i| i as u8);
    let b: [u8; BLOCK] = core::array::from_fn(|i| !(i as u8));
    let r = exchange(&mut session, &mut disk, &command_payload(0x01, 2, 1_500, Some(&a)), 1, true);
    assert_eq!(r, vec![vec![0x81, 2, 0, 0, 0, 0, 0u8.wrapping_sub(0x83)]]);
    let r = exchange(&mut session, &mut disk, &command_payload(0x41, 1, 0, Some(&b)), 1, true);
    assert_eq!(r[0][..6], [0x81, 1, 0, 0, 0, 0]);
    assert_eq!(&disk.data[1_500 * BLOCK..1_501 * BLOCK], &a);
    assert_eq!(&disk.data[1_501 * BLOCK..1_502 * BLOCK], &b);
    // Write-and-verify too.
    let r = exchange(&mut session, &mut disk, &command_payload(0x02, 1, 7, Some(&b)), 1, true);
    assert_eq!(r[0][..3], [0x82, 1, 0]);
    assert_eq!(&disk.data[7 * BLOCK..8 * BLOCK], &b);
    assert_eq!((session.blocks_read, session.blocks_written), (3, 3));
    // Read it back.
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 2, 1_500, None), 77, true);
    assert_eq!(&r[0][26..538], &a);
    assert_eq!(&r[1][26..538], &b);
    // Nothing else changed.
    for blk in 0..blocks {
        if [7, 1_500, 1_501].contains(&blk) {
            continue;
        }
        assert_eq!(&disk.data[blk * BLOCK..(blk + 1) * BLOCK], &original[blk * BLOCK..(blk + 1) * BLOCK]);
    }

    // Errors: out of range, a continuation with no write before it, a read-only disk, a card error.
    let errors = session.errors;
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 2, blocks as u32 - 1, None), 77, true);
    assert!(r.len() == 1 && r[0][..3] == [0x80, 2, 0x80]);
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 1, 5, None), 77, true);
    assert_eq!(r[0][2], 0);
    let r = exchange(&mut session, &mut disk, &command_payload(0x41, 1, 0, Some(&a)), 1, true);
    assert_eq!(r[0][..3], [0x81, 1, 0x80]);
    disk.writable = false;
    let r = exchange(&mut session, &mut disk, &command_payload(0x01, 1, 3, Some(&a)), 1, true);
    assert_eq!(r[0][2], 0x80);
    let r = exchange(&mut session, &mut disk, &command_payload(0x03, 0, 0, None), 49, true);
    assert_eq!(r[0][10], 0xCE);
    disk.writable = true;
    disk.fail_read = Some(11);
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 4, 9, None), 77, true);
    assert_eq!(r.len(), 3);
    assert_eq!(r[2][..3], [0x80, 2, 0x80]);
    disk.fail_read = None;
    assert_eq!(session.errors, errors + 4);

    // A garbled command gets a NAK of the size the Mac asked for; an unknown one an empty success.
    let r = exchange(&mut session, &mut disk, &command_payload(0x00, 1, 5, None), 77, false);
    assert_eq!(r[0].len(), 539);
    assert_eq!(r[0][0], 0x7F);
    let r = exchange(&mut session, &mut disk, &command_payload(0x19, 1, 0, None), 1, true);
    assert_eq!(r[0][..6], [0x99, 0, 0, 0, 0, 0]);
    assert_eq!(session.last, dcd::Last::Other(0x19));
    println!("session: ok");
}

// ---- Finding the volume ------------------------------------------------------------------

fn volume_of(file: &[u8]) -> Result<dcd::Volume, &'static str> {
    dcd::find_volume((file.len() / BLOCK) as u32, |b, out| {
        let at = b as usize * BLOCK;
        out.copy_from_slice(file.get(at..at + BLOCK).ok_or("past the end")?);
        Ok(())
    })
}

fn check_volumes() {
    // An 800K HFS volume (its master directory block at block 2) and a 400K MFS one.
    for (blocks, signature, kind) in [(1600, *b"BD", "HFS"), (800, [0xD2, 0xD7], "MFS")] {
        let mut file = vec![0u8; blocks * BLOCK];
        file[2 * BLOCK..2 * BLOCK + 2].copy_from_slice(&signature);
        let v = volume_of(&file).unwrap();
        assert_eq!(v, dcd::Volume { first: 0, blocks: blocks as u32, kind });
    }
    // An Apple partition map: driver descriptor, map entries, the HFS partition at 96.
    let mut file = vec![0u8; 400 * BLOCK];
    file[..4].copy_from_slice(&[b'E', b'R', 0x02, 0x00]);
    let entry = |file: &mut Vec<u8>, i: usize, start: u32, len: u32, kind: &[u8]| {
        let e = &mut file[i * BLOCK..(i + 1) * BLOCK];
        e[..2].copy_from_slice(b"PM");
        e[4..8].copy_from_slice(&3u32.to_be_bytes());
        e[8..12].copy_from_slice(&start.to_be_bytes());
        e[12..16].copy_from_slice(&len.to_be_bytes());
        e[48..48 + kind.len()].copy_from_slice(kind);
    };
    entry(&mut file, 1, 1, 63, b"Apple_partition_map");
    entry(&mut file, 2, 64, 32, b"Apple_Driver43");
    entry(&mut file, 3, 96, 304, b"Apple_HFS");
    file[98 * BLOCK..98 * BLOCK + 2].copy_from_slice(b"BD");
    assert_eq!(volume_of(&file), Ok(dcd::Volume { first: 96, blocks: 304, kind: "APM HFS" }));
    entry(&mut file, 3, 96, 305, b"Apple_HFS");
    assert!(volume_of(&file).is_err());
    entry(&mut file, 3, 96, 304, b"Apple_Free");
    assert!(volume_of(&file).is_err());
    file[..2].copy_from_slice(&[0, 0]);
    assert!(volume_of(&file).is_err());
    file[2 * BLOCK..2 * BLOCK + 2].copy_from_slice(b"H+");
    assert!(volume_of(&file).is_err());
    println!("volumes: ok");
}

// ---- Cluster chains ----------------------------------------------------------------------

/// A card image with a FAT volume: MBR, boot sector, FATs (FAT16 or FAT32), and clusters of
/// `per_cluster` sectors. Returns the image and the layout the test expects.
fn fat_card(fat32: bool, per_cluster: u8, clusters: u32) -> (Vec<u8>, clusters::Layout) {
    let start = 63u32;
    let reserved = 32u32;
    let fats = 2u32;
    let entry = if fat32 { 4 } else { 2 };
    let fat_sectors = ((clusters + 2) * entry).div_ceil(BLOCK as u32);
    let root_sectors = if fat32 { 0 } else { 32 };
    let data = reserved + fats * fat_sectors + root_sectors;
    let total = data + clusters * u32::from(per_cluster);
    let mut card = vec![0u8; ((start + total) as usize) * BLOCK];
    card[446 + 4] = if fat32 { 0x0C } else { 0x06 };
    card[446 + 8..446 + 12].copy_from_slice(&start.to_le_bytes());
    card[510] = 0x55;
    card[511] = 0xAA;
    let b = &mut card[start as usize * BLOCK..][..BLOCK];
    b[11..13].copy_from_slice(&512u16.to_le_bytes());
    b[13] = per_cluster;
    b[14..16].copy_from_slice(&(reserved as u16).to_le_bytes());
    b[16] = fats as u8;
    b[17..19].copy_from_slice(&(if fat32 { 0u16 } else { 512 }).to_le_bytes());
    if total < 65_536 {
        b[19..21].copy_from_slice(&(total as u16).to_le_bytes());
    } else {
        b[32..36].copy_from_slice(&total.to_le_bytes());
    }
    if fat32 {
        b[36..40].copy_from_slice(&fat_sectors.to_le_bytes());
    } else {
        b[22..24].copy_from_slice(&(fat_sectors as u16).to_le_bytes());
    }
    let layout = clusters::Layout {
        fat: start + reserved,
        heap: start + data,
        per_cluster: u32::from(per_cluster),
        clusters,
        kind: if fat32 { clusters::Fat::Fat32 } else { clusters::Fat::Fat16 },
    };
    (card, layout)
}

fn set_fat(card: &mut [u8], layout: &clusters::Layout, cluster: u32, next: u32) {
    let bytes = if layout.kind == clusters::Fat::Fat16 { 2 } else { 4 };
    let at = layout.fat as usize * BLOCK + (cluster * bytes) as usize;
    if bytes == 2 {
        card[at..at + 2].copy_from_slice(&(next as u16).to_le_bytes());
    } else {
        card[at..at + 4].copy_from_slice(&next.to_le_bytes());
    }
}

fn walk(card: &[u8], layout: &clusters::Layout, first: u32, blocks: u32, contiguous: bool) -> Result<Vec<(u32, u32)>, &'static str> {
    let mut runs = vec![];
    clusters::walk(
        layout,
        first,
        blocks,
        contiguous,
        |lba, out| {
            out.copy_from_slice(&card[lba as usize * BLOCK..][..BLOCK]);
            Ok(())
        },
        |lba, n| {
            runs.push((lba, n));
            Ok(())
        },
    )?;
    Ok(runs)
}

fn check_clusters() {
    let mut s = 0xFA7u64;
    for (fat32, per_cluster, count) in [(false, 4u8, 20_000u32), (true, 1, 70_000), (true, 8, 70_000)] {
        let (mut card, want) = fat_card(fat32, per_cluster, count);
        let got = clusters::Layout::fat(|lba, out| {
            out.copy_from_slice(&card[lba as usize * BLOCK..][..BLOCK]);
            Ok(())
        })
        .unwrap();
        assert_eq!(got, want);
        let l = got;
        let end = if fat32 { 0x0FFF_FFFF } else { 0xFFFF };
        // A fragmented chain: pieces of random length in random places (not overlapping).
        for trial in 0..20 {
            let mut chain = vec![];
            let mut next_free = 2 + (rng(&mut s) % 100) as u32;
            while chain.len() < 300 {
                let len = 1 + rng(&mut s) % 12;
                for _ in 0..len {
                    chain.push(next_free);
                    next_free += 1;
                }
                next_free += (rng(&mut s) % 3) as u32 * (1 + (rng(&mut s) % 40) as u32);
            }
            for w in chain.windows(2) {
                set_fat(&mut card, &l, w[0], w[1]);
            }
            set_fat(&mut card, &l, *chain.last().unwrap(), end);
            let blocks = (chain.len() as u32 - 1) * l.per_cluster + 1 + (rng(&mut s) as u32 % l.per_cluster);
            let runs = walk(&card, &l, chain[0], blocks, false).unwrap();
            // Expand the runs and compare with the chain, sector by sector.
            let sectors: Vec<u32> = runs.iter().flat_map(|&(lba, n)| lba..lba + n).collect();
            let want: Vec<u32> = chain.iter().flat_map(|&c| l.sector(c)..l.sector(c) + l.per_cluster).take(blocks as usize).collect();
            assert_eq!(sectors, want, "trial {}", trial);
            assert!(runs.windows(2).all(|w| w[0].0 + w[0].1 != w[1].0), "runs not merged");
            assert_eq!(l.cluster_at(l.sector(chain[0])), Some(chain[0]));
            // A chain that ends too soon.
            set_fat(&mut card, &l, chain[100], end);
            assert!(walk(&card, &l, chain[0], blocks, false).is_err());
            // A loop.
            set_fat(&mut card, &l, chain[100], chain[50]);
            assert!(walk(&card, &l, chain[0], blocks, false).is_err());
            for &c in &chain {
                set_fat(&mut card, &l, c, 0);
            }
        }
        assert_eq!(l.cluster_at(l.sector(5) + 1), if l.per_cluster == 1 { Some(6) } else { None });
    }
    // exFAT: contiguous files need no FAT; chains are 32-bit.
    let l = clusters::Layout::exfat(2048, 2048, 4096, 64, 30_000);
    let card = vec![0u8; 8_192 * BLOCK];
    assert_eq!(walk(&card, &l, 10, 1_000_000, true).unwrap(), vec![(l.sector(10), 1_000_000)]);
    assert!(walk(&card, &l, 29_990, 1_000_000, true).is_err());
    let mut card = card;
    let fat = l.fat as usize * BLOCK;
    for (c, n) in [(3u32, 4u32), (4, 9), (9, 0xFFFF_FFFF)] {
        card[fat + 4 * c as usize..][..4].copy_from_slice(&n.to_le_bytes());
    }
    assert_eq!(walk(&card, &l, 3, 150, false).unwrap(), vec![(l.sector(3), 128), (l.sector(9), 22)]);
    println!("clusters: ok");
}

/// A short directory entry: `name` (11 bytes, as stored), a file.
fn short_entry(name: &[u8; 11]) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[..11].copy_from_slice(name);
    e[11] = 0x20;
    e
}

/// The long name in the entries just before short entry `at` of `dir` (entries of 32 bytes),
/// checked against the short entry's checksum, as an OS reads it.
fn long_name_before(dir: &[[u8; 32]], at: usize) -> Option<String> {
    let mut sum = 0u8;
    for &c in &dir[at][..11] {
        sum = (if sum & 1 != 0 { 0x80 } else { 0 }) + (sum >> 1) + c;
    }
    let mut units = vec![];
    for (n, e) in dir[..at].iter().rev().enumerate() {
        if e[11] != 0x0F || e[13] != sum || usize::from(e[0] & 0x1F) != n + 1 {
            return None;
        }
        for k in [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30] {
            units.push(u16::from_le_bytes([e[k], e[k + 1]]));
        }
        if e[0] & 0x40 != 0 {
            let end = units.iter().position(|&u| u == 0).unwrap_or(units.len());
            assert!(units[end..].iter().skip(1).all(|&u| u == 0xFFFF), "long name not padded with FFFF");
            return String::from_utf16(&units[..end]).ok();
        }
    }
    None
}

/// `clusters::add_root_file` on a card: the root's entries before and after.
fn add_root_file(card: &mut Vec<u8>, root: &[u32], long: &str) -> Result<([u8; 11], Vec<[u8; 32]>), &'static str> {
    let card = std::cell::RefCell::new(card);
    let short = clusters::add_root_file(
        |lba, out| {
            out.copy_from_slice(&card.borrow()[lba as usize * BLOCK..][..BLOCK]);
            Ok(())
        },
        |lba, data| {
            card.borrow_mut()[lba as usize * BLOCK..][..BLOCK].copy_from_slice(data);
            Ok(())
        },
        long,
    )?;
    let card = card.borrow();
    let dir = root.iter().flat_map(|&lba| card[lba as usize * BLOCK..][..BLOCK].chunks_exact(32)).map(|e| e.try_into().unwrap()).collect();
    Ok((short, dir))
}

fn check_root_file() {
    // FAT16: a fixed root of 32 sectors after the FATs.
    let (card, l) = fat_card(false, 4, 20_000);
    let first = l.heap - 32;
    let root: Vec<u32> = (first..l.heap).collect();
    let put = |card: &mut Vec<u8>, slot: usize, e: [u8; 32]| card[first as usize * BLOCK + slot * 32..][..32].copy_from_slice(&e);
    let mut card = card;
    let mut label = short_entry(b"SETTINGS   ");
    label[11] = 0x08;
    put(&mut card, 0, label);
    put(&mut card, 1, short_entry(b"SETTIN~1TXT"));
    let mut deleted = short_entry(b"OLD     TXT");
    deleted[0] = 0xE5;
    put(&mut card, 2, deleted);
    put(&mut card, 3, short_entry(b"SETTIN~3TXT"));
    put(&mut card, 4, deleted);
    put(&mut card, 5, short_entry(b"DISK    IMG"));
    // Free pairs: 6 and 7 (6 deleted, 7 the end).
    put(&mut card, 6, deleted);
    let before = card.clone();
    let (short, dir) = add_root_file(&mut card, &root, "settings.txt").unwrap();
    assert_eq!(&short, b"SETTIN~2TXT");
    assert_eq!(dir[7], dated(short_entry(b"SETTIN~2TXT")));
    assert_eq!(long_name_before(&dir, 7).as_deref(), Some("settings.txt"));
    // Nothing else changed, and the end is still marked after it.
    assert_eq!(dir[8][0], 0);
    let changed = (0..card.len() / 32).filter(|&i| card[i * 32..][..32] != before[i * 32..][..32]).count();
    assert_eq!(changed, 2);

    // A free pair across a sector boundary; a shorter name padded with FFFF.
    let mut card = before.clone();
    for slot in 6..15 {
        put(&mut card, slot, short_entry(b"FILLER  BIN"));
    }
    put(&mut card, 15, deleted);
    put(&mut card, 16, deleted);
    put(&mut card, 17, short_entry(b"AFTER   BIN"));
    let (short, dir) = add_root_file(&mut card, &root, "a.b").unwrap();
    assert_eq!(&short, b"A~1     B  ");
    assert_eq!(long_name_before(&dir, 16).as_deref(), Some("a.b"));
    assert_eq!(&dir[17][..11], b"AFTER   BIN");

    // A full root.
    let mut card = before.clone();
    for slot in 0..512 {
        put(&mut card, slot, short_entry(b"FULL    BIN"));
    }
    assert_eq!(add_root_file(&mut card, &root, "settings.txt").unwrap_err(), "FAT: root folder is full");
    // All of SETTIN~1..9 taken.
    let mut card = before.clone();
    for n in 1..=9u8 {
        let mut name = *b"SETTIN~1TXT";
        name[7] = b'0' + n;
        put(&mut card, 6 + usize::from(n), short_entry(&name));
    }
    assert_eq!(add_root_file(&mut card, &root, "settings.txt").unwrap_err(), "FAT: no short name left");
    assert!(add_root_file(&mut before.clone(), &root, "much too long.txt").is_err());

    // FAT32: the root is a cluster chain (here 2 → 7, two sectors a cluster), the first cluster full.
    let (mut card, l) = fat_card(true, 2, 70_000);
    card[63 * BLOCK + 44..][..4].copy_from_slice(&2u32.to_le_bytes());
    let fat = l.fat as usize * BLOCK;
    card[fat + 8..][..4].copy_from_slice(&7u32.to_le_bytes());
    card[fat + 28..][..4].copy_from_slice(&0x0FFF_FFFFu32.to_le_bytes());
    let root = vec![l.sector(2), l.sector(2) + 1, l.sector(7), l.sector(7) + 1];
    for slot in 0..32 {
        let lba = root[slot / 16] as usize;
        card[lba * BLOCK + slot % 16 * 32..][..32].copy_from_slice(&short_entry(b"FILLER  BIN"));
    }
    let (short, dir) = add_root_file(&mut card, &root, "settings.txt").unwrap();
    assert_eq!(&short, b"SETTIN~1TXT");
    assert_eq!(long_name_before(&dir, 33).as_deref(), Some("settings.txt"));
    assert_eq!(dir[34][0], 0);
    println!("root file: ok");
}

/// Short entry `e` with what `add_root_file` puts in besides the name: 1980-01-01 dates.
fn dated(mut e: [u8; 32]) -> [u8; 32] {
    for at in [16, 18, 24] {
        e[at..at + 2].copy_from_slice(&0x0021u16.to_le_bytes());
    }
    e
}

fn main() {
    check_groups();
    check_receive();
    check_session();
    check_volumes();
    check_clusters();
    check_root_file();
    println!("dcd_check: all passed");
}
