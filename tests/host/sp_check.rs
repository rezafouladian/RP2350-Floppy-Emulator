//! Host-side check of SmartPort support (`src/smartport.rs`): the packet codec against an
//! independent encoder framing the host's packets as `capture.pio`
//! captures them (run by `pio_sim` on simulated NRZI writes), and the commands against in-memory
//! units.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/sp_check.rs -o target/sp_check && target/sp_check

mod pio_sim;
#[path = "../../src/smartport.rs"]
mod smartport;

use core::sync::atomic::{AtomicU8, Ordering::Relaxed};
use smartport::{BLOCK, Framed, Framer, MAX_PACKET, MAX_REPLY, UNITS, Units};

fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// A packet from `C3` to `C8`, encoded (written here
/// separately from `smartport::encode`).
fn reference(dest: u8, src: u8, ptype: u8, stat: u8, data: &[u8]) -> Vec<u8> {
    let odd = data.len() % 7;
    let groups = data.len() / 7;
    let mut p = vec![0xC3, dest, src, ptype, 0x80, stat | 0x80, odd as u8 | 0x80, groups as u8 | 0x80];
    let mut sum = data.iter().fold(0u8, |s, b| s ^ b);
    for b in &p[1..8] {
        sum ^= b;
    }
    if odd > 0 {
        let mut msb = 0x80;
        for i in 0..odd {
            msb |= (data[i] & 0x80) >> (1 + i);
        }
        p.push(msb);
        p.extend(data[..odd].iter().map(|b| b | 0x80));
    }
    for g in 0..groups {
        let grp = &data[odd + 7 * g..odd + 7 * g + 7];
        let mut msb = 0x80;
        for (i, b) in grp.iter().enumerate() {
            msb |= (b >> (i + 1)) & (0x80 >> (i + 1));
        }
        p.push(msb);
        p.extend(grp.iter().map(|b| b | 0x80));
    }
    p.push(sum | 0xAA);
    p.push(sum >> 1 | 0xAA);
    p.push(0xC8);
    p
}

fn check_codec() {
    let mut s = 0x5EEDu64;
    let mut out = [0u8; MAX_REPLY];
    for n in 0..=smartport::MAX_DATA {
        let data: Vec<u8> = (0..n).map(|_| rng(&mut s) as u8).collect();
        let len = smartport::encode(&mut out, 0x81, smartport::TYPE_STATUS, 0x27, &data);
        assert_eq!(out[..6], smartport::SYNC);
        let p = &out[6..len];
        assert_eq!(p, &reference(0x80, 0x81, 0x81, 0x27, &data)[..], "{n} bytes");
        assert_eq!(p.len(), smartport::packet_len(p[6], p[7]));
        assert!(p.iter().all(|b| b & 0x80 != 0), "every byte has its top bit set");
        assert!(smartport::checksum_ok(p, false));
        let mut back = vec![0u8; n];
        assert_eq!(smartport::decode(p, &mut back), n);
        assert_eq!(back, data);
        if n > 0 {
            // A changed data bit is caught (a top-bits byte's unused bits change nothing).
            let mut bad = p.to_vec();
            let k = 8 + (rng(&mut s) as usize % (bad.len() - 11));
            bad[k] ^= 1 << (rng(&mut s) % 7);
            let mut changed = vec![0u8; n];
            smartport::decode(&bad, &mut changed);
            assert!(changed == data || !smartport::checksum_ok(&bad, false), "{n} bytes, byte {k} changed");
        }
    }
    assert!(MAX_PACKET >= reference(0x80, 0x81, 0x81, 0, &[0; smartport::MAX_DATA]).len());
    // An example command packet: READBLOCK of block 2 into $1800, with its
    // checksum worked out for that (the example's `BB EE` is for block 1).
    let cmd = reference(0x81, 0x80, 0x80, 0, &[0x01, 0x03, 0x00, 0x18, 0x02, 0x00, 0x00, 0x00, 0x00]);
    assert_eq!(cmd[..19], [0xC3, 0x81, 0x80, 0x80, 0x80, 0x80, 0x82, 0x81, 0x80, 0x81, 0x83, 0x80, 0x80, 0x98, 0x82, 0x80, 0x80, 0x80, 0x80]);
    let mut example = cmd.clone();
    example[14] = 0x81; // block 1
    example[19] = 0xBB;
    example[20] = 0xEE;
    assert!(smartport::checksum_ok(&example, false), "example checksum");
    // The IIc ROM 0 / Liron workaround: a 256–511 byte data packet with a bad checksum.
    let mut liron = reference(0x81, 0x80, 0x82, 0, &[0x55; 300]);
    let at = liron.len() - 3;
    liron[at] ^= 0x01;
    assert!(!smartport::checksum_ok(&liron, false) && smartport::checksum_ok(&liron, true));
    println!("codec: every length 0-767 equal to the reference, round trips, checksum, Liron leniency");
}

/// Frame `cells` until a packet comes out.
fn frame(cells: impl IntoIterator<Item = bool>) -> (Framed, Vec<u8>) {
    let mut f = Framer::new();
    let mut buf = [0u8; MAX_PACKET];
    for c in cells {
        match f.cell(c, &mut buf) {
            Framed::More => {}
            Framed::Packet(n) => return (Framed::Packet(n), buf[..n].to_vec()),
            Framed::Bad => return (Framed::Bad, vec![]),
        }
    }
    (Framed::More, vec![])
}

fn bits(bytes: &[u8]) -> Vec<bool> {
    bytes.iter().flat_map(|&b| (0..8).rev().map(move |k| b >> k & 1 != 0)).collect()
}

/// PIO cycles per cell for the capture program (32x the cell rate).
const CELL: f64 = 32.0;

/// The host writing `bytes` (sync and a packet) on WR, NRZI, captured by `gcr_write` running
/// `1 + speed` times slower than the host's cell, with ±`jitter` cycles per edge. WR may start
/// at either level and take a stray edge first, as when the IWM's write begins.
fn capture(bytes: &[u8], speed: f64, jitter: u64, seed: u64, start_high: bool, stray: bool) -> Vec<bool> {
    let src = std::fs::read_to_string("src/capture.pio").unwrap();
    let prog = pio_sim::parse(&src, "gcr_write");
    let mut s = seed | 1;
    let cell = CELL * (1.0 + speed);
    let mut t = 200.0;
    let mut edges = vec![];
    if stray {
        edges.push(t as u64);
        t += 2.5 * cell;
    }
    for b in bits(bytes) {
        if b {
            let j = (rng(&mut s) % (2 * jitter + 1)) as f64 - jitter as f64;
            edges.push((t + j) as u64);
        }
        t += cell;
    }
    let end = t as u64 + 40 * 32;
    let pin = |at: u64| {
        let k = edges.partition_point(|&e| e <= at);
        (k % 2 == 0) == start_high
    };
    pio_sim::run(&prog, end, pin)
}

fn check_framing() {
    let data = [0x01, 0x03, 0x00, 0x18, 0x02, 0x00, 0x00, 0x00, 0x00];
    let cmd = reference(0x81, 0x80, 0x80, 0, &data);
    let mut wire = smartport::SYNC.to_vec();
    wire.extend(&cmd);
    // Straight from bits, after a run of zeros and stray ones before the sync.
    let mut cells = vec![false; 37];
    cells.extend([true, false, true, true, false, false]);
    cells.extend(bits(&wire));
    let (r, got) = frame(cells);
    assert_eq!((r, &got), (Framed::Packet(cmd.len()), &cmd));
    // Not ending in C8.
    let mut bad = wire.clone();
    *bad.last_mut().unwrap() = 0xC9;
    assert_eq!(frame(bits(&bad)).0, Framed::Bad);

    // Through the separator: a command, and a write block's data packet (512 bytes, runs of up
    // to 7 zeros), at the host's speed give or take, with jitter.
    let mut s = 99u64;
    let block: Vec<u8> = (0..BLOCK).map(|_| rng(&mut s) as u8).collect();
    let data_pkt = reference(0x81, 0x80, 0x82, 0, &block);
    let mut runs = 0;
    for (pkt, what) in [(&cmd, "command"), (&data_pkt, "data")] {
        let mut wire = smartport::SYNC.to_vec();
        wire.extend(pkt.iter());
        for speed in [-0.01, 0.0, 0.01] {
            for jitter in [0, 2, 3] {
                for (high, stray) in [(true, false), (false, false), (true, true), (false, true)] {
                    let cells = capture(&wire, speed, jitter, s, high, stray);
                    let (r, got) = frame(cells);
                    assert_eq!(r, Framed::Packet(pkt.len()), "{what} at {speed}, jitter {jitter}, high {high} stray {stray}");
                    assert_eq!(&got, pkt);
                    assert!(smartport::checksum_ok(&got, false));
                    runs += 1;
                    rng(&mut s);
                }
            }
        }
    }
    // A reply as the host's IWM frames it.
    let mut out = [0u8; MAX_REPLY];
    let n = smartport::encode(&mut out, 0x82, smartport::TYPE_STATUS, 0, &block);
    let (r, got) = frame(bits(&out[..n]));
    assert_eq!(r, Framed::Packet(n - 6));
    let mut back = [0u8; BLOCK];
    smartport::decode(&got, &mut back);
    assert_eq!(&back[..], &block[..]);
    println!("framing: from bits, through gcr_write in {runs} runs (±1% speed, ±3 cycles, stray edges), a reply framed back");
}

/// Units in memory: `disks[u]` is unit `u`'s blocks.
struct Mem {
    disks: Vec<Option<Vec<[u8; BLOCK]>>>,
    writable: Vec<bool>,
    switched: Vec<bool>,
}

impl Units for Mem {
    fn blocks(&self, u: usize) -> Option<u32> {
        self.disks[u].as_ref().map(|d| d.len() as u32)
    }
    fn writable(&self, u: usize) -> bool {
        self.writable[u]
    }
    fn read(&mut self, u: usize, block: u32, out: &mut [u8; BLOCK]) -> Result<(), ()> {
        *out = self.disks[u].as_ref().ok_or(())?[block as usize];
        Ok(())
    }
    fn write(&mut self, u: usize, block: u32, data: &[u8; BLOCK]) -> Result<(), ()> {
        self.disks[u].as_mut().ok_or(())?[block as usize] = *data;
        Ok(())
    }
    fn eject(&mut self, u: usize) {
        self.disks[u] = None;
    }
    fn take_switched(&mut self, u: usize) -> bool {
        core::mem::take(&mut self.switched[u])
    }
    fn name(&self, _: usize) -> &[u8] {
        b"RP2350"
    }
}

/// Send `data` as a command packet to `dest` (with `extra` as its data packet, if any); returns
/// the reply's STAT and decoded data, or `None` for no reply.
fn ask(ids: &[AtomicU8; UNITS], units: &mut Mem, dest: u8, data: &[u8], extra: &[u8]) -> Option<(u8, Vec<u8>)> {
    let cmd = reference(dest, 0x80, 0x80, 0, data);
    assert!(smartport::checksum_ok(&cmd, false));
    let mut reply = [0u8; MAX_REPLY];
    let a = smartport::command(ids, &cmd, extra, units, &mut reply);
    if a.len == 0 {
        return None;
    }
    let p = &reply[6..a.len];
    assert!(smartport::checksum_ok(p, false));
    assert_eq!((p[1], p[3]), (0x80, smartport::TYPE_STATUS), "to the host, a status packet");
    assert_eq!(p[2], dest | 0x80, "from the unit asked");
    let mut d = vec![0u8; smartport::MAX_DATA];
    let n = smartport::decode(p, &mut d);
    d.truncate(n);
    Some((p[5] & 0x7F, d))
}

fn check_commands() {
    let ids: [AtomicU8; UNITS] = Default::default();
    let pattern = |b: usize| -> [u8; BLOCK] { core::array::from_fn(|i| (b * 7 + i) as u8) };
    let mut units = Mem {
        disks: vec![Some((0..1600).map(pattern).collect()), Some((0..20_000).map(pattern).collect()), None, Some(vec![[0; BLOCK]; 280])],
        writable: vec![true, true, true, false],
        switched: vec![false; UNITS],
    };
    // INIT: units take IDs 81–84 in turn; the last says it's the end of the chain.
    for (k, id) in (0x81..=0x84u8).enumerate() {
        let (stat, d) = ask(&ids, &mut units, id, &[smartport::INIT, 0x00, 0, 0, 0, 0, 0, 0, 0], &[]).expect("INIT answered");
        assert_eq!((stat, d.len()), (if k == UNITS - 1 { smartport::END_OF_CHAIN } else { 0 }, 0));
        assert_eq!(ids[k].load(Relaxed), id);
    }
    assert!(ask(&ids, &mut units, 0x85, &[smartport::INIT, 0, 0, 0, 0, 0, 0, 0, 0], &[]).is_none(), "nobody left for 85");
    assert!(ask(&ids, &mut units, 0x86, &[smartport::STATUS, 3, 0, 0, 0, 0, 0, 0, 0], &[]).is_none(), "not one of ours");

    // STATUS 0 and the DIB.
    let status = |units: &mut Mem, id: u8, code: u8| ask(&ids, units, id, &[smartport::STATUS, 3, 0, 0x18, code, 0, 0, 0, 0], &[]).unwrap();
    assert_eq!(status(&mut units, 0x81, 0), (0, vec![0xF8, 0x40, 0x06, 0x00]), "800K, online, writable");
    assert_eq!(status(&mut units, 0x83, 0), (0, vec![0xE8, 0, 0, 0]), "empty");
    assert_eq!(status(&mut units, 0x84, 0), (0, vec![0xFC, 0x18, 0x01, 0x00]), "280 blocks, write-protected");
    let (stat, dib) = status(&mut units, 0x81, 3);
    assert_eq!((stat, dib.len()), (0, 25));
    assert_eq!(&dib[4..21], b"\x10RP2350          ");
    assert_eq!(dib[21..23], [0x01, 0x40], "a 3.5\" disk, removable");
    assert_eq!(status(&mut units, 0x82, 3).1[21], 0x02, "20,000 blocks: a hard disk");
    assert_eq!(status(&mut units, 0x81, 7).0, smartport::BAD_CODE);
    units.switched[1] = true;
    assert_eq!(status(&mut units, 0x82, 0).1[0] & 1, 1, "switched, once");
    assert_eq!(status(&mut units, 0x82, 0).1[0] & 1, 0);

    // Blocks.
    let read = |units: &mut Mem, id: u8, b: u32| ask(&ids, units, id, &[smartport::READ_BLOCK, 3, 0, 0x18, b as u8, (b >> 8) as u8, (b >> 16) as u8, 0, 0], &[]).unwrap();
    assert_eq!(read(&mut units, 0x81, 1599), (0, pattern(1599).to_vec()));
    assert_eq!(read(&mut units, 0x82, 19_999), (0, pattern(19_999).to_vec()));
    assert_eq!(read(&mut units, 0x81, 1600).0, smartport::BAD_BLOCK);
    assert_eq!(read(&mut units, 0x83, 0).0, smartport::OFFLINE);
    let fresh = [0xA5u8; BLOCK];
    let write = |units: &mut Mem, id: u8, b: u32| {
        ask(&ids, units, id, &[smartport::WRITE_BLOCK, 3, 0, 0x18, b as u8, (b >> 8) as u8, 0, 0, 0], &fresh).unwrap().0
    };
    assert_eq!(write(&mut units, 0x82, 300), 0);
    assert_eq!(read(&mut units, 0x82, 300).1, fresh.to_vec());
    assert_eq!(write(&mut units, 0x84, 3), smartport::WRITE_PROTECTED);
    assert_eq!(write(&mut units, 0x83, 3), smartport::OFFLINE);
    assert_eq!(write(&mut units, 0x81, 2000), smartport::BAD_BLOCK);

    // CONTROL: eject, and other codes accepted. FORMAT, and what isn't supported.
    assert_eq!(ask(&ids, &mut units, 0x81, &[smartport::CONTROL, 3, 0, 0x18, 0x00, 0, 0, 0, 0], &[]).unwrap().0, 0);
    assert_eq!(ask(&ids, &mut units, 0x81, &[smartport::FORMAT, 1, 0, 0, 0, 0, 0, 0, 0], &[]).unwrap().0, 0);
    assert_eq!(ask(&ids, &mut units, 0x81, &[smartport::CONTROL, 3, 0, 0x18, 0x04, 0, 0, 0, 0], &[]).unwrap().0, 0);
    assert_eq!(read(&mut units, 0x81, 0).0, smartport::OFFLINE, "ejected");
    assert_eq!(ask(&ids, &mut units, 0x82, &[smartport::WRITE, 4, 0, 0, 0, 0, 0, 0, 0], &[]).unwrap().0, smartport::BAD_COMMAND);
    assert_eq!(ask(&ids, &mut units, 0x82, &[0x41, 4, 0, 0, 0, 0, 0, 0, 0], &[]).unwrap().0, smartport::BAD_COMMAND, "extended");
    assert!(smartport::has_data(smartport::WRITE_BLOCK) && smartport::has_data(smartport::CONTROL | smartport::EXTENDED));
    assert!(!smartport::has_data(smartport::READ_BLOCK) && !smartport::has_data(smartport::INIT));
    println!("commands: INIT down the chain, STATUS and DIB, read and write blocks, errors, eject");
}

/// The host's side of the handshake, run against `Handshake` pass by pass. Each transaction is
/// a list of steps; the device's outputs are checked at each.
struct Bus {
    hs: smartport::Handshake,
    enabled: bool,
    req: bool,
    /// The device's last outputs.
    out: smartport::Step,
    publishes: usize,
    sends: usize,
    aborts: usize,
    passes: usize,
}

impl Bus {
    fn new() -> Self {
        Bus { hs: smartport::Handshake::new(), enabled: true, req: false, out: Default::default(), publishes: 0, sends: 0, aborts: 0, passes: 0 }
    }

    /// One device pass with these inputs.
    fn pass(&mut self, rx: smartport::Rx, replied: Option<bool>, sent: bool) {
        self.out = self.hs.step(self.enabled, self.req, rx, replied, sent);
        self.publishes += self.out.publish as usize;
        self.sends += self.out.send as usize;
        self.aborts += self.out.abort as usize;
        self.passes += 1;
    }

    /// Passes until `done` (at most 50), with nothing new from the receive side or core 0.
    fn until(&mut self, what: &str, done: impl Fn(&Bus) -> bool) {
        for _ in 0..50 {
            if done(self) {
                return;
            }
            self.pass(smartport::Rx::Nothing, None, false);
        }
        panic!("{what}: stuck in {:?}", self.hs.phase);
    }

    /// The host's half of a whole transaction: a command (with a data packet if `data`), the
    /// device's reply after core 0 has taken `busy` passes. Checks each handshake step.
    fn transaction(&mut self, data: bool, busy: usize) {
        use smartport::{Phase, Rx};
        let (p0, s0) = (self.publishes, self.sends);
        assert!(!self.out.ack_low, "ACK up while idle");
        // REQ up: the command packet arrives over a few passes, then the device ACKs.
        self.req = true;
        self.pass(Rx::Nothing, None, false);
        assert!(self.out.open && self.hs.phase == Phase::RxCommand, "capture opened");
        for _ in 0..5 {
            self.pass(Rx::Nothing, None, false);
            assert!(!self.out.ack_low, "no ACK before the packet is in");
        }
        self.pass(Rx::Ours { data_follows: data }, None, false);
        assert!(self.out.ack_low && self.out.close);
        // The host sees ACK and puts REQ down.
        for _ in 0..3 {
            self.pass(Rx::Nothing, None, false);
            assert!(self.out.ack_low, "ACK held while REQ is still up");
        }
        self.req = false;
        if data {
            // ACK goes up: ready for the data packet. REQ up, the packet, ACK, REQ down.
            self.until("ready for data", |b| !b.out.ack_low);
            assert_eq!(self.hs.phase, Phase::WaitData);
            self.req = true;
            self.pass(Rx::Nothing, None, false);
            assert!(self.out.open && self.hs.phase == Phase::RxData);
            self.pass(Rx::Nothing, None, false);
            self.pass(Rx::Ours { data_follows: false }, None, false);
            assert!(self.out.ack_low);
            self.pass(Rx::Nothing, None, false);
            self.req = false;
        }
        assert_eq!(self.publishes, p0 + 1, "handed to core 0 once, whole");
        // Core 0 works; ACK stays low until it has answered.
        for _ in 0..busy {
            self.pass(Rx::Nothing, None, false);
            assert!(self.out.ack_low && self.hs.phase == Phase::Busy);
        }
        self.pass(Rx::Nothing, Some(true), false);
        assert!(!self.out.ack_low && self.hs.phase == Phase::Offered, "ACK up: ready to send");
        self.pass(Rx::Nothing, None, false);
        assert!(!self.out.driving, "RD quiet until REQ");
        // REQ up: the reply goes out on RD, then ACK, REQ down, ACK up.
        self.req = true;
        self.pass(Rx::Nothing, None, false);
        assert!(self.out.send && self.out.driving);
        for _ in 0..4 {
            self.pass(Rx::Nothing, None, false);
            assert!(self.out.driving && !self.out.ack_low);
        }
        self.pass(Rx::Nothing, None, true);
        assert!(!self.out.driving && self.out.ack_low, "sent: RD let go, ACK down");
        self.pass(Rx::Nothing, None, false);
        self.req = false;
        self.pass(Rx::Nothing, None, false);
        assert!(!self.out.ack_low && self.hs.phase == Phase::Idle);
        assert_eq!(self.sends, s0 + 1);
    }
}

fn check_handshake() {
    use smartport::{Phase, Rx};
    let mut b = Bus::new();
    b.pass(Rx::Nothing, None, false);
    b.transaction(false, 0); // status: answered at once
    b.transaction(false, 40); // a read block that takes a while
    b.transaction(true, 10); // a write block
    b.transaction(false, 3);

    // Someone else's packet: no ACK, quiet until REQ goes down, then the next is ours.
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::NotOurs, None, false);
    for _ in 0..5 {
        b.pass(Rx::Nothing, None, false);
        assert!(!b.out.ack_low && !b.out.driving);
    }
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    assert_eq!(b.hs.phase, Phase::Idle);
    b.transaction(false, 1);

    // The host gives up waiting for a slow reply and starts over: the device goes back to idle
    // and takes the new command.
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::Ours { data_follows: false }, None, false);
    b.req = false;
    b.until("busy", |b| b.hs.phase == Phase::Busy);
    b.pass(Rx::Nothing, None, false);
    let aborts = b.aborts;
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    assert!(b.aborts == aborts + 1 && !b.out.ack_low && b.hs.phase == Phase::Idle);
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    b.transaction(false, 2);

    // A packet cut short (REQ down before it's in), core 0 with nothing to say, and the bus let
    // go mid-reply.
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    assert!(b.out.close && b.hs.phase == Phase::Idle);
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::Ours { data_follows: false }, None, false);
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::Nothing, Some(false), false);
    assert!(!b.out.ack_low && b.hs.phase == Phase::Idle, "nothing to send: back to idle");
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::Ours { data_follows: false }, None, false);
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    b.pass(Rx::Nothing, Some(true), false);
    b.req = true;
    b.pass(Rx::Nothing, None, false);
    assert!(b.out.send);
    b.enabled = false;
    b.req = false;
    b.pass(Rx::Nothing, None, false);
    assert!(b.out.flush && b.out.abort && !b.out.driving && !b.out.ack_low && b.hs.phase == Phase::Idle);
    b.enabled = true;
    b.pass(Rx::Nothing, None, false);
    b.transaction(true, 0);
    println!("handshake: commands, a write's data packet, slow replies, someone else's packet, the host giving up, the bus let go ({} passes)", b.passes);
}

fn main() {
    check_codec();
    check_framing();
    check_commands();
    check_handshake();
    println!("sp_check: all passed");
}
