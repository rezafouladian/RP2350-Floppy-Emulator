//! SmartPort, the Apple II's bus for block devices: the IIc's, IIgs's and IIc+'s disk port, or a
//! IIe's with a SmartPort card (Liron, UniDisk 3.5 controller). The emulator is a few devices on it
//! ("units"): an image of 800K or less is a 3.5" drive (as a UniDisk 3.5), a bigger one a hard
//! disk. Hardware-free, so the host tests compile it directly; `drive::run_smartport` (core 1)
//! receives packets with [`Framer`] and `sp.rs` (core 0) answers them with [`command`].
//!
//! The bus.
//! - **Lines.** PH1 and PH3 both high enable the bus (a Disk II's stepper never has them both on);
//!   PH0 is REQ, from the host; WRPROT is ACK, from the device: driven low, or left
//!   high-impedance (the host's pull-up makes it high). PH0 and PH2 high with PH1 and PH3 low
//!   reset the bus, and every device forgets its ID.
//! - **Cells.** 4 µs. The host writes NRZI on WR (each transition a 1); the device sends pulses on
//!   RD (each falling edge a 1), and drives RD only while it does.
//! - **Handshake**, four phases each way. A command: REQ↑, the host sends the packet, the device
//!   that it's for drives ACK↓, REQ↓. Then for a command with data (write block, control): ACK↑
//!   (ready), REQ↑, the data packet, ACK↓, REQ↓. Then the reply: ACK↑ once it's ready, REQ↑, the
//!   device sends it, ACK↓, REQ↓, ACK↑ (idle). A device that isn't addressed stays quiet.
//! - **Packets.** Sync (the device's: [`SYNC`], four 10-bit `FF`s), then `C3`, DEST, SRC, TYPE,
//!   AUX, STAT, ODDCNT, GRP7CNT, the data, two checksum bytes and `C8`, every byte with its top bit
//!   set. The data is `ODDCNT` (= length mod 7) odd bytes after a byte of their top bits, then
//!   `GRP7CNT` groups of 7 as 8: a byte of their top bits (bit 6 the first's), then their low 7
//!   bits. The checksum is the XOR of the header bytes DEST to GRP7CNT (as sent) and of the data
//!   bytes, sent as `chk | AA` and `chk >> 1 | AA`.
//! - **IDs.** After a reset the host sends INIT to IDs `81`, `82`, … in turn. The first device
//!   without an ID takes it, and its reply's STAT is `FF` if it's the last one in the chain. The
//!   emulator is the end of its chain: its units take IDs in order, and the last says so.
//! - **Commands** (the first data byte; the rest are the call's parameter list after the unit
//!   number: a 2-byte buffer address, then a status/control code or a 3-byte block number):
//!   STATUS (code 0: device status, 4 bytes; 3: the device information block, 25 bytes),
//!   READBLOCK (a reply of 512 bytes), WRITEBLOCK (a data packet of 512, then a status reply),
//!   FORMAT, CONTROL (code 4 ejects), INIT; OPEN/CLOSE/READ/WRITE are for character devices.
//!   Extended commands (`$40`+, 4-byte parameters) only go to devices that claim them in their
//!   subtype; these units don't.

use core::sync::atomic::{AtomicU8, Ordering::Relaxed};

pub const BLOCK: usize = 512;
/// Units the emulator presents, whether they hold a disk or not: the host looks for devices
/// when it starts up.
pub const UNITS: usize = 4;
/// Most data bytes in a packet.
pub const MAX_DATA: usize = 767;
/// Most bytes of a packet from `C3` to `C8`: the header, 4 odd bytes and their top bits, 109
/// groups, the checksum and the end.
pub const MAX_PACKET: usize = 8 + 5 + (MAX_DATA / 7) * 8 + 3;
/// Bytes of a reply on the wire: sync and packet.
pub const MAX_REPLY: usize = SYNC.len() + MAX_PACKET;
/// What the device sends before a packet: `FF`, then four 10-bit syncs (`1111111100`).
pub const SYNC: [u8; 6] = [0xFF, 0x3F, 0xCF, 0xF3, 0xFC, 0xFF];
pub const BEGIN: u8 = 0xC3;
pub const END: u8 = 0xC8;
/// The host's ID, as a reply's DEST.
const HOST: u8 = 0x80;

/// Packet type (TYPE) of a reply: a status packet, with data or not.
/// (Commands are `80`, data packets `82`.)
pub const TYPE_STATUS: u8 = 0x81;

// Commands.
pub const STATUS: u8 = 0x00;
pub const READ_BLOCK: u8 = 0x01;
pub const WRITE_BLOCK: u8 = 0x02;
pub const FORMAT: u8 = 0x03;
pub const CONTROL: u8 = 0x04;
pub const INIT: u8 = 0x05;
pub const WRITE: u8 = 0x09;
/// Extended commands (`$40` + the command), with 4-byte addresses and block numbers.
pub const EXTENDED: u8 = 0x40;

// Errors (a reply's STAT, without its top bit).
pub const BAD_COMMAND: u8 = 0x01;
pub const BAD_CODE: u8 = 0x21;
pub const IO_ERROR: u8 = 0x27;
pub const WRITE_PROTECTED: u8 = 0x2B;
pub const BAD_BLOCK: u8 = 0x2D;
pub const OFFLINE: u8 = 0x2F;
/// INIT's reply from the last device in the chain.
pub const END_OF_CHAIN: u8 = 0x7F;

/// Status and control codes.
const STATUS_DEVICE: u8 = 0x00;
const STATUS_DIB: u8 = 0x03;
const CONTROL_EJECT: u8 = 0x04;

/// Device types for the DIB: a 3.5" disk (as a UniDisk 3.5), or a hard disk. Subtype `$40`:
/// removable media that reports disk switches, no extended commands.
const TYPE_35: u8 = 0x01;
const TYPE_HARD_DISK: u8 = 0x02;
const SUBTYPE: u8 = 0x40;
/// Images of up to this many blocks are 3.5" disks (800K).
pub const FLOPPY_BLOCKS: u32 = 1_600;

/// A packet's commands for a device that has a data packet after them: write block, control,
/// write (and their extended forms).
#[inline(always)]
pub fn has_data(command: u8) -> bool {
    matches!(command & !EXTENDED, WRITE_BLOCK | CONTROL | WRITE)
}

/// Length from `C3` to `C8` of a packet with this ODDCNT and GRP7CNT.
#[inline(always)]
pub fn packet_len(oddcnt: u8, grp7cnt: u8) -> usize {
    let odd = (oddcnt & 0x7F) as usize;
    let groups = (grp7cnt & 0x7F) as usize;
    8 + if odd > 0 { 1 + odd } else { 0 } + groups * 8 + 3
}

/// What [`Framer::cell`] made of a cell.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Framed {
    More,
    /// The packet is in, `C3` to `C8`, this many bytes (its checksum not yet checked).
    Packet(usize),
    /// Its length doesn't fit the buffer, or it doesn't end in `C8`.
    Bad,
}

/// Frames the host's cells into packet bytes as an IWM does (a byte is complete when its top bit
/// is set), skipping sync until `C3`. For core 1: no panics, no calls out of line.
pub struct Framer {
    shifter: u8,
    len: usize,
    want: usize,
}

impl Framer {
    pub const fn new() -> Self {
        Self { shifter: 0, len: 0, want: 0 }
    }

    /// Take one cell (1 = a transition), storing bytes from `C3` on in `buf`.
    #[inline(always)]
    pub fn cell(&mut self, one: bool, buf: &mut [u8]) -> Framed {
        self.shifter = self.shifter << 1 | one as u8;
        if self.shifter & 0x80 == 0 {
            return Framed::More;
        }
        let byte = self.shifter;
        self.shifter = 0;
        if self.len == 0 && byte != BEGIN {
            return Framed::More; // sync
        }
        match buf.get_mut(self.len) {
            Some(slot) => *slot = byte,
            None => return Framed::Bad,
        }
        self.len += 1;
        if self.len == 8 {
            let (odd, groups) = match (buf.get(6), buf.get(7)) {
                (Some(&o), Some(&g)) => (o, g),
                _ => return Framed::Bad,
            };
            self.want = packet_len(odd, groups);
            if self.want > buf.len() {
                return Framed::Bad;
            }
        }
        if self.len >= 8 && self.len == self.want {
            return if byte == END { Framed::Packet(self.len) } else { Framed::Bad };
        }
        Framed::More
    }
}

/// The data bytes of packet `p` (`C3` to `C8`), each passed to `f` in order. False if `p` is
/// shorter than its header says.
#[inline(always)]
pub fn data_bytes(p: &[u8], mut f: impl FnMut(u8)) -> bool {
    let (Some(&odd), Some(&groups)) = (p.get(6), p.get(7)) else { return false };
    if p.len() < packet_len(odd, groups) {
        return false;
    }
    // Each chunk: a byte of top bits (bit 6 the first byte's), then the bytes. `get` throughout:
    // core 1 runs this, and mustn't have a panic path.
    let mut chunk = |at: usize, n: usize| -> bool {
        let Some(&msb) = p.get(at) else { return false };
        for i in 0..n {
            let Some(&b) = p.get(at + 1 + i) else { return false };
            f(msb.wrapping_shl(i as u32 + 1) & 0x80 | b & 0x7F);
        }
        true
    };
    let odd = (odd & 0x7F) as usize;
    let mut at = 8;
    if odd > 0 {
        if !chunk(at, odd) {
            return false;
        }
        at += 1 + odd;
    }
    for _ in 0..(groups & 0x7F) {
        if !chunk(at, 7) {
            return false;
        }
        at += 8;
    }
    true
}

/// The checksum of packet `p` (`C3` to `C8`) is right. `lenient`: also take a data packet of
/// 256–511 bytes whose checksum is wrong, as for the IIc ROM 0 and the Liron card,
/// which get it wrong.
#[inline(always)]
pub fn checksum_ok(p: &[u8], lenient: bool) -> bool {
    let mut sum = 0u8;
    let mut count = 0usize;
    if !data_bytes(p, |b| {
        sum ^= b;
        count += 1;
    }) {
        return false;
    }
    for i in 1..8 {
        sum ^= p.get(i).copied().unwrap_or(0);
    }
    let at = p.len().saturating_sub(3);
    let (Some(&lo), Some(&hi)) = (p.get(at), p.get(at + 1)) else { return false };
    let sent = lo & 0x55 | (hi & 0x55) << 1;
    sent == sum || (lenient && (0x100..0x200).contains(&count))
}

/// Decode the data of a packet whose checksum has been checked into `out`; returns how many
/// bytes (at most `out.len()`).
pub fn decode(p: &[u8], out: &mut [u8]) -> usize {
    let mut n = 0;
    data_bytes(p, |b| {
        if let Some(slot) = out.get_mut(n) {
            *slot = b;
        }
        n += 1;
    });
    n.min(out.len())
}

/// Encode a reply from `src` with STAT `stat` and `data` (at most [`MAX_DATA`] bytes) into `out`,
/// sync first, as it goes on the wire. Returns its length.
pub fn encode(out: &mut [u8; MAX_REPLY], src: u8, packet_type: u8, stat: u8, data: &[u8]) -> usize {
    let data = &data[..data.len().min(MAX_DATA)];
    let (odd, groups) = (data.len() % 7, data.len() / 7);
    out[..SYNC.len()].copy_from_slice(&SYNC);
    let p = &mut out[SYNC.len()..];
    let header = [BEGIN, HOST, src | 0x80, packet_type, 0x80, stat | 0x80, odd as u8 | 0x80, groups as u8 | 0x80];
    p[..8].copy_from_slice(&header);
    let mut sum = header[1..].iter().fold(0, |s, b| s ^ b);
    let mut at = 8;
    let mut put = |chunk: &[u8], at: &mut usize| {
        let mut msb = 0x80;
        for (i, &b) in chunk.iter().enumerate() {
            msb |= (b & 0x80) >> (i + 1);
            p[*at + 1 + i] = b | 0x80;
            sum ^= b;
        }
        p[*at] = msb;
        *at += 1 + chunk.len();
    };
    if odd > 0 {
        put(&data[..odd], &mut at);
    }
    for g in data[odd..].chunks(7) {
        put(g, &mut at);
    }
    p[at] = sum | 0xAA;
    p[at + 1] = sum >> 1 | 0xAA;
    p[at + 2] = END;
    SYNC.len() + at + 3
}

/// The units' disks, as [`command`] sees them.
pub trait Units {
    /// Blocks of unit `u`'s disk, or `None` with no disk in.
    fn blocks(&self, u: usize) -> Option<u32>;
    fn writable(&self, u: usize) -> bool;
    fn read(&mut self, u: usize, block: u32, out: &mut [u8; BLOCK]) -> Result<(), ()>;
    fn write(&mut self, u: usize, block: u32, data: &[u8; BLOCK]) -> Result<(), ()>;
    /// The host ejected unit `u`'s disk.
    fn eject(&mut self, u: usize);
    /// Unit `u`'s disk changed since this was last asked (and isn't any more).
    fn take_switched(&mut self, u: usize) -> bool;
    /// The DIB's name for unit `u` (at most 16 bytes, upper case).
    fn name(&self, u: usize) -> &[u8];
}

/// What a command was, for the display.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Last {
    None,
    Init,
    Status,
    Read,
    Write,
    Control,
    Other(u8),
}

/// What [`command`] did.
pub struct Answer {
    /// The reply's bytes in `reply`, from sync; 0 for none (not one of the units').
    pub len: usize,
    pub last: Last,
    /// The unit it was for.
    pub unit: Option<usize>,
    /// The reply's error, 0 for none.
    pub error: u8,
}

/// The unit whose ID is `id`, if any. `ids[u]` is unit `u`'s ID (`81`…), or 0 for none yet.
pub fn unit_of(ids: &[AtomicU8; UNITS], id: u8) -> Option<usize> {
    (id != 0).then(|| ids.iter().position(|i| i.load(Relaxed) == id)).flatten()
}

/// Answer command packet `cmd` (`C3` to `C8`, checksum checked) for the units whose IDs are in
/// `ids`, with `data` the decoded data packet that came after it, if any. INIT assigns the next
/// unit its ID. The reply goes in `reply`.
pub fn command(ids: &[AtomicU8; UNITS], cmd: &[u8], data: &[u8], units: &mut impl Units, reply: &mut [u8; MAX_REPLY]) -> Answer {
    let mut frame = [0u8; 9];
    decode(cmd, &mut frame);
    let dest = cmd.get(1).copied().unwrap_or(0);
    let code = frame[0];
    let mut answer = Answer { len: 0, last: Last::Other(code), unit: None, error: 0 };
    if code == INIT {
        // The next unit without an ID takes this one; the last says so.
        let Some(u) = ids.iter().position(|i| i.load(Relaxed) == 0) else { return answer };
        ids[u].store(dest, Relaxed);
        let last = ids.iter().all(|i| i.load(Relaxed) != 0);
        let stat = if last { END_OF_CHAIN } else { 0 };
        answer.len = encode(reply, dest, TYPE_STATUS, stat, &[]);
        answer.last = Last::Init;
        answer.unit = Some(u);
        return answer;
    }
    let Some(u) = unit_of(ids, dest) else { return answer };
    answer.unit = Some(u);
    let blocks = units.blocks(u);
    let block = u32::from_le_bytes([frame[4], frame[5], frame[6], 0]);
    let mut buf = [0u8; BLOCK];
    let (stat, payload): (u8, &[u8]) = match code {
        STATUS => {
            answer.last = Last::Status;
            match frame[4] {
                STATUS_DEVICE => {
                    let s = status_byte(units, u);
                    let n = blocks.unwrap_or(0).to_le_bytes();
                    buf[..4].copy_from_slice(&[s, n[0], n[1], n[2]]);
                    (0, &buf[..4])
                }
                STATUS_DIB => {
                    let s = status_byte(units, u);
                    let n = blocks.unwrap_or(0).to_le_bytes();
                    buf[..5].copy_from_slice(&[s, n[0], n[1], n[2], 16]);
                    let name = units.name(u);
                    buf[5..21].fill(b' ');
                    let k = name.len().min(16);
                    buf[5..5 + k].copy_from_slice(&name[..k]);
                    let kind = if blocks.is_some_and(|b| b <= FLOPPY_BLOCKS) { TYPE_35 } else { TYPE_HARD_DISK };
                    buf[21..25].copy_from_slice(&[kind, SUBTYPE, 0x00, 0x01]);
                    (0, &buf[..25])
                }
                _ => (BAD_CODE, &[]),
            }
        }
        READ_BLOCK => {
            answer.last = Last::Read;
            match blocks {
                None => (OFFLINE, &[]),
                Some(n) if block >= n => (BAD_BLOCK, &[]),
                Some(_) => match units.read(u, block, &mut buf) {
                    Ok(()) => (0, &buf[..]),
                    Err(()) => (IO_ERROR, &[]),
                },
            }
        }
        WRITE_BLOCK => {
            answer.last = Last::Write;
            match blocks {
                None => (OFFLINE, &[]),
                Some(_) if !units.writable(u) => (WRITE_PROTECTED, &[]),
                Some(n) if block >= n => (BAD_BLOCK, &[]),
                Some(_) if data.len() < BLOCK => (IO_ERROR, &[]),
                Some(_) => {
                    buf.copy_from_slice(&data[..BLOCK]);
                    match units.write(u, block, &buf) {
                        Ok(()) => (0, &[]),
                        Err(()) => (IO_ERROR, &[]),
                    }
                }
            }
        }
        FORMAT => match blocks {
            None => (OFFLINE, &[]),
            Some(_) => (0, &[]),
        },
        CONTROL => {
            answer.last = Last::Control;
            if frame[4] == CONTROL_EJECT {
                units.eject(u);
            }
            // Other codes (reset, set DCB, …) need nothing here; GS/OS hangs at shutdown on
            // devices that refuse them.
            (0, &[])
        }
        _ => (BAD_COMMAND, &[]),
    };
    answer.error = stat;
    answer.len = encode(reply, dest, TYPE_STATUS, stat, payload);
    answer
}

/// A unit's status byte: block device, read, write and format allowed; online with a disk in;
/// write-protected; the disk switched since the last time (which this clears).
fn status_byte(units: &mut impl Units, u: usize) -> u8 {
    let mut s = 0x80 | 0x40 | 0x20 | 0x08;
    if units.blocks(u).is_some() {
        s |= 0x10;
        if !units.writable(u) {
            s |= 0x04;
        }
    }
    if units.take_switched(u) {
        s |= 0x01;
    }
    s
}

/// Where [`Handshake`] is in a transaction.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Phase {
    /// Waiting for REQ with the bus enabled.
    Idle,
    /// Receiving the command packet, or its data packet.
    RxCommand,
    RxData,
    /// Command ACKed with a data packet to come: waiting for REQ↓, then REQ↑ to receive it.
    WaitDataDown,
    WaitData,
    /// ACK low until REQ is down and core 0 has answered.
    Busy,
    /// ACK up: waiting for REQ↑ to send the reply.
    Offered,
    Sending,
    /// Sent, ACK low: waiting for REQ↓.
    Sent,
    /// Someone else's packet, or a bad one: quiet until REQ↓.
    Ignore,
}

/// What the receive side made of this pass's cells, in [`Phase::RxCommand`]/[`Phase::RxData`].
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Rx {
    Nothing,
    /// A whole packet for one of the units, its checksum right; for a command, whether a data
    /// packet follows.
    Ours { data_follows: bool },
    /// A whole packet for someone else, or a bad one.
    NotOurs,
}

/// What core 1 is to do after a [`Handshake::step`].
#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct Step {
    /// ACK low (else high-impedance).
    pub ack_low: bool,
    /// RD driven with the reply (else high-impedance).
    pub driving: bool,
    /// Start capturing a packet / stop.
    pub open: bool,
    pub close: bool,
    /// Hand the transaction to core 0.
    pub publish: bool,
    /// Start sending the reply / drop what's queued of it.
    pub send: bool,
    pub flush: bool,
    /// The host gave up on a transaction under way.
    pub abort: bool,
}

/// Core 1's side of the four-phase handshake, stepped once per pass with what it sees.
pub struct Handshake {
    pub phase: Phase,
    /// In [`Phase::Busy`]: REQ has gone down since the ACK.
    req_down: bool,
}

impl Handshake {
    pub const fn new() -> Self {
        Self { phase: Phase::Idle, req_down: false }
    }

    /// One pass: the bus is `enabled` (PH1 and PH3 high, and not the IIgs's 3.5" drives), REQ is
    /// `req`, the receive side says `rx`, core 0 has answered the transaction with `replied`
    /// (`Some(false)`: with nothing to send), and in [`Phase::Sending`] the whole reply is out
    /// (`sent`).
    #[inline(always)]
    pub fn step(&mut self, enabled: bool, req: bool, rx: Rx, replied: Option<bool>, sent: bool) -> Step {
        let mut out = Step::default();
        if !enabled {
            // The host let go of the bus: whatever was under way is over.
            match self.phase {
                Phase::Idle => {}
                Phase::RxCommand | Phase::RxData => out.close = true,
                Phase::Sending => out.flush = true,
                _ => {}
            }
            out.abort = !matches!(self.phase, Phase::Idle | Phase::Ignore | Phase::Sent);
            self.phase = Phase::Idle;
            return out;
        }
        match self.phase {
            Phase::Idle => {
                if req {
                    out.open = true;
                    self.phase = Phase::RxCommand;
                }
            }
            Phase::RxCommand | Phase::RxData => match rx {
                Rx::Ours { data_follows } => {
                    out.close = true;
                    out.ack_low = true;
                    if self.phase == Phase::RxCommand && data_follows {
                        self.phase = Phase::WaitDataDown;
                    } else {
                        out.publish = true;
                        self.req_down = false;
                        self.phase = Phase::Busy;
                    }
                }
                Rx::NotOurs => {
                    out.close = true;
                    self.phase = Phase::Ignore;
                }
                Rx::Nothing if !req => {
                    // The host stopped before the packet was in.
                    out.close = true;
                    self.phase = Phase::Idle;
                }
                Rx::Nothing => {}
            },
            Phase::WaitDataDown => {
                out.ack_low = req;
                if !req {
                    self.phase = Phase::WaitData;
                }
            }
            Phase::WaitData => {
                if req {
                    out.open = true;
                    self.phase = Phase::RxData;
                }
            }
            Phase::Busy => {
                out.ack_low = true;
                self.req_down |= !req;
                if req && self.req_down {
                    // Up again with ACK still low: the host gave up waiting and is starting over.
                    out.ack_low = false;
                    out.abort = true;
                    self.phase = Phase::Idle;
                } else if self.req_down && let Some(reply) = replied {
                    // Ready to send; with nothing to send, let the host time out and try again.
                    out.ack_low = false;
                    self.phase = if reply { Phase::Offered } else { Phase::Idle };
                }
            }
            Phase::Offered => {
                if req {
                    out.send = true;
                    out.driving = true;
                    self.phase = Phase::Sending;
                }
            }
            Phase::Sending => {
                out.driving = true;
                if sent {
                    out.driving = false;
                    out.ack_low = true;
                    self.phase = Phase::Sent;
                }
            }
            Phase::Sent => {
                out.ack_low = req;
                if !req {
                    self.phase = Phase::Idle;
                }
            }
            Phase::Ignore => {
                if !req {
                    self.phase = Phase::Idle;
                }
            }
        }
        out
    }
}
