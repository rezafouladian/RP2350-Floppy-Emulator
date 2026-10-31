//! The HD20's protocol: DCD ("Directly Connected Disk"), Apple's hard disk interface on the floppy
//! port.
//!
//! On the wire (core 1, `drive::run_dcd`):
//! - CA2–CA0 are a state, not a register address. 0 holdoff, 1 data, 2 idle, 3 host: RD carries
//!   data in 0 and 1 and /HSHK (low = ready) in 2 and 3. RD reads low in 4 (reset) and 5, high in
//!   6 and 7: the Mac's probe (R10 = 0, R12 = R14 = 1).
//! - A pulse on LSTRB (CA3) passes the Mac on to the next device in the chain. There is none, so
//!   RD goes high-impedance until /ENBL goes high; otherwise the Mac sees the same disk four times.
//! - The Mac raises HOST (2 → 3), waits for /HSHK, goes to state 1 and writes: `AA`, 0x80 + the
//!   groups it sends, 0x80 + the groups it wants back, then the groups. It goes back to 3 when
//!   done, waits for /HSHK to go high, then to 2. When the reply is ready the drive asserts
//!   /HSHK; the Mac goes 3 → 1 and reads `AA` and exactly the groups it asked for, then 3 → 2.
//! - Holdoff: the Mac goes from 1 to 0 to serve an interrupt. The group under way is finished;
//!   on 0 → 1 the transfer resumes with `AA` and the next group. A Mac that is writing first
//!   brings WR low if the last bit left it high: that edge is not data.
//!
//! Each payload byte travels in a 7-to-8 group: seven bytes shifted right with the top bit set,
//! plus a byte of their low bits (bit `i` for byte `i`), also with the top bit set. The Mac sends
//! that byte first, the drive last. Every payload ends with a checksum byte that makes the whole
//! payload sum to 0.
//!
//! Hardware-free, so the host tests compile it directly.

/// The sync byte that starts every transfer, and every resumption after a holdoff.
pub const SYNC: u8 = 0xAA;
/// Groups in one transfer: the Mac's group counts are 7 bits.
pub const MAX_GROUPS: usize = 0x7F;
pub const MAX_PAYLOAD: usize = 7 * MAX_GROUPS;
pub const MAX_ENCODED: usize = 8 * MAX_GROUPS;
pub const BLOCK: usize = 512;
/// Tag bytes per block (the Lisa's; the Mac ignores them).
const TAGS: usize = 20;
/// Payload offset of a block's data in a read reply or a write command: identifier, count,
/// address or status, the tags.
const DATA_AT: usize = 6 + TAGS;
/// Blocks a DCD can have: the address and size fields are 3 bytes.
pub const MAX_BLOCKS: u32 = 0xFF_FFFF;
/// Status for a failed command. Anything but 0 is an error to the Mac.
const ERROR_STATUS: u8 = 0x80;
/// Identifier of a reply asking the Mac to send its command again (bad checksum).
const NAK: u8 = 0x7F;

/// Zero cells in a row that only a pause makes: within a transfer the Mac's bytes follow each
/// other with at most 7 (a byte is at most `1000_0000`, and the next starts with a 1).
const PAUSE_CELLS: u32 = 9;

/// The seven payload bytes of a group as the Mac sends it (low bits first).
pub fn decode_group(raw: &[u8]) -> [u8; 7] {
    let lsbs = raw[0];
    core::array::from_fn(|i| raw[1 + i] << 1 | (lsbs >> i & 1))
}

/// A group as the drive sends it (low bits last).
pub fn encode_group(data: &[u8]) -> [u8; 8] {
    let mut raw = [0u8; 8];
    for i in 0..7 {
        raw[i] = 0x80 | data[i] >> 1;
        raw[7] |= (data[i] & 1) << i;
    }
    raw[7] |= 0x80;
    raw
}

/// The byte that makes `bytes` and itself sum to 0.
pub fn checksum(bytes: &[u8]) -> u8 {
    0u8.wrapping_sub(bytes.iter().fold(0u8, |s, &b| s.wrapping_add(b)))
}

fn be24(b: &[u8]) -> u32 {
    u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2])
}

/// A transfer from the Mac, decoded into a payload buffer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Received {
    /// Payload bytes decoded (7 per group).
    pub len: usize,
    /// Groups the Mac wants back.
    pub reply_groups: usize,
    /// Every group the Mac said it would send arrived, and the checksum holds.
    pub ok: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum Stage {
    Sync,
    Sends,
    Replies,
    Group,
    /// After a holdoff: waiting for the `AA` that resumes the transfer.
    Resync,
}

/// Decode a transfer from the cells `capture.pio`'s GCR separator made of WR (1 = a transition).
///
/// The capture starts with a 1 that isn't data: the separator starts out taking WR as high, so a
/// low WR shows as a transition at once, and a high one the Mac brings low before it starts. That
/// 1 is skipped; if that doesn't give a good transfer, the cells are read again without skipping.
/// Holdoffs are found by their pause, and the edge the Mac may make before resuming by following
/// WR's level through the transfer.
///
/// Fails only if not even the length bytes arrived; otherwise `Received::ok` says whether the
/// payload can be trusted.
pub fn receive(cells: &impl Fn(usize) -> bool, len: usize, out: &mut [u8; MAX_PAYLOAD]) -> Result<Received, &'static str> {
    let first = receive_from(cells, len, true, out);
    if matches!(first, Ok(r) if r.ok) {
        return first;
    }
    let mut again = [0u8; MAX_PAYLOAD];
    match receive_from(cells, len, false, &mut again) {
        Ok(r) if r.ok || first.is_err() => {
            out.copy_from_slice(&again);
            Ok(r)
        }
        _ => first,
    }
}

fn receive_from(cells: &impl Fn(usize) -> bool, len: usize, skip_first: bool, out: &mut [u8; MAX_PAYLOAD]) -> Result<Received, &'static str> {
    // WR's level, from the separator's point of view (it starts out assuming high).
    let mut level = true;
    let mut skip = skip_first;
    let mut zeros = 0u32;
    let mut shifter = 0u8;
    let mut stage = Stage::Sync;
    let (mut sends, mut replies, mut groups) = (0usize, 0usize, 0usize);
    let mut raw = [0u8; 8];
    let mut k = 0;
    let result = |groups: usize, sends: usize, replies: usize, out: &[u8; MAX_PAYLOAD]| {
        let len = 7 * groups;
        let ok = groups == sends && out[..len].iter().fold(0u8, |s, &b| s.wrapping_add(b)) == 0;
        Received { len, reply_groups: replies, ok }
    };
    for i in 0..len {
        let b = cells(i);
        if b {
            level = !level;
            zeros = 0;
        } else {
            zeros += 1;
        }
        if shifter == 0 {
            if !b {
                // Between bytes. A pause after a group (or the length bytes) is a holdoff: the
                // Mac resumes with WR low, so if it is high now, the next 1 is that edge.
                if zeros == PAUSE_CELLS && (stage == Stage::Group && k == 0 || stage == Stage::Resync) {
                    stage = Stage::Resync;
                    skip = level;
                }
                continue;
            }
            if skip {
                skip = false;
                continue;
            }
        }
        shifter = shifter << 1 | b as u8;
        if shifter & 0x80 == 0 {
            continue;
        }
        let byte = shifter;
        shifter = 0;
        match stage {
            Stage::Sync => {
                if byte == SYNC {
                    stage = Stage::Sends;
                }
            }
            Stage::Sends => {
                sends = usize::from(byte & 0x7F);
                if sends == 0 {
                    return Err("DCD: empty command");
                }
                stage = Stage::Replies;
            }
            Stage::Replies => {
                replies = usize::from(byte & 0x7F);
                stage = Stage::Group;
            }
            Stage::Group => {
                raw[k] = byte;
                k += 1;
                if k == 8 {
                    k = 0;
                    out[7 * groups..7 * groups + 7].copy_from_slice(&decode_group(&raw));
                    groups += 1;
                    if groups == sends {
                        return Ok(result(groups, sends, replies, out));
                    }
                }
            }
            Stage::Resync => {
                if byte == SYNC {
                    stage = Stage::Group;
                }
            }
        }
    }
    match stage {
        Stage::Sync | Stage::Sends | Stage::Replies => Err("DCD: no command"),
        _ => Ok(result(groups, sends, replies, out)),
    }
}

/// Encode a reply of `groups` groups (1 to [`MAX_GROUPS`]) into `out`; `fill` writes the payload
/// before the checksum (it's given exactly that much room). Returns the bytes written.
pub fn reply(groups: usize, out: &mut [u8], fill: impl FnOnce(&mut [u8])) -> usize {
    let groups = groups.clamp(1, MAX_GROUPS);
    let n = 7 * groups;
    let mut p = [0u8; MAX_PAYLOAD];
    fill(&mut p[..n - 1]);
    p[n - 1] = checksum(&p[..n - 1]);
    for g in 0..groups {
        out[8 * g..8 * g + 8].copy_from_slice(&encode_group(&p[7 * g..7 * g + 7]));
    }
    8 * groups
}

/// Copy `bytes` to `p[at..]`, as far as it fits.
fn put(p: &mut [u8], at: usize, bytes: &[u8]) {
    if at < p.len() {
        let n = bytes.len().min(p.len() - at);
        p[at..at + n].copy_from_slice(&bytes[..n]);
    }
}

/// The reply header: identifier, count, status (4 bytes; 0 = no error).
fn header(p: &mut [u8], id: u8, count: u8, ok: bool) {
    put(p, 0, &[id, count, if ok { 0 } else { ERROR_STATUS }, 0, 0, 0]);
}

/// Where the Get Info window says the disk is (a Pascal string of up to 15 characters).
const LOCATION: &[u8] = b"rp2350-floppy";

/// The Finder's icon for the disk, and its mask: the body's outline, filled.
const ICON_ROWS: [&[u8; 32]; 32] = [
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"..############################..",
    b".#............................#.",
    b".#..######################....#.",
    b".#..#....................#....#.",
    b".#..#..################..#....#.",
    b".#..#....................#....#.",
    b".#..######################....#.",
    b".#............................#.",
    b".#............................#.",
    b".#............................#.",
    b".#..#.#.#.#.#.#.#.#.....####..#.",
    b".#..#.#.#.#.#.#.#.#.....####..#.",
    b".#............................#.",
    b".#............................#.",
    b"..############################..",
    b"...#........................#...",
    b"...##########################...",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
    b"................................",
];

/// [`ICON_ROWS`] as an `ICN#`: 128 bytes of icon (MSB = leftmost pixel), then 128 of mask.
const ICON: [u8; 256] = {
    let mut out = [0u8; 256];
    let mut y = 0;
    while y < 32 {
        let row = ICON_ROWS[y];
        let (mut first, mut last) = (32, 0);
        let mut x = 0;
        while x < 32 {
            if row[x] == b'#' {
                out[4 * y + x / 8] |= 0x80 >> (x % 8);
                if first == 32 {
                    first = x;
                }
                last = x;
            }
            x += 1;
        }
        let mut x = first;
        while x <= last && first < 32 {
            out[128 + 4 * y + x / 8] |= 0x80 >> (x % 8);
            x += 1;
        }
        y += 1;
    }
    out
};

/// Device characteristics in the status reply.
const MOUNTABLE: u8 = 0x80;
const READABLE: u8 = 0x40;
const WRITABLE: u8 = 0x20;
const WRITE_PROTECTED: u8 = 0x08;
const HAS_ICON: u8 = 0x04;
const DISK_IN_PLACE: u8 = 0x02;

/// Reply to Controller Status (`03`): 343 payload bytes, 49 groups. The Mac learns the disk's size
/// and icon from it.
pub fn status_reply(blocks: u32, writable: bool, groups: usize, out: &mut [u8]) -> usize {
    reply(groups, out, |p| {
        header(p, 0x83, 0, true);
        put(p, 6, &[0, 1, 0, 1]); // device type, manufacturer: as the HD20
        let access = if writable { WRITABLE } else { WRITE_PROTECTED };
        let b = blocks.min(MAX_BLOCKS);
        put(p, 10, &[MOUNTABLE | READABLE | access | HAS_ICON | DISK_IN_PLACE, (b >> 16) as u8, (b >> 8) as u8, b as u8]);
        // 14–17: spare and bad blocks, 18–69 reserved: all 0.
        put(p, 70, &ICON);
        put(p, 326, &[LOCATION.len() as u8]);
        put(p, 327, LOCATION);
    })
}

/// One block of a read (`80`): `remaining` counts this one.
pub fn read_reply(remaining: u8, data: &[u8; BLOCK], groups: usize, out: &mut [u8]) -> usize {
    reply(groups, out, |p| {
        header(p, 0x80, remaining, true);
        put(p, DATA_AT, data);
    })
}

/// A reply with only the header: to a write (`81`/`82`), to a command this doesn't know, or to one
/// that failed.
pub fn plain_reply(id: u8, count: u8, ok: bool, groups: usize, out: &mut [u8]) -> usize {
    reply(groups, out, |p| header(p, id, count, ok))
}

/// Ask the Mac to send its command again.
pub fn nak_reply(groups: usize, out: &mut [u8]) -> usize {
    reply(groups, out, |p| put(p, 0, &[NAK]))
}

/// The disk behind the protocol: blocks of 512 bytes from 0.
pub trait Storage {
    fn blocks(&self) -> u32;
    fn writable(&self) -> bool;
    fn read(&mut self, block: u32, out: &mut [u8; BLOCK]) -> Result<(), ()>;
    fn write(&mut self, block: u32, data: &[u8; BLOCK]) -> Result<(), ()>;
}

/// What the last command was, for the display.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Last {
    None,
    Read,
    Write,
    Status,
    /// A command this doesn't know (format, verify, ID…), answered with an empty success.
    Other(u8),
    /// Bad checksum or a transfer cut short: answered with a NAK.
    Garbled,
}

/// The drive's side of the protocol: answers commands and keeps a multi-block read going.
pub struct Session {
    /// Blocks still to be sent for a read: next block, how many are left, the groups per reply.
    read: Option<(u32, u8, usize)>,
    /// Where a write continuation (`41`/`42`) goes: the block after the last one written.
    next_write: Option<u32>,
    pub last: Last,
    pub blocks_read: u32,
    pub blocks_written: u32,
    /// Commands that failed (out of range, card errors, writes to a read-only disk) and NAKs.
    pub errors: u32,
}

impl Session {
    pub const fn new() -> Self {
        Self { read: None, next_write: None, last: Last::None, blocks_read: 0, blocks_written: 0, errors: 0 }
    }

    /// Replies of a read are still owed: call [`next_reply`](Self::next_reply).
    pub fn reading(&self) -> bool {
        self.read.is_some()
    }

    /// The Mac has moved on (a new command, or it gave up): forget the read.
    pub fn abandon(&mut self) {
        self.read = None;
    }

    /// Handle a command the Mac sent. Returns the reply's length in `out`, or 0 if replies come
    /// from [`next_reply`](Self::next_reply) (a read).
    pub fn command<S: Storage>(&mut self, rx: &Received, payload: &[u8], disk: &mut S, out: &mut [u8]) -> usize {
        self.read = None;
        // A continuation only follows the write before it.
        let next_write = self.next_write.take();
        let groups = rx.reply_groups;
        if !rx.ok || rx.len < 7 {
            self.last = Last::Garbled;
            self.errors += 1;
            return nak_reply(groups, out);
        }
        let p = payload;
        let (id, count) = (p[0], p[1]);
        match id {
            0x00 => {
                self.last = Last::Read;
                let first = be24(&p[2..5]);
                let end = u64::from(first) + u64::from(count);
                if count == 0 || end > u64::from(disk.blocks()) {
                    self.errors += 1;
                    return plain_reply(0x80, count, false, groups, out);
                }
                self.read = Some((first, count, groups));
                0
            }
            0x01 | 0x02 | 0x41 | 0x42 => {
                self.last = Last::Write;
                let block = if id & 0x40 == 0 { Some(be24(&p[2..5])) } else { next_write };
                let mut ok = false;
                if let Some(block) = block
                    && rx.len > DATA_AT + BLOCK
                    && disk.writable()
                    && block < disk.blocks()
                {
                    let mut data = [0u8; BLOCK];
                    data.copy_from_slice(&p[DATA_AT..DATA_AT + BLOCK]);
                    ok = disk.write(block, &data).is_ok();
                    if ok {
                        self.blocks_written += 1;
                        self.next_write = Some(block + 1);
                    }
                }
                if !ok {
                    self.errors += 1;
                }
                plain_reply(0x80 | id & 0x3F, count, ok, groups, out)
            }
            0x03 => {
                self.last = Last::Status;
                status_reply(disk.blocks(), disk.writable(), groups, out)
            }
            _ => {
                // Format (19), verify (1A), read ID (04)…: nothing to do on an image.
                self.last = Last::Other(id);
                plain_reply(0x80 | id & 0x3F, 0, true, groups, out)
            }
        }
    }

    /// The next reply of a read, or `None` once they've all been made.
    pub fn next_reply<S: Storage>(&mut self, disk: &mut S, out: &mut [u8]) -> Option<usize> {
        let (block, left, groups) = self.read?;
        let mut data = [0u8; BLOCK];
        if disk.read(block, &mut data).is_err() {
            self.read = None;
            self.errors += 1;
            return Some(plain_reply(0x80, left, false, groups, out));
        }
        self.blocks_read += 1;
        self.read = if left > 1 { Some((block + 1, left - 1, groups)) } else { None };
        Some(read_reply(left, &data, groups, out))
    }
}

/// Where the volume is in an image file.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Volume {
    /// First block of the volume in the file.
    pub first: u32,
    pub blocks: u32,
    pub kind: &'static str,
}

/// Find the volume in an image of `file_blocks` blocks: the whole file if it is an HFS or MFS
/// volume, or the first `Apple_HFS` partition of an Apple partition map (as in a SCSI disk
/// image). The DCD has no partition map of its own: the Mac reads the volume from block 0.
pub fn find_volume(file_blocks: u32, mut read: impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>) -> Result<Volume, &'static str> {
    let mut b = [0u8; BLOCK];
    if file_blocks < 3 {
        return Err("too small for a volume");
    }
    read(0, &mut b)?;
    if b[..2] == *b"ER" {
        let block_size = u16::from_be_bytes([b[2], b[3]]);
        if block_size != 512 {
            return Err("partition map isn't in 512-byte blocks");
        }
        read(1, &mut b)?;
        let entries = u32::from_be_bytes([b[4], b[5], b[6], b[7]]).min(file_blocks - 1);
        for i in 1..=entries {
            if i > 1 {
                read(i, &mut b)?;
            }
            if b[..2] != *b"PM" {
                break;
            }
            let field = |at: usize| u32::from_be_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]);
            let kind = &b[48..80];
            let kind = &kind[..kind.iter().position(|&c| c == 0).unwrap_or(kind.len())];
            if kind == b"Apple_HFS" {
                let (first, blocks) = (field(8), field(12));
                if u64::from(first) + u64::from(blocks) > u64::from(file_blocks) || blocks < 3 {
                    return Err("HFS partition runs past the file");
                }
                if blocks > MAX_BLOCKS {
                    return Err("HFS partition over 8 GB");
                }
                return Ok(Volume { first, blocks, kind: "APM HFS" });
            }
        }
        return Err("no Apple_HFS partition");
    }
    // The master directory block (or MFS's volume information) is block 2.
    read(2, &mut b)?;
    let kind = match [b[0], b[1]] {
        [0x42, 0x44] => "HFS",
        [0xD2, 0xD7] => "MFS",
        [0x48, 0x2B] | [0x48, 0x58] => return Err("HFS+ (the HD20 driver needs HFS)"),
        _ => return Err("not an HFS volume or disk image"),
    };
    if file_blocks > MAX_BLOCKS {
        return Err("volume over 8 GB");
    }
    Ok(Volume { first: 0, blocks: file_blocks, kind })
}
