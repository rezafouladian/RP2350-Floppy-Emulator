//! Drive side of the Mac floppy interface: always a SuperDrive (FDHD), holding a 400K/800K
//! (DD, GCR) or 1.44 MB (HD, MFM) disk. Only R15 (media density) changes with the disk,
//! so any image can follow any other without the Mac restarting.
//! R11 follows the Mac's GCR/MFM mode command; the stream format doesn't.
//!
//! Work is split three ways:
//! - **PIO0 SM0** plays the current track's cell stream on RD: one cell per bit, a short low
//!   pulse for each `1`. Two DMA channels loop the track buffer forever.
//! - **PIO1** captures writes: `capture.pio` turns WR into cells while /WRREQ is low, and DMA
//!   channel 2 streams them into [`RING`].
//! - **Core 1** spins with interrupts off, decodes CA0–CA2/SEL into a register number,
//!   and drives RD through the pad's output/OE override bits:
//!   tri-stated while /ENBL is high, the PIO stream for R8/R9, or a forced status level otherwise.
//!   It executes LSTRB commands (step, motor, eject), opens and closes write captures, and owns
//!   the output DMA: it switches the stream to the other side's buffer when the Mac selects R8 or
//!   R9, and to a new cylinder when core 0 publishes one.
//! - **Core 0** (main) reads the head's cylinder from the SD card, builds both sides into a free
//!   buffer set and publishes it ([`PUBLISHED`]). It decodes captured writes, republishes the
//!   changed cylinder and writes the changed blocks back to the card. /READY reads "not ready"
//!   until core 1 is playing the cylinder the head is on.
//!
//! In HD20 mode core 1 runs [`run_dcd`] instead, and in Apple II mode [`run_a2`], a Disk II
//! drive: no register file, a stepper driven by the phase lines, and write protect on a pin of
//! its own ([`PIN_WRPROT`]). In Apple II 3.5" mode it runs [`run_a35`], an Apple 3.5 Drive: the
//! Mac 800K drive's register file (it's the same Sony mechanism), sensed on WRPROT as well as RD,
//! and selected only while /3.5DISK ([`PIN_EN35`]) says so.

use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering::{Acquire, Relaxed, Release}};

use rp235x_hal::pac;
use rp235x_hal::pio::{
    Buffers, PIOBuilder, PIOExt, PinDir, Running, Rx as PioRx, ShiftDirection, StateMachine, Stopped, Tx, SM0, SM1,
    SM2,
};

use crate::smartport::{self, Framed, Framer, Handshake, Phase, Rx};
use crate::{a2, gcr};

// GPIO header pins. SEL, CA0, CA1, CA2 must stay consecutive in this order so that
// `(GPIO_IN >> PIN_SEL) & 0xF` is the register number R = CA2·8 + CA1·4 + CA0·2 + SEL.
// `main` configures the same pins by number; keep the two in sync.
//
//   Signal   GPIO  Header pin   Mac DB-19   Mac internal 20-pin
//   SEL      2     3            16          12
//   CA0/PH0  3     5            11          2
//   CA1/PH1  4     7            12          4
//   CA2/PH2  5     29           13          6
//   LSTRB    8     24           14          8
//   /ENBL    9     21           17          14
//   /WRREQ   10    19           15          10
//   WR       11    23           19          18
//   RD       12    32           18          16
//   WRPROT   13    33           10          20          Apple II modes only (the Mac's PWM)
//   /3.5DISK 14    8            4           7           Apple II 3.5" mode only (Mac: GND)
//   /DRIVE2  15    10           9           -           Apple II 3.5" mode, optional
//   GND      -     6, 9, ...    1-4         1, 3, 5, 7
//
// In Apple II mode CA0–CA2 and LSTRB are the stepper phases PH0–PH3, and SEL is unused. On a
// Disk II controller's 20-pin cable, pin 12 (SEL here) is +5 V; on a IIgs, DB-19 pin 4 is
// /3.5DISK, a signal, not ground. In Apple II 3.5" mode they're CA0–CA2, LSTRB and SEL (the
// IIgs's HDSEL) as on the Mac.
pub const PIN_SEL: u8 = 2;
pub const PIN_CA0: u8 = 3;
pub const PIN_CA1: u8 = 4;
pub const PIN_CA2: u8 = 5;
pub const PIN_LSTRB: u8 = 8;
pub const PIN_ENBL: u8 = 9;
pub const PIN_WRREQ: u8 = 10;
pub const PIN_WR: u8 = 11;
pub const PIN_RD: u8 = 12;
/// Apple II write protect, out (high = protected). An input in the Mac modes, where the Mac's
/// PWM line is on that pin.
pub const PIN_WRPROT: u8 = 13;
/// Apple II 3.5" mode: /3.5DISK (DB-19 pin 4 on a IIgs or IIc+), low while the computer is using
/// its 3.5" drives rather than the 5.25" ones and SmartPort devices on the same port (IIgs DISKREG
/// bit 6). Pulled down, so an unwired pin selects the drive whenever /ENBL does.
pub const PIN_EN35: u8 = 14;
/// Apple II 3.5" mode: /DRIVE2 (DB-19 pin 9 on a IIgs), the second drive's enable. The drive
/// answers only while it's high. Pulled up; leave it unwired on a IIc+
/// (whose pin 9 is something else).
pub const PIN_DRIVE2: u8 = 15;

const _: () = assert!(
    PIN_CA0 == PIN_SEL + 1 && PIN_CA1 == PIN_SEL + 2 && PIN_CA2 == PIN_SEL + 3 && PIN_RD < 32
);

/// PIO clock for GCR: the Mac's bit cell is 16 periods of the 7.8336 MHz IWM clock
/// (2.0425 µs), and the PIO program spends 16 cycles per cell.
pub const GCR_PIO_HZ: u32 = 7_833_600;
/// PIO clock for MFM: 16 cycles per 1 µs half-cell (500 kbit/s HD media).
pub const MFM_PIO_HZ: u32 = 16_000_000;

/// PIO clock for the Apple II's 4 µs cells (300 RPM, 50,000 per revolution, as WOZ images are
/// stored), 16 cycles per cell.
pub const A2_PIO_HZ: u32 = 4_000_000;
/// The Apple II writes a cell every 8 cycles of its 2.0455 MHz Q3 clock (14.31818 MHz / 7), as
/// the IIc's and IIgs's IWM do in 28 of their 7.16 MHz clocks: 3.911 µs. 32 capture cycles each.
pub const A2_CAPTURE_HZ: u32 = 8_181_818;
/// PIO clock for sector images: they play at that write cell (16 cycles each), so a write is laid
/// into the track cell for cell where it was made (`writes::splice_a2`). Disks an Apple II wrote
/// read back at it too.
pub const A2_SECTOR_PIO_HZ: u32 = A2_CAPTURE_HZ / 2;

/// The IIgs writes its 3.5" disks at 14 cycles of its 7.15909 MHz IWM clock a cell, 1.9556 µs
///, 4% short of the Mac's: 32 capture cycles each.
pub const A35_CAPTURE_HZ: u32 = 16_363_636;

/// Write capture clocks: 32 PIO cycles per cell (see `capture.pio`).
const GCR_CAPTURE_HZ: u32 = 2 * GCR_PIO_HZ;
const MFM_CAPTURE_HZ: u32 = 2 * MFM_PIO_HZ;

const DMA_DATA: usize = 0;
const DMA_CTRL: usize = 1;
const DMA_CAPTURE: usize = 2;
/// Apple II mode: puts each revolution's cell count in the FIFO ahead of its words
/// ([`STREAM_COUNT`], `src/disk2.pio`).
const DMA_COUNT: usize = 3;

/// A write gate open longer than this is a format, not a sector write: the GCR formatter then
/// expects the next header to be sector 0, so the stream restarts at word 0.
const FORMAT_GATE_US: u32 = 50_000;
/// Keep capturing this long after the gate closes, so the separator shifts out the last cells
/// (32 cells of 0s push the final partial word).
const CAPTURE_TAIL_US: u32 = 100;
/// The same for a Disk II, whose cells are twice as long: 36 of them. A gate that opens again
/// within it carries on with the same capture ([`CAP_REST`]): DOS 3.3's formatter turns write mode
/// off between fields for ~11 cells, and stopping the separator there lost what it hadn't pushed.
const A2_CAPTURE_TAIL_US: u32 = 140;
/// Index pulse, once per playing-track revolution: on R7 in MFM mode, and on R8/R9 while writing
/// to an MFM disk, for the MFM formatter.
const INDEX_LOW_US: u32 = 2_000;
/// Index period when no track is playing: 300 RPM.
const INDEX_PERIOD_US: u32 = 200_000;
/// Apple 3.5 Drive: this long after /3.5DISK goes high (the computer has turned to its 5.25"
/// drives), the motor stops.
const A35_MOTOR_OFF_US: u32 = 500_000;
/// Apple 3.5 Drive: after this long empty, another disk may go in whether or not the Apple II has
/// sensed R1 (ProDOS 8 doesn't poll); the disk-switched latch (R6) tells it the disk changed.
const A35_EMPTY_US: u32 = 2_000_000;

// ---- State shared between the cores ------------------------------------------------------

/// Disk in place. Set by core 0 on insertion, cleared by core 1 on an EJECT command or by core 0
/// when the disk is ejected from the knob.
pub static DISK_IN: AtomicBool = AtomicBool::new(false);
/// The Mac knows the drive is empty: it ejected the disk itself, or sensed R1 (/CSTIN) while no
/// disk was in. The `.Sony` driver only checks R1 at the start of a request, so after an
/// eject it didn't ask for, core 0 waits for this before inserting another disk; otherwise the
/// driver would take the new disk for the one it has mounted. Cleared by core 0 when it ejects.
pub static MAC_SAW_EMPTY: AtomicBool = AtomicBool::new(true);
/// The disk is HD media (R15 = 0: 1.44 MB) rather than DD (R15 = 1: GCR or 720K). Set by core 0
/// as soon as the image type is known, before the disk is inserted.
pub static HD_MEDIA: AtomicBool = AtomicBool::new(false);
/// The disk is MFM (720K or 1.44 MB): writes go through the MFM separator, and R8/R9 show the
/// index pulse while writing. Set by core 0 before the disk is inserted.
pub static MFM: AtomicBool = AtomicBool::new(false);
/// The disk accepts writes (R3). Set by core 0 before the disk is inserted.
pub static WRITABLE: AtomicBool = AtomicBool::new(false);
/// What plays after a step or side switch. Off: the new track from word 0 (its first sector next,
/// so a read of a cylinder's first block finds it at once). On: the new track from the same
/// fraction of a revolution, as if the disk kept turning, like a real drive. The `.Sony`
/// track cache fills with the sectors that pass while it waits for the one it wants, so
/// this changes how single-block reads perform. Set from the knob menu, and always on for a MOOF
/// imaged with cross-track sync (`moof::Moof::synchronized`): The Ancient Art of War's protection
/// steps from track 78 to 79 just after sector 0 and counts nibbles to a mark on 79.
pub static KEEP_ROTATION: AtomicBool = AtomicBool::new(false);
/// Head position, published by core 1.
pub static HEAD_TRACK: AtomicU8 = AtomicU8::new(0);
/// Head (side) selected by the last R8/R9 access, published by core 1.
pub static SIDE: AtomicU8 = AtomicU8::new(0);
pub static MOTOR_ON: AtomicBool = AtomicBool::new(false);
/// Set by core 1 whenever /ENBL is low; core 0 clears it after showing it.
pub static SELECTED: AtomicBool = AtomicBool::new(false);
pub static STEPS: AtomicU32 = AtomicU32::new(0);
pub static EJECTS: AtomicU32 = AtomicU32::new(0);
pub static STROBES: AtomicU32 = AtomicU32::new(0);
/// Last command as `CA1 CA0 SEL CA2`: `(latch << 1) | value`.
pub static LAST_CMD: AtomicU8 = AtomicU8::new(0xFF);
/// `diag` builds, the Mac drive loop: commands by [`LAST_CMD`] value, and housekeeping's samples
/// of the register addressed while selected with the motor on, and of those with /READY high.
/// Core 0 clears them when a disk goes in ([`diag_reset`]).
#[cfg(feature = "diag")]
pub static CMD_COUNT: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];
#[cfg(feature = "diag")]
pub static REG_SAMPLES: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];
#[cfg(feature = "diag")]
pub static NOT_READY: AtomicU32 = AtomicU32::new(0);

/// `diag` builds, the Mac drive loop: an event log, which core 1 alone writes and core 0 saves to
/// the card (`main::DiagLog`). Each event is core 1's timer (µs) and `kind << 24 | data`.
#[cfg(feature = "diag")]
pub const LOG_LEN: usize = 4096;
#[cfg(feature = "diag")]
pub static LOG_T: [AtomicU32; LOG_LEN] = [const { AtomicU32::new(0) }; LOG_LEN];
#[cfg(feature = "diag")]
pub static LOG_W: [AtomicU32; LOG_LEN] = [const { AtomicU32::new(0) }; LOG_LEN];
/// Events ever logged.
#[cfg(feature = "diag")]
pub static LOG_HEAD: AtomicU32 = AtomicU32::new(0);
/// The drive was selected (/ENBL low); data: the register addressed | RD's level for it << 4.
#[cfg(feature = "diag")]
pub const LOG_SEL: u32 = 1;
/// Deselected; data: register changes while it was selected.
#[cfg(feature = "diag")]
pub const LOG_DESEL: u32 = 2;
/// Another register addressed while selected (the first [`LOG_REG_MAX`] of a selection); data as
/// [`LOG_SEL`].
#[cfg(feature = "diag")]
pub const LOG_REG: u32 = 3;
#[cfg(feature = "diag")]
pub const LOG_REG_MAX: u32 = 24;
/// A command; data: its [`LAST_CMD`] value | the head's track after it << 8.
#[cfg(feature = "diag")]
pub const LOG_CMD: u32 = 4;
/// Core 1 took the disk as in (1) or out (0).
#[cfg(feature = "diag")]
pub const LOG_DISK: u32 = 5;

#[cfg(feature = "diag")]
#[inline(always)]
fn log(t: u32, kind: u32, data: u32) {
    let head = LOG_HEAD.load(Relaxed);
    let k = head as usize % LOG_LEN;
    LOG_T[k].store(t, Relaxed);
    LOG_W[k].store(kind << 24 | data & 0xFF_FFFF, Relaxed);
    LOG_HEAD.store(head.wrapping_add(1), Release);
}

#[cfg(feature = "diag")]
pub fn diag_reset() {
    for c in CMD_COUNT.iter().chain(REG_SAMPLES.iter()) {
        c.store(0, Relaxed);
    }
    NOT_READY.store(0, Relaxed);
}
/// Core 1 loop iterations, in units of 256. Core 0 turns this into a loop time.
pub static LOOPS: AtomicU32 = AtomicU32::new(0);

/// Read address the DMA control channel reloads into the data channel on each wrap.
static STREAM_PTR: AtomicU32 = AtomicU32::new(0);
/// Apple II mode: a revolution's cells less one, which the count channel puts in the FIFO ahead
/// of each revolution's words (`src/disk2.pio`).
static STREAM_COUNT: AtomicU32 = AtomicU32::new(0);

// ---- Write captures ----------------------------------------------------------------------
// While /WRREQ is low, DMA channel 2 streams captured cells into RING (it wraps by itself).
// When the gate closes, core 1 queues where the capture is and what was under the head; core 0
// decodes it.

/// Capture ring size in words: 32 KB, the most a DMA ring can wrap. Holds more than one MFM
/// revolution (~6,250 words) or ~40 GCR sector writes (~190 words each).
pub const RING_WORDS: usize = 8_192;

#[repr(C, align(32768))]
pub struct Ring(UnsafeCell<[u32; RING_WORDS]>);

// Safety: written only by DMA; core 0 reads finished captures through `word`.
unsafe impl Sync for Ring {}

/// At the start of RAM (`memory.x`'s `.ring`), where its 32 KB alignment costs no padding. Not
/// zeroed at startup: the DMA writes each word before core 0 or 1 reads it.
#[unsafe(link_section = ".ring")]
pub static RING: Ring = Ring(UnsafeCell::new([0; RING_WORDS]));

impl Ring {
    /// Word `i` (wrapping) of the ring.
    pub fn word(&self, i: usize) -> u32 {
        unsafe { read_volatile((self.0.get() as *const u32).add(i % RING_WORDS)) }
    }

    fn base(&self) -> u32 {
        self.0.get() as u32
    }
}

pub const CAPTURE_QUEUE: usize = 16;
/// Finished captures, by `CAP_HEAD % CAPTURE_QUEUE`: first ring word, length in words, the
/// cell under the head when the gate opened (in the side buffer that was playing), the
/// cylinder that was playing (a [`PUBLISHED`] value) with [`CAP_SIDE`]/[`CAP_FORMAT`] flags,
/// and the ring total at its start (see [`CAP_TOTAL`]).
pub static CAP_START: [AtomicU32; CAPTURE_QUEUE] = [const { AtomicU32::new(0) }; CAPTURE_QUEUE];
pub static CAP_WORDS: [AtomicU32; CAPTURE_QUEUE] = [const { AtomicU32::new(0) }; CAPTURE_QUEUE];
pub static CAP_POS: [AtomicU32; CAPTURE_QUEUE] = [const { AtomicU32::new(0) }; CAPTURE_QUEUE];
pub static CAP_INFO: [AtomicU32; CAPTURE_QUEUE] = [const { AtomicU32::new(0) }; CAPTURE_QUEUE];
pub static CAP_SEQ: [AtomicU32; CAPTURE_QUEUE] = [const { AtomicU32::new(0) }; CAPTURE_QUEUE];
pub const CAP_SIDE: u32 = 1 << 20;
pub const CAP_FORMAT: u32 = 1 << 21;
/// The rest of the capture before it in the ring: a Disk II's gate closed and opened again within
/// [`A2_CAPTURE_TAIL_US`], and the separator kept running (`writes::splice_a2`).
pub const CAP_REST: u32 = 1 << 22;
/// HD20 + floppy mode: a capture that is an HD20 command (`hd20::Server`), not a floppy write.
pub const CAP_DCD: u32 = 1 << 23;
/// Captures queued by core 1 / taken by core 0.
pub static CAP_HEAD: AtomicU32 = AtomicU32::new(0);
pub static CAP_TAIL: AtomicU32 = AtomicU32::new(0);
/// Captures lost because the queue was full.
pub static CAP_DROPPED: AtomicU32 = AtomicU32::new(0);
/// Words ever written to the ring, including a capture in progress. A capture that started at
/// ring total `seq` is intact while `CAP_TOTAL <= seq + RING_WORDS`.
pub static CAP_TOTAL: AtomicU32 = AtomicU32::new(0);

/// Queue a capture of `words` ring words from `start` for core 0 (or count it lost), and add them
/// to the ring total.
#[inline(always)]
fn queue_capture(start: u32, words: u32, pos: u32, info: u32, total: &mut u32) {
    let head = CAP_HEAD.load(Relaxed);
    if head.wrapping_sub(CAP_TAIL.load(Acquire)) < CAPTURE_QUEUE as u32 {
        let k = head as usize % CAPTURE_QUEUE;
        CAP_START[k].store(start, Relaxed);
        CAP_WORDS[k].store(words, Relaxed);
        CAP_POS[k].store(pos, Relaxed);
        CAP_INFO[k].store(info, Relaxed);
        CAP_SEQ[k].store(*total, Relaxed);
        CAP_HEAD.store(head.wrapping_add(1), Release);
    } else {
        CAP_DROPPED.fetch_add(1, Relaxed);
    }
    *total = total.wrapping_add(words);
    CAP_TOTAL.store(*total, Release);
}
/// A write gate is open, or its capture is still being finished. Goes false (Release) only
/// once the capture is queued.
pub static WRITING: AtomicBool = AtomicBool::new(false);

// Cylinder hand-off. Core 0 fills buffer set `s` (one track buffer per side), stores each
// side's address and length here, then stores `cylinder(s, track)` in PUBLISHED. Core 1 switches
// to it and echoes the value in PLAYING; from then on the other set is free for core 0.
// A single-sided image points both sides at the same buffer.

/// Buffer addresses, `[set][side]`.
pub static CYL_PTR: [[AtomicU32; 2]; 2] = [const { [const { AtomicU32::new(0) }; 2] }; 2];
/// Buffer lengths in words, `[set][side]`.
pub static CYL_LEN: [[AtomicU32; 2]; 2] = [const { [const { AtomicU32::new(0) }; 2] }; 2];
/// Where each buffer's word 0 is in its track as imaged, in words, `[set][side]`: a GCR MOOF
/// track's rotation (`moof::find_rotation`), 0 for the rest. [`KEEP_ROTATION`] keeps the angle in
/// these terms, so a MOOF's tracks line up as they did on the disk.
pub static CYL_ROT: [[AtomicU32; 2]; 2] = [const { [const { AtomicU32::new(0) }; 2] }; 2];
/// Per set, for a Disk II: the quarter tracks that hear its track ([`span`]). A real drive reads
/// the same data across them, so core 1 plays on as the head moves among them, not noise until
/// core 0 publishes the same buffers again (Hard Hat Mack's track two tracks wide; loaders that
/// step a quarter track and read on).
pub static CYL_SPAN: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

/// The highest quarter track (a WOZ's TMAP has 160).
pub const A2_LAST_QUARTER: u8 = 159;

/// Quarter tracks `lo..=hi`, as [`CYL_SPAN`] holds them.
pub const fn span(lo: u8, hi: u8) -> u32 {
    lo as u32 | (hi as u32) << 8
}

/// Quarter track `q` is in a [`CYL_SPAN`] value.
#[inline(always)]
const fn in_span(span: u32, q: u8) -> bool {
    (q as u32).wrapping_sub(span & 0xFF) <= (span >> 8 & 0xFF).wrapping_sub(span & 0xFF)
}

/// Apple II mode, per set: the track's cells (0: the noise buffer, no track), its rotation in
/// cells (the stored cell at buffer cell 0, so cells as imaged are found), and 2^32 / cells (for
/// a fraction of a revolution without a division core 1 would have to call).
pub static CYL_BITS: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
pub static CYL_ROTC: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
pub static CYL_INV: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
/// Apple II mode: cells a µs × 2^32 at the loaded disk's PIO clock ([`a2_cells_q32`]), for
/// where the disk is from the timer.
pub static A2_CELLS_Q32: AtomicU32 = AtomicU32::new(1 << 30);
/// A Disk II's tracks, per set: the PIO clock its track plays at ([`clkdiv_word`]), and its
/// cells a µs × 2^32 ([`a2_cells_q32`]). A WOZ's (or A2R's) tracks each play their cells in a
/// 200 ms revolution (`woz::track_hz`), as the disk turned; others at the disk's cell.
pub static CYL_CLKDIV: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
pub static CYL_Q32: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

/// [`A2_CELLS_Q32`] for cells of 16 cycles at `pio_hz`, through the divisor the PIO really
/// gets: core 1 counts cells from the timer for as long as the head stays on a track.
pub fn a2_cells_q32(sys_hz: u32, pio_hz: u32) -> u32 {
    let (int, frac) = clock_divisor(sys_hz, pio_hz);
    let div = u128::from(int) << 8 | u128::from(frac);
    ((u128::from(sys_hz) << 40) / (div * 16 * 1_000_000)) as u32
}
/// Latest cylinder published by core 0, or 0 for none yet.
pub static PUBLISHED: AtomicU32 = AtomicU32::new(0);
/// Cylinder core 1 is playing, or 0 for none yet.
pub static PLAYING: AtomicU32 = AtomicU32::new(0);

const CYL_VALID: u32 = 1 << 16;

/// Encode a buffer set and track for [`PUBLISHED`]/[`PLAYING`].
pub const fn cylinder(set: usize, track: u8) -> u32 {
    CYL_VALID | (set as u32) << 8 | track as u32
}

/// Buffer set of a [`PUBLISHED`]/[`PLAYING`] value.
pub const fn cylinder_set(c: u32) -> usize {
    (c >> 8 & 1) as usize
}

/// Track of a [`PUBLISHED`]/[`PLAYING`] value, if any.
pub const fn cylinder_track(c: u32) -> Option<u8> {
    if c & CYL_VALID != 0 { Some(c as u8) } else { None }
}

/// In dual mode, set in a [`PUBLISHED`]/[`PLAYING`] (and so [`CAP_INFO`]) value for drive B, the
/// Disk II, whose track is then a quarter track. `Cylinder::apply` masks it off with the rest of
/// the flags.
pub const CYL_DRIVE2: u32 = 1 << 17;

/// The same cylinder of the same drive, whichever buffer set it's in.
const fn same_cylinder(a: u32, b: u32) -> bool {
    a & !(1 << 8) == b & !(1 << 8)
}

// ---- Track stream (PIO + DMA) -------------------------------------------------------------

/// Owns the PIO state machine that plays the track. Keep it alive; core 1 drives the DMA
/// through [`Stream::player`].
pub struct Stream {
    _sm: StateMachine<(pac::PIO0, SM0), Running>,
    _tx: Tx<(pac::PIO0, SM0)>,
    pub player: Player,
}

/// The read pulses [`Stream`] plays, and on which pins.
#[derive(Clone, Copy, PartialEq)]
pub enum Pulses {
    /// The Mac's drives, on RD (HD20 replies and HD20 + floppy mode).
    Mac,
    /// The Mac's, on [`PIN_RD_STREAM`], which [`Mux`] puts on RD while R8/R9 are addressed:
    /// floppy mode.
    MacMux,
    /// A Disk II's, at 4 µs cells, on RD (SmartPort replies, a byte at a time).
    DiskII,
    /// A Disk II's that loops each track at exactly its cells (`src/disk2.pio`), for Apple II
    /// mode. Tracks are started with [`Player::play_a2`].
    DiskIIExact,
    /// An Apple 3.5 Drive's: the Mac's, on [`PIN_RD_STREAM`], which [`Mux`] puts on RD and
    /// WRPROT both (see [`run_a35`]).
    Apple35,
    /// Dual mode: [`Pulses::Apple35`]'s program for drive A, and beside it
    /// [`Pulses::DiskIIExact`]'s for drive B, which core 1 switches between
    /// ([`Player::use_program`]).
    Dual,
}

impl Stream {
    /// Load the bit-cell program on PIO0 SM0 driving `PIN_RD` (and `PIN_WRPROT` for
    /// [`Pulses::Dual`]; [`PIN_RD_STREAM`] instead for [`Pulses::MacMux`] and
    /// [`Pulses::Apple35`]), and take DMA channels 0 and 1. The pins must already be in
    /// `FunctionPio0`.
    pub fn new(pio0: pac::PIO0, resets: &mut pac::RESETS, sys_hz: u32, pulses: Pulses) -> Self {
        // Each cell is 16 cycles. The falling edge comes at cycle 9; a `1` holds RD low for
        // 7 cycles (0.89 µs) and RD is high for at least 9 cycles (1.15 µs) before each edge.
        let mac = pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "cell:",
            "    out x, 1      side 1 [7]",
            "    jmp !x zero   side 1",
            "    jmp cell      side 0 [6]",
            "zero:",
            "    nop           side 1 [6]",
            ".wrap",
        );
        // Apple II, at 4 µs cells: a `1` is a 1 µs low pulse (4 cycles) from cycle 12, as a
        // Disk II's read amplifier gives.
        let disk2 = pio::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "cell:",
            "    out x, 1      side 1 [10]",
            "    jmp !x zero   side 1",
            "    jmp cell      side 0 [3]",
            "zero:",
            "    nop           side 1 [3]",
            ".wrap",
        );
        // The Mac's, driving RD and WRPROT together (two side-set pins leave 3 delay bits: the
        // longest delay is 7).
        let sony = pio::pio_asm!(
            ".side_set 2",
            ".wrap_target",
            "cell:",
            "    out x, 1      side 3 [7]",
            "    jmp !x zero   side 3",
            "    jmp cell      side 0 [6]",
            "zero:",
            "    nop           side 3 [6]",
            ".wrap",
        );
        let exact = pio::pio_file!("src/disk2.pio", select_program("disk2"));
        let program = match pulses {
            Pulses::Mac | Pulses::MacMux | Pulses::Apple35 => &mac.program,
            Pulses::DiskII => &disk2.program,
            Pulses::DiskIIExact => &exact.program,
            Pulses::Dual => &sony.program,
        };
        let pin = if matches!(pulses, Pulses::MacMux | Pulses::Apple35) { PIN_RD_STREAM } else { PIN_RD };
        let (mut pio, sm0, _, _, _) = pio0.split(resets);
        let installed = pio.install(program).unwrap();
        let offset = installed.offset();
        // Where `disk2` is: the program played in Apple II mode, beside drive A's in dual mode.
        let exact_at = match pulses {
            Pulses::DiskIIExact => offset,
            Pulses::Dual => pio.install(&exact.program).unwrap().offset(),
            _ => 0,
        };
        // What `Player::play_a2` executes to start a track: `pull`, `mov y, osr`, `out null, k`
        // (k in the low 5 bits) and `jmp cell`, all with RD high.
        let encode = |operands| pio::Instruction { operands, delay: 0, side_set: Some(1) }.encode(exact.program.side_set);
        let a2_exec = [
            encode(pio::InstructionOperands::PULL { if_empty: false, block: true }),
            encode(pio::InstructionOperands::MOV { destination: pio::MovDestination::Y, op: pio::MovOperation::None, source: pio::MovSource::OSR }),
            encode(pio::InstructionOperands::OUT { destination: pio::OutDestination::NULL, bit_count: 1 }) & !0x1F,
            encode(pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: exact_at + exact.public_defines.cell as u8,
            }),
        ];
        // Dual mode: what `Player::use_program` executes to go back to drive A's program.
        let sony_jmp = pio::Instruction {
            operands: pio::InstructionOperands::JMP { condition: pio::JmpCondition::Always, address: offset },
            delay: 0,
            side_set: Some(3),
        }
        .encode(sony.program.side_set);
        let (div_int, div_frac) = clock_divisor(sys_hz, GCR_PIO_HZ);
        let (mut sm, _rx, tx) = PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(pin)
            .out_shift_direction(ShiftDirection::Left)
            .autopull(pulses != Pulses::DiskIIExact)
            .pull_threshold(0) // 0 means 32
            .buffers(rp235x_hal::pio::Buffers::OnlyTx)
            .clock_divisor_fixed_point(div_int, div_frac)
            .build(sm0);
        sm.set_pindirs([(pin, PinDir::Output)]);
        if pulses == Pulses::Dual {
            const { assert!(PIN_WRPROT == PIN_RD + 1) };
            sm.set_pindirs([(PIN_WRPROT, PinDir::Output)]);
        }
        let txf = tx.fifo_address() as u32;
        let dreq = tx.dreq_value();
        let sm = sm.start();

        resets.reset().modify(|_, w| w.dma().clear_bit());
        while resets.reset_done().read().dma().bit_is_clear() {}

        let pio0 = unsafe { &*pac::PIO0::ptr() };
        let dma = unsafe { &*pac::DMA::ptr() };
        let data = dma.ch(DMA_DATA);
        let ctrl = dma.ch(DMA_CTRL);
        let count = dma.ch(DMA_COUNT);

        // Work out the CTRL words with the PAC, written disabled so nothing starts yet.
        // Control channel: copy STREAM_PTR into the data channel's READ_ADDR_TRIG, which
        // restarts it with its transfer count reloaded.
        ctrl.ch_al1_ctrl().write(|w| unsafe {
            w.data_size().size_word();
            w.incr_read().clear_bit();
            w.incr_write().clear_bit();
            w.treq_sel().bits(0x3F); // unpaced
            w.chain_to().bits(DMA_CTRL as u8); // chaining to itself = no chain
            w.irq_quiet().set_bit()
        });
        // Data channel: track buffer -> PIO TX FIFO, then hand over to the control channel (in
        // Apple II mode, to the count channel first).
        const EN: u32 = 1;
        let data_cfg = |chain: usize| {
            data.ch_al1_ctrl().write(|w| unsafe {
                w.data_size().size_word();
                w.incr_read().set_bit();
                w.incr_write().clear_bit();
                w.treq_sel().bits(dreq);
                w.chain_to().bits(chain as u8);
                w.irq_quiet().set_bit()
            });
            data.ch_al1_ctrl().read().bits() | EN
        };
        let exact_cfg = data_cfg(DMA_COUNT);
        let data_cfg = data_cfg(if pulses == Pulses::DiskIIExact { DMA_COUNT } else { DMA_CTRL });
        // Count channel (Apple II mode): STREAM_COUNT -> PIO TX FIFO, once, then the control
        // channel.
        count.ch_al1_ctrl().write(|w| unsafe {
            w.data_size().size_word();
            w.incr_read().clear_bit();
            w.incr_write().clear_bit();
            w.treq_sel().bits(dreq);
            w.chain_to().bits(DMA_CTRL as u8);
            w.irq_quiet().set_bit()
        });

        // Dual mode: SM0's settings for drive A's program (as built) and drive B's (`disk2`: no
        // autopull, one side-set pin, its own wrap), and the data channel's chain for each.
        let regs = pio0.sm(0);
        let (shift, exec, pins) = (regs.sm_shiftctrl().read().bits(), regs.sm_execctrl().read().bits(), regs.sm_pinctrl().read().bits());
        let wrap = |w: pio::Wrap| (exec & !(0x3FF << 7)) | u32::from(exact_at) + u32::from(w.source) << 12 | u32::from(exact_at) + u32::from(w.target) << 7;
        let programs = [
            [shift, exec, pins, data_cfg],
            [shift & !(1 << 17), wrap(exact.program.wrap), pins & !(7 << 29) | 1 << 29, exact_cfg],
        ];

        let player = Player {
            abort: dma.chan_abort().as_ptr(),
            data_read: data.ch_read_addr().as_ptr(),
            data_write: data.ch_write_addr().as_ptr(),
            data_count: data.ch_trans_count().as_ptr(),
            data_ctrl: data.ch_al1_ctrl().as_ptr(),
            data_ctrl_trig: data.ch_ctrl_trig().as_ptr(),
            data_cfg,
            data_restart: data.ch_al3_read_addr_trig().as_ptr() as u32,
            ctrl_read: ctrl.ch_read_addr().as_ptr(),
            ctrl_write: ctrl.ch_write_addr().as_ptr(),
            ctrl_count: ctrl.ch_trans_count().as_ptr(),
            ctrl_ctrl: ctrl.ch_al1_ctrl().as_ptr(),
            ctrl_cfg: ctrl.ch_al1_ctrl().read().bits() | EN,
            shiftctrl: pio0.sm(0).sm_shiftctrl().as_ptr(),
            flevel: pio0.flevel().as_ptr(),
            clkdiv: pio0.sm(0).sm_clkdiv().as_ptr(),
            txf,
            count_read: count.ch_read_addr().as_ptr(),
            count_write: count.ch_write_addr().as_ptr(),
            count_count: count.ch_trans_count().as_ptr(),
            count_ctrl: count.ch_al1_ctrl().as_ptr(),
            count_cfg: count.ch_al1_ctrl().read().bits() | EN,
            pio_ctrl: pio0.ctrl().as_ptr(),
            sm_instr: pio0.sm(0).sm_instr().as_ptr(),
            sm_execctrl: pio0.sm(0).sm_execctrl().as_ptr(),
            a2_exec,
            pinctrl: pio0.sm(0).sm_pinctrl().as_ptr(),
            programs,
            sony_jmp,
        };
        Self { _sm: sm, _tx: tx, player }
    }

    /// Change the cell rate (`GCR_PIO_HZ` or `MFM_PIO_HZ`). Call before the first cylinder is
    /// published.
    pub fn set_cell_rate(&self, sys_hz: u32, pio_hz: u32) {
        let (int, frac) = clock_divisor(sys_hz, pio_hz);
        let pio0 = unsafe { &*pac::PIO0::ptr() };
        pio0.sm(0).sm_clkdiv().write(|w| unsafe { w.int().bits(int).frac().bits(frac) });
    }
}

// ---- Register mux (PIO2) ------------------------------------------------------------------
// The Mac samples SENSE a few hundred ns after changing SEL and CA0–CA2. An LC III, with
// its SWIM in ISM mode, did it sooner than core 1's loop follows the lines: it read the last
// register's bit (R6's 0 as R2, `cantStepErr`; R7's 0 as R1, a disk in an empty drive) and
// missed short LSTRB pulses (`seekErr`). So in the Mac floppy and Apple II 3.5" modes PIO2
// answers:
// - SM0 jumps to entry r (SEL, CA0–CA2) of a 16-instruction table at address 0, which side-sets
//   RD (and WRPROT, for an Apple II) to register r's bit and jumps back: ~20 ns from the lines
//   to RD. Entries 8 and 9 copy the track stream instead, which PIO0 SM0 plays on
//   [`PIN_RD_STREAM`]. Core 1 rewrites an entry when its bit changes ([`MuxPort::show`]).
// - SM1 gives RD its output enable while /ENBL is low.
// - SM2 takes the lines on each LSTRB rising edge, for core 1's commands ([`MuxPort::strobe`]).

/// Where PIO0 SM0 plays the track in the modes [`Mux`] answers: a pin connected to nothing,
/// which SM0 of PIO2 reads with `jmp pin`.
pub const PIN_RD_STREAM: u8 = 27;

/// In a [`MuxPort::show`] value: R8 and R9 play the track (else bits 8 and 9 are their levels,
/// the index pulse while writing to an MFM disk).
const MUX_DATA: u32 = 1 << 16;

/// The table's loop (`sense`), just past it: where its program is installed (`.origin 16`).
const MUX_LOOP: u8 = 16;

/// The PIO2 state machines (keep it alive) and [`MuxPort`], core 1's handle on them.
pub struct Mux {
    _sm0: StateMachine<(pac::PIO2, SM0), Running>,
    _sm1: StateMachine<(pac::PIO2, SM1), Running>,
    _sm2: StateMachine<(pac::PIO2, SM2), Running>,
    _rx: PioRx<(pac::PIO2, SM2)>,
    pub port: MuxPort,
}

impl Mux {
    /// Start PIO2. RD (and WRPROT with `two_pins`, the Apple 3.5 Drive's SENSE) must be in
    /// `FunctionPio2`, [`PIN_RD_STREAM`] in `FunctionPio0`. RD is high-impedance until /ENBL
    /// goes low, and every register reads 0 until core 1 shows its status.
    pub fn new(pio2: pac::PIO2, resets: &mut pac::RESETS, two_pins: bool) -> Self {
        const { assert!(PIN_CA2 == PIN_SEL + 3 && PIN_WRPROT == PIN_RD + 1 && PIN_LSTRB == 8) };
        // `mov osr, pins` from SEL up, `out pc, 4`: entry r. `jmp pin` reads the stream.
        let one = pio::pio_asm!(
            ".origin 16",
            ".side_set 1 opt",
            "sense:",
            "    mov osr, pins",
            "    out pc, 4",
            "public data:",
            "    jmp pin high",
            "    jmp sense     side 0",
            "high:",
            "    jmp sense     side 1",
        );
        let two = pio::pio_asm!(
            ".origin 16",
            ".side_set 2 opt",
            "sense:",
            "    mov osr, pins",
            "    out pc, 4",
            "public data:",
            "    jmp pin high",
            "    jmp sense     side 0",
            "high:",
            "    jmp sense     side 3",
        );
        // /ENBL (the jmp pin) high: high-impedance.
        let oe_one = pio::pio_asm!(
            ".origin 21",
            "top:",
            "    jmp pin off",
            "    set pindirs, 1",
            "    jmp top",
            "off:",
            "    set pindirs, 0",
        );
        let oe_two = pio::pio_asm!(
            ".origin 21",
            "top:",
            "    jmp pin off",
            "    set pindirs, 3",
            "    jmp top",
            "off:",
            "    set pindirs, 0",
        );
        // GP2 up to GP15 (SEL, CA0–CA2, LSTRB, /ENBL, … /DRIVE2) once LSTRB has been high for
        // ~60 ns (a glitch is dropped).
        let strobe = pio::pio_asm!(
            ".origin 25",
            "top:",
            "    wait 0 gpio 8",
            "    wait 1 gpio 8 [7]",
            "    jmp pin take",
            "    jmp top",
            "take:",
            "    in pins, 14",
            "    push noblock",
        );
        let (mux, data, oe) = if two_pins {
            (two.program, two.public_defines.data, oe_two.program)
        } else {
            (one.program, one.public_defines.data, oe_one.program)
        };
        let side_set = mux.side_set;
        let jmp = |address, side| {
            pio::Instruction {
                operands: pio::InstructionOperands::JMP { condition: pio::JmpCondition::Always, address },
                delay: 0,
                side_set: side,
            }
            .encode(side_set)
        };
        let all = if two_pins { 3 } else { 1 };
        let data_at = MUX_LOOP + data as u8;
        let port = MuxPort {
            instr: unsafe { (*pac::PIO2::ptr()).instr_mem(0).as_ptr() },
            fstat: unsafe { (*pac::PIO2::ptr()).fstat().as_ptr() },
            rxf: unsafe { (*pac::PIO2::ptr()).rxf(2).as_ptr() },
            low: u32::from(jmp(MUX_LOOP, Some(0))),
            high: u32::from(jmp(MUX_LOOP, Some(all))),
            data: u32::from(jmp(data_at, None)),
        };

        let (mut pio, sm0, sm1, sm2, _) = pio2.split(resets);
        // The table, which no program claims: every register 0, R8/R9 the stream.
        for k in 0..16 {
            let word = if k & 0b1110 == 8 { port.data } else { port.low };
            unsafe { write_volatile(port.instr.add(k), word) };
        }
        let mux = pio.install(&mux).unwrap();
        let oe = pio.install(&oe).unwrap();
        let strobe = pio.install(&strobe.program).unwrap();
        debug_assert!(mux.offset() == MUX_LOOP);

        let (sm0, _, _) = PIOBuilder::from_installed_program(mux)
            .side_set_pin_base(PIN_RD)
            .in_pin_base(PIN_SEL)
            .jmp_pin(PIN_RD_STREAM)
            .out_shift_direction(ShiftDirection::Right)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);
        let (mut sm1, _, _) = PIOBuilder::from_installed_program(oe)
            .set_pins(PIN_RD, if two_pins { 2 } else { 1 })
            .jmp_pin(PIN_ENBL)
            .clock_divisor_fixed_point(1, 0)
            .build(sm1);
        // Inputs until SM1 first runs.
        sm1.set_pindirs([(PIN_RD, PinDir::Input)]);
        if two_pins {
            sm1.set_pindirs([(PIN_WRPROT, PinDir::Input)]);
        }
        let (sm2, rx, _) = PIOBuilder::from_installed_program(strobe)
            .in_pin_base(PIN_SEL)
            .jmp_pin(PIN_LSTRB)
            .in_shift_direction(ShiftDirection::Left)
            .buffers(Buffers::OnlyRx)
            .clock_divisor_fixed_point(1, 0)
            .build(sm2);
        Self { _sm0: sm0.start(), _sm1: sm1.start(), _sm2: sm2.start(), _rx: rx, port }
    }
}

/// [`Mux`]'s registers, for core 1.
#[derive(Clone, Copy)]
pub struct MuxPort {
    /// PIO2's INSTR_MEM0: entry k of the table is instruction k.
    instr: *mut u32,
    fstat: *const u32,
    rxf: *const u32,
    /// The entries: register 0, register 1, the stream.
    low: u32,
    high: u32,
    data: u32,
}

unsafe impl Send for MuxPort {}

impl MuxPort {
    /// Show `want` (bit k: register k's level; [`MUX_DATA`]: R8/R9 play the track) where the
    /// table last showed `shown`, rewriting only the entries that change.
    #[inline(always)]
    fn show(&self, shown: &mut u32, want: u32) {
        let mut diff = *shown ^ want;
        if diff == 0 {
            return;
        }
        *shown = want;
        if diff & MUX_DATA != 0 {
            diff |= 0x300;
        }
        let mut diff = diff & 0xFFFF;
        while diff != 0 {
            let k = diff.trailing_zeros();
            diff &= diff - 1;
            let word = if want & MUX_DATA != 0 && k & 0b1110 == 8 {
                self.data
            } else if want >> k & 1 != 0 {
                self.high
            } else {
                self.low
            };
            unsafe { write_volatile(self.instr.add(k as usize), word) };
        }
    }

    /// The lines at the next LSTRB rising edge not yet taken: GP2 up as bits 0–13 (the
    /// register's address in bits 0–3).
    #[inline(always)]
    fn strobe(&self) -> Option<u32> {
        // FSTAT's RXEMPTY for SM2.
        if unsafe { read_volatile(self.fstat) } & 1 << (8 + 2) != 0 {
            None
        } else {
            Some(unsafe { read_volatile(self.rxf) })
        }
    }
}

/// PIO clock divisor as 16.8 fixed point, rounded.
fn clock_divisor(sys_hz: u32, pio_hz: u32) -> (u16, u8) {
    let div = (u64::from(sys_hz) * 256 + u64::from(pio_hz) / 2) / u64::from(pio_hz);
    ((div >> 8) as u16, div as u8)
}

/// The SM_CLKDIV value that runs a state machine at `pio_hz`, for [`CLKDIV`] and [`CAP_CLKDIV`].
pub fn clkdiv_word(sys_hz: u32, pio_hz: u32) -> u32 {
    let (int, frac) = clock_divisor(sys_hz, pio_hz);
    u32::from(int) << 16 | u32::from(frac) << 8
}

/// Raw DMA/PIO registers for restarting the stream, so core 1 can do it from RAM without
/// calling into flash.
#[derive(Clone, Copy)]
pub struct Player {
    abort: *mut u32,
    data_read: *mut u32,
    data_write: *mut u32,
    data_count: *mut u32,
    data_ctrl: *mut u32,
    data_ctrl_trig: *mut u32,
    data_cfg: u32,
    data_restart: u32,
    ctrl_read: *mut u32,
    ctrl_write: *mut u32,
    ctrl_count: *mut u32,
    ctrl_ctrl: *mut u32,
    ctrl_cfg: u32,
    shiftctrl: *mut u32,
    flevel: *mut u32,
    /// SM0's SM_CLKDIV, for changing the cell rate from core 1 (dual mode).
    clkdiv: *mut u32,
    txf: u32,
    // Apple II mode ([`Player::play_a2`]): the count channel, PIO0's CTRL, SM0's INSTR and
    // EXECCTRL, and the instructions it executes.
    count_read: *mut u32,
    count_write: *mut u32,
    count_count: *mut u32,
    count_ctrl: *mut u32,
    count_cfg: u32,
    pio_ctrl: *mut u32,
    sm_instr: *mut u32,
    sm_execctrl: *mut u32,
    a2_exec: [u16; 4],
    // Dual mode ([`Player::use_program`]): SM0's PINCTRL; per program (drive A's, drive B's) its
    // SHIFTCTRL, EXECCTRL, PINCTRL and the data channel's CTRL; and the jump to drive A's.
    pinctrl: *mut u32,
    programs: [[u32; 4]; 2],
    sony_jmp: u16,
}

// Safety: only core 1 uses the registers once it has started.
unsafe impl Send for Player {}

/// SM_SHIFTCTRL.FJOIN_TX; changing any FJOIN bit flushes the state machine's FIFOs.
const FJOIN_TX: u32 = 1 << 30;
/// SM_EXECCTRL.EXEC_STALLED: an instruction written to SM_INSTR hasn't run yet.
const EXEC_STALLED: u32 = 1 << 31;

impl Player {
    /// Restart the stream on a new track buffer at word `start`, and loop the whole buffer
    /// after that until the next call.
    ///
    /// # Safety
    /// `words[..len]` must stay valid and unmodified until `play` is called with another buffer.
    #[inline(always)]
    unsafe fn play(&self, words: u32, len: u32, start: u32) {
        unsafe {
            // Disable before aborting so an abort can't fire the chain.
            write_volatile(self.data_ctrl, 0);
            write_volatile(self.ctrl_ctrl, 0);
            write_volatile(self.abort, 1 << DMA_DATA | 1 << DMA_CTRL);
            while read_volatile(self.abort) != 0 {}

            // Drop what the old track left in the FIFO, so the new one starts at once.
            let shift = read_volatile(self.shiftctrl);
            write_volatile(self.shiftctrl, shift ^ FJOIN_TX);
            write_volatile(self.shiftctrl, shift);

            STREAM_PTR.store(words, Relaxed);
            write_volatile(self.ctrl_read, STREAM_PTR.as_ptr() as u32);
            write_volatile(self.ctrl_write, self.data_restart);
            write_volatile(self.ctrl_count, 1);
            write_volatile(self.ctrl_ctrl, self.ctrl_cfg);

            // The first pass runs from `start`. TRANS_COUNT is copied into the live counter
            // when the channel is triggered; writing it again afterward only changes that
            // RELOAD value, so every later pass (restarted by the control channel) is whole.
            write_volatile(self.data_read, words + 4 * start);
            write_volatile(self.data_write, self.txf);
            write_volatile(self.data_count, len - start);
            write_volatile(self.data_ctrl_trig, self.data_cfg);
            write_volatile(self.data_count, len);
        }
    }

    /// Stop the stream and drop what's queued (HD20 + floppy mode: SM0 is to send an HD20 reply a
    /// byte at a time, [`DcdPort::bytewise`]). [`play`](Self::play) starts it again.
    #[inline(always)]
    fn stop(&self) {
        unsafe {
            write_volatile(self.data_ctrl, 0);
            write_volatile(self.ctrl_ctrl, 0);
            write_volatile(self.abort, 1 << DMA_DATA | 1 << DMA_CTRL);
            while read_volatile(self.abort) != 0 {}
            let shift = read_volatile(self.shiftctrl);
            write_volatile(self.shiftctrl, shift ^ FJOIN_TX);
            write_volatile(self.shiftctrl, shift);
        }
    }

    /// Run the stream at another cell rate (a [`clkdiv_word`]). The next [`play`](Self::play)
    /// flushes what was queued at the old one.
    #[inline(always)]
    fn set_clkdiv(&self, word: u32) {
        unsafe { write_volatile(self.clkdiv, word) };
    }

    /// Apple II mode (`src/disk2.pio`): play `bits` cells a revolution from `words` (`len` words:
    /// `bits / 32 + 1`), starting now at cell `cell`. The state machine is restarted, so the first
    /// cell begins at once rather than after the word playing; it's primed with the cells left in
    /// this revolution and the cell's word, `cell % 32` of whose cells are skipped.
    ///
    /// # Safety
    /// `words[..len]` must stay valid and unmodified until the next call.
    #[inline(always)]
    unsafe fn play_a2(&self, words: u32, len: u32, bits: u32, cell: u32) {
        let cell = if cell < bits { cell } else { 0 };
        unsafe {
            let ctrl = read_volatile(self.pio_ctrl) & 0xF;
            write_volatile(self.pio_ctrl, ctrl & !1);
            write_volatile(self.data_ctrl, 0);
            write_volatile(self.ctrl_ctrl, 0);
            write_volatile(self.count_ctrl, 0);
            write_volatile(self.abort, 1 << DMA_DATA | 1 << DMA_CTRL | 1 << DMA_COUNT);
            while read_volatile(self.abort) != 0 {}
            let shift = read_volatile(self.shiftctrl);
            write_volatile(self.shiftctrl, shift ^ FJOIN_TX);
            write_volatile(self.shiftctrl, shift);
            write_volatile(self.pio_ctrl, ctrl & !1 | 1 << 4); // SM_RESTART

            // The cells left in this revolution (less one), then the DMA: the rest of its words,
            // the next revolution's count, and over again.
            write_volatile(self.txf as *mut u32, bits - cell - 1);
            STREAM_PTR.store(words, Relaxed);
            STREAM_COUNT.store(bits - 1, Relaxed);
            write_volatile(self.ctrl_read, STREAM_PTR.as_ptr() as u32);
            write_volatile(self.ctrl_write, self.data_restart);
            write_volatile(self.ctrl_count, 1);
            write_volatile(self.ctrl_ctrl, self.ctrl_cfg);
            write_volatile(self.count_read, STREAM_COUNT.as_ptr() as u32);
            write_volatile(self.count_write, self.txf);
            write_volatile(self.count_count, 1);
            write_volatile(self.count_ctrl, self.count_cfg);
            let first = cell / 32;
            write_volatile(self.data_read, words + 4 * first);
            write_volatile(self.data_write, self.txf);
            write_volatile(self.data_count, len - first);
            write_volatile(self.data_ctrl_trig, self.data_cfg);
            write_volatile(self.data_count, len);

            // Prime it: the count into Y, the cell's word into the OSR, skip to the cell, and on
            // from `cell` (each run at once, the state machine stopped).
            let mut spin = 0;
            while read_volatile(self.flevel) & 0xF < 2 && spin < 1_000 {
                spin += 1;
            }
            let skip = cell % 32;
            let exec = [self.a2_exec[0], self.a2_exec[1], self.a2_exec[0], self.a2_exec[2] | skip as u16, self.a2_exec[3]];
            for (k, &ins) in exec.iter().enumerate() {
                if k == 3 && skip == 0 {
                    continue;
                }
                write_volatile(self.sm_instr, u32::from(ins));
                let mut spin = 0;
                while read_volatile(self.sm_execctrl) & EXEC_STALLED != 0 && spin < 1_000 {
                    spin += 1;
                }
            }
            write_volatile(self.pio_ctrl, ctrl | 1);
        }
    }

    /// Dual mode ([`Pulses::Dual`]): run SM0 on drive B's program (`src/disk2.pio`, played with
    /// [`play_a2`](Self::play_a2), which starts it) or on drive A's (`b` false: played with
    /// [`play`](Self::play), then [`resume`](Self::resume)). The stream stops meanwhile.
    #[inline(always)]
    fn use_program(&mut self, b: bool) {
        let [shift, exec, pins, data] = self.programs[b as usize];
        unsafe {
            let ctrl = read_volatile(self.pio_ctrl) & 0xF;
            write_volatile(self.pio_ctrl, ctrl & !1);
            // The count channel too: left pending, it would restart the data channel later.
            write_volatile(self.data_ctrl, 0);
            write_volatile(self.ctrl_ctrl, 0);
            write_volatile(self.count_ctrl, 0);
            write_volatile(self.abort, 1 << DMA_DATA | 1 << DMA_CTRL | 1 << DMA_COUNT);
            while read_volatile(self.abort) != 0 {}
            write_volatile(self.shiftctrl, shift);
            write_volatile(self.sm_execctrl, exec);
            write_volatile(self.pinctrl, pins);
            if !b {
                // From the top of drive A's program, its OSR empty, so its first `out` pulls.
                write_volatile(self.pio_ctrl, ctrl & !1 | 1 << 4); // SM_RESTART
                write_volatile(self.sm_instr, u32::from(self.sony_jmp));
            }
        }
        self.data_cfg = data;
    }

    /// Start SM0 again, once [`play`](Self::play) has queued drive A's words after
    /// [`use_program`](Self::use_program).
    #[inline(always)]
    fn resume(&self) {
        unsafe { write_volatile(self.pio_ctrl, read_volatile(self.pio_ctrl) & 0xF | 1) };
    }

    /// Play `side` of buffer set `set`, as published by core 0, from word `start`.
    #[inline(always)]
    fn play_cylinder(&self, set: usize, side: u8, start: u32) {
        let side = side as usize & 1;
        let words = CYL_PTR[set][side].load(Relaxed);
        let len = CYL_LEN[set][side].load(Relaxed);
        let start = if start < len { start } else { 0 };
        // Safety: core 0 doesn't touch a set while it is in PLAYING.
        unsafe { self.play(words, len, start) };
    }

    /// The word of `side` of `set` the PIO will take next: the DMA read position less what is
    /// still queued in the TX FIFO. The word being shifted out is the one before it.
    #[inline(always)]
    fn next_word(&self, set: usize, side: u8) -> u32 {
        let side = side as usize & 1;
        let base = CYL_PTR[set][side].load(Relaxed);
        let len = CYL_LEN[set][side].load(Relaxed);
        let read = unsafe { read_volatile(self.data_read) };
        let queued = unsafe { read_volatile(self.flevel) } & 0xF; // TX0 level
        let at = read.wrapping_sub(base) / 4;
        if len == 0 || at > len { 0 } else { (at + len - queued) % len }
    }

    /// Where `to_side` of `to_set` should start in place of the playing `from_side` of
    /// `from_set`: word 0, or if `keep` ([`KEEP_ROTATION`]) the same fraction of a revolution as
    /// the word about to play, so the disk seems to keep turning. The fraction is of the tracks
    /// as imaged ([`CYL_ROT`]), not of the rotated buffers.
    #[inline(always)]
    fn spin_start(&self, keep: bool, from_set: usize, from_side: u8, to_set: usize, to_side: u8) -> u32 {
        if !keep {
            return 0;
        }
        let (from, to) = (from_side as usize & 1, to_side as usize & 1);
        let from_len = CYL_LEN[from_set][from].load(Relaxed);
        let to_len = CYL_LEN[to_set][to].load(Relaxed);
        // The word and the rotations are each below their buffer's length, so one subtraction
        // wraps them (a `%` here cost the hot path a register). A bad start plays from word 0.
        let mut at = self.next_word(from_set, from_side) + CYL_ROT[from_set][from].load(Relaxed);
        if at >= from_len {
            at -= from_len;
        }
        // Both lengths are at most MAX_TRACK_WORDS (6,400), so the product fits.
        let start = (at * to_len / from_len.max(1) + to_len).wrapping_sub(CYL_ROT[to_set][to].load(Relaxed));
        if start >= to_len { start.wrapping_sub(to_len) } else { start }
    }
}

/// Owns the PIO1 state machines that capture writes and DMA channel 2. Keep it alive; core 1
/// gates the capture through [`Writer::capture`].
pub struct Writer {
    _gcr: StateMachine<(pac::PIO1, SM0), Stopped>,
    _mfm: StateMachine<(pac::PIO1, SM1), Stopped>,
    rxf: [u32; 2],
    dreq: [u8; 2],
    pub capture: Capture,
}

impl Writer {
    /// Load both separators from `capture.pio` on PIO1 (SM0 GCR, SM1 MFM), reading `PIN_WR`.
    /// Neither runs until core 1 opens a gate.
    pub fn new(pio1: pac::PIO1, resets: &mut pac::RESETS, sys_hz: u32) -> Self {
        let gcr_program = pio::pio_file!("src/capture.pio", select_program("gcr_write"));
        let mfm_program = pio::pio_file!("src/capture.pio", select_program("mfm_write"));
        let (mut pio, sm0, sm1, _, _) = pio1.split(resets);
        let gcr_installed = pio.install(&gcr_program.program).unwrap();
        let mfm_installed = pio.install(&mfm_program.program).unwrap();
        // `jmp <start>` with no condition or delay is just the address.
        let entry = [u32::from(gcr_installed.offset()), u32::from(mfm_installed.offset())];

        let (int, frac) = clock_divisor(sys_hz, GCR_CAPTURE_HZ);
        let (gcr, gcr_rx, _) = PIOBuilder::from_installed_program(gcr_installed)
            .jmp_pin(PIN_WR)
            .in_pin_base(PIN_WR)
            .in_shift_direction(ShiftDirection::Left)
            .autopush(true)
            .push_threshold(0) // 0 means 32
            .buffers(Buffers::OnlyRx)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);
        let (int, frac) = clock_divisor(sys_hz, MFM_CAPTURE_HZ);
        let (mfm, mfm_rx, _) = PIOBuilder::from_installed_program(mfm_installed)
            .jmp_pin(PIN_WR)
            .in_pin_base(PIN_WR)
            .in_shift_direction(ShiftDirection::Left)
            .autopush(true)
            .push_threshold(0)
            .buffers(Buffers::OnlyRx)
            .clock_divisor_fixed_point(int, frac)
            .build(sm1);

        let pio1 = unsafe { &*pac::PIO1::ptr() };
        let dma = unsafe { &*pac::DMA::ptr() };
        let ch = dma.ch(DMA_CAPTURE);
        let rxf = [gcr_rx.fifo_address() as u32, mfm_rx.fifo_address() as u32];
        let dreq = [gcr_rx.dreq_value(), mfm_rx.dreq_value()];
        // For `Capture::retarget`: the channel's CTRL for each separator (as `start` sets it), and
        // an endless transfer count. Written while it's disabled, so nothing starts.
        let cfg = dreq.map(|dreq| {
            ch.ch_al1_ctrl().write(|w| unsafe {
                w.data_size().size_word();
                w.incr_read().clear_bit();
                w.incr_write().set_bit();
                w.ring_sel().set_bit();
                w.ring_size().bits(15);
                w.treq_sel().bits(dreq);
                w.chain_to().bits(DMA_CAPTURE as u8);
                w.irq_quiet().set_bit()
            });
            ch.ch_al1_ctrl().read().bits() | 1
        });
        ch.ch_al1_ctrl().write(|w| unsafe { w.bits(0) });
        ch.ch_trans_count().write(|w| unsafe { w.mode().endless().count().bits(1) });
        let endless = ch.ch_trans_count().read().bits();
        let capture = Capture {
            ctrl: pio1.ctrl().as_ptr(),
            shiftctrl: [pio1.sm(0).sm_shiftctrl().as_ptr(), pio1.sm(1).sm_shiftctrl().as_ptr()],
            instr: [pio1.sm(0).sm_instr().as_ptr(), pio1.sm(1).sm_instr().as_ptr()],
            entry,
            dma_write: ch.ch_write_addr().as_ptr(),
            gcr_clkdiv: pio1.sm(0).sm_clkdiv().as_ptr(),
            dma_abort: dma.chan_abort().as_ptr(),
            dma_read: ch.ch_read_addr().as_ptr(),
            dma_count: ch.ch_trans_count().as_ptr(),
            dma_ctrl: ch.ch_al1_ctrl().as_ptr(),
            dma_ctrl_trig: ch.ch_ctrl_trig().as_ptr(),
            rxf,
            cfg,
            endless,
        };
        Self { _gcr: gcr, _mfm: mfm, rxf, dreq, capture }
    }

    /// Run the GCR separator at `hz` (32 cycles per cell): [`A2_CAPTURE_HZ`] for an Apple II.
    /// Call while no gate is open.
    pub fn set_gcr_rate(&self, sys_hz: u32, hz: u32) {
        let (int, frac) = clock_divisor(sys_hz, hz);
        let pio1 = unsafe { &*pac::PIO1::ptr() };
        pio1.sm(0).sm_clkdiv().write(|w| unsafe { w.int().bits(int).frac().bits(frac) });
    }

    /// Push the GCR separator's cells every `bits` (8 for SmartPort, so a packet's last byte reaches
    /// core 1 within 8 cells; 32, the default, fills whole words). With 8, each ring word holds 8
    /// cells in its low byte. Call while no gate is open.
    pub fn set_push_bits(&self, bits: u32) {
        let pio1 = unsafe { &*pac::PIO1::ptr() };
        pio1.sm(0).sm_shiftctrl().modify(|_, w| unsafe { w.push_thresh().bits((bits % 32) as u8) });
    }

    /// Start DMA channel 2 streaming the GCR or MFM separator's output into [`RING`], forever.
    /// Call once the image type is known, before the disk is inserted.
    /// Call again to switch separators while no gate is open.
    pub fn start(&self, mfm: bool) {
        let sm = mfm as usize;
        let dma = unsafe { &*pac::DMA::ptr() };
        let ch = dma.ch(DMA_CAPTURE);
        // Stop the previous stream, if any: disable first so the abort can't chain.
        ch.ch_al1_ctrl().write(|w| unsafe { w.bits(0) });
        dma.chan_abort().write(|w| unsafe { w.bits(1 << DMA_CAPTURE) });
        while dma.chan_abort().read().bits() != 0 {}
        ch.ch_read_addr().write(|w| unsafe { w.bits(self.rxf[sm]) });
        ch.ch_write_addr().write(|w| unsafe { w.bits(RING.base()) });
        ch.ch_trans_count().write(|w| unsafe { w.mode().endless().count().bits(1) });
        ch.ch_ctrl_trig().write(|w| unsafe {
            w.data_size().size_word();
            w.incr_read().clear_bit();
            w.incr_write().set_bit();
            w.ring_sel().set_bit(); // wrap the write address
            w.ring_size().bits(15); // 2^15 bytes = RING_WORDS words
            w.treq_sel().bits(self.dreq[sm]);
            w.chain_to().bits(DMA_CAPTURE as u8); // chaining to itself = no chain
            w.irq_quiet().set_bit();
            w.en().set_bit()
        });
    }
}

/// Raw PIO1/DMA registers for gating the capture from core 1.
#[derive(Clone, Copy)]
pub struct Capture {
    ctrl: *mut u32,
    shiftctrl: [*mut u32; 2],
    instr: [*mut u32; 2],
    entry: [u32; 2],
    dma_write: *mut u32,
    /// The GCR separator's SM_CLKDIV, for each drive's write cell in dual mode.
    gcr_clkdiv: *mut u32,
    // HD20 + floppy mode ([`Capture::retarget`]): DMA channel 2's registers, each separator's RX
    // FIFO and the channel's CTRL for it, and an endless TRANS_COUNT.
    dma_abort: *mut u32,
    dma_read: *mut u32,
    dma_count: *mut u32,
    dma_ctrl: *mut u32,
    dma_ctrl_trig: *mut u32,
    rxf: [u32; 2],
    cfg: [u32; 2],
    endless: u32,
}

// Safety: only core 1 uses the registers once it has started.
unsafe impl Send for Capture {}

/// SM_SHIFTCTRL.FJOIN_RX, toggled to flush the RX FIFO.
const FJOIN_RX: u32 = 1 << 31;

impl Capture {
    /// Ring word DMA writes next.
    #[inline(always)]
    fn position(&self) -> u32 {
        let at = unsafe { read_volatile(self.dma_write) };
        (at.wrapping_sub(RING.base()) / 4) % RING_WORDS as u32
    }

    /// Start state machine `sm` (0 GCR, 1 MFM) from the top of its program, empty.
    #[inline(always)]
    fn open(&self, sm: usize) {
        unsafe {
            let shift = read_volatile(self.shiftctrl[sm]);
            write_volatile(self.shiftctrl[sm], shift ^ FJOIN_RX);
            write_volatile(self.shiftctrl[sm], shift);
            let ctrl = read_volatile(self.ctrl) & 0xF;
            write_volatile(self.ctrl, ctrl | 1 << (4 + sm)); // SM_RESTART: clear ISR and counters
            write_volatile(self.instr[sm], self.entry[sm]);
            write_volatile(self.ctrl, ctrl | 1 << sm); // SM_ENABLE
        }
    }

    /// Run the GCR separator at another rate (a [`clkdiv_word`]). Only while it's stopped.
    #[inline(always)]
    fn set_gcr_clkdiv(&self, word: u32) {
        unsafe { write_volatile(self.gcr_clkdiv, word) };
    }

    /// Have DMA channel 2 take separator `sm`'s cells from now on, going on in the ring where it
    /// was ([`Writer::start`] from core 1, with no gate open): HD20 + floppy mode takes the
    /// HD20's commands with the GCR separator and an MFM floppy's writes with the MFM one.
    #[inline(always)]
    fn retarget(&self, sm: usize) {
        let sm = sm & 1;
        unsafe {
            write_volatile(self.dma_ctrl, 0);
            write_volatile(self.dma_abort, 1 << DMA_CAPTURE);
            while read_volatile(self.dma_abort) != 0 {}
            // The write address stays where the last capture left it.
            write_volatile(self.dma_read, self.rxf[sm]);
            write_volatile(self.dma_count, self.endless);
            write_volatile(self.dma_ctrl_trig, self.cfg[sm]);
        }
    }

    #[inline(always)]
    fn close(&self, sm: usize) {
        unsafe {
            let ctrl = read_volatile(self.ctrl) & 0xF;
            write_volatile(self.ctrl, ctrl & !(1 << sm));
        }
    }
}

// ---- Core 1: register file and commands ---------------------------------------------------

/// GPIO_CTRL override fields.
const OUTOVER_SHIFT: u32 = 12;
const OEOVER_SHIFT: u32 = 14;
const OVER_MASK: u32 = 0b1111 << OUTOVER_SHIFT;
const OUT_NORMAL: u32 = 0;
const OUT_LOW: u32 = 2;
const OUT_HIGH: u32 = 3;
const OE_DISABLE: u32 = 2;
const OE_ENABLE: u32 = 3;

/// Half a TACH period in µs per zone: TACH runs at RPM Hz (60 pulses per revolution).
/// In RAM so core 1 never waits on flash.
#[unsafe(link_section = ".data.tach_half_us")]
static TACH_HALF_US: [u32; 5] = [
    500_000 / gcr::rpm(0),
    500_000 / gcr::rpm(16),
    500_000 / gcr::rpm(32),
    500_000 / gcr::rpm(48),
    500_000 / gcr::rpm(64),
];

/// Status bit R11: the drive is in MFM mode.
const MFM_MODE_BIT: u32 = 1 << 11;
/// [`KEEP_ROTATION`], carried in an unused status bit (registers are 0–15) so the side switch
/// doesn't load the atomic: that put an address in the hot path. Refreshed by housekeeping.
const KEEP_ROTATION_BIT: u32 = 1 << 16;

/// Status register levels as returned on RD, bit R = register R. R8/R9 (read data) are
/// handled by the mux. `superdrive` is false for an Apple 3.5 Drive (an 800K drive), whose eject
/// latch R6 is `switched`.
#[inline(always)]
fn status_bits(
    dir_out: bool,
    track: u8,
    motor_on: bool,
    r7: bool,
    mfm_mode: bool,
    disk_in: bool,
    ready: bool,
    hd: bool,
    writable: bool,
    superdrive: bool,
    switched: bool,
) -> u32 {
    let mut s = 0;
    s |= (dir_out as u32) << 0; //   R0 step direction: 1 = toward track 0
    s |= (!disk_in as u32) << 1; //  R1 /CSTIN
    s |= 1 << 2; //                  R2 /STEP: steps complete at once
    s |= (writable as u32) << 3; //  R3 /WRTPRT: 0 = write-protected
    s |= (!motor_on as u32) << 4; // R4 /MOTORON
    s |= ((track != 0) as u32) << 5; // R5 /TK0
    s |= (switched as u32) << 6; //  R6 eject / disk-switched latch (0 on a SuperDrive)
    s |= (r7 as u32) << 7; //        R7 TACH in GCR mode, the index pulse in MFM mode
    s |= (superdrive as u32) << 10; // R10 SuperDrive: always in the Mac modes
    if mfm_mode {
        s |= MFM_MODE_BIT; //        R11 drive mode, as last set by the 011 command
    }
    s |= 1 << 12; //                 R12 SIDES: double-sided drive
    s |= (!ready as u32) << 13; //   R13 /READY
    //                               R14 /DRVIN = 0: drive connected
    s |= (!hd as u32) << 15; //      R15 media: 0 = HD, 1 = DD (also "new interface" to older ROMs)
    s
}

/// Core 1 entry point. Never returns; runs from RAM with interrupts off, so commands and head
/// selection are followed promptly while core 0 is busy with flash, SD and display. RD itself is
/// [`Mux`]'s: core 1 keeps its table up to date.
///
/// The hot path is: sample the pins, follow head select, take a strobe [`Mux`] latched.
/// Everything that changes on a millisecond scale (TACH, write gates, new cylinders and disk
/// state from core 0, counters) is handled on every 16th pass.
#[unsafe(link_section = ".data.drive_loop")]
#[inline(never)]
pub fn run(player: Player, mux: MuxPort, capture: Capture) -> ! {
    sony::<false>(player, mux, capture)
}

/// Core 1 entry point in Apple II 3.5" mode: an Apple 3.5 Drive, the IIgs's and IIc+'s 800K
/// drive. It's [`run`]'s Sony drive, as an 800K drive (R10 = 0, no MFM mode) with:
/// - the register's level on WRPROT as well as RD: an Apple II senses the drive's registers on
///   WRPROT, where a Disk II has its write-protect switch;
/// - selection by /ENBL, /3.5DISK ([`PIN_EN35`]) low and /DRIVE2 ([`PIN_DRIVE2`]) high, so it
///   ignores the phase lines while the computer is using a 5.25" drive or SmartPort device;
/// - the disk-switched latch (R6), set when the disk goes and cleared by the reset-switch command
///   (001, CA2 = 1), which the IIgs firmware uses to notice a new disk;
/// - the motor stopping once /3.5DISK has been high for [`A35_MOTOR_OFF_US`].
#[unsafe(link_section = ".data.a35_loop")]
#[inline(never)]
pub fn run_a35(player: Player, mux: MuxPort, capture: Capture) -> ! {
    sony::<true>(player, mux, capture)
}

/// [`run`] (a SuperDrive for a Mac) and, with `A35`, [`run_a35`] (an Apple 3.5 Drive). Inlined
/// into each, so the Mac's loop compiles as if the Apple II's parts weren't there.
#[inline(always)]
fn sony<const A35: bool>(player: Player, mux: MuxPort, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let timer = unsafe { (*pac::TIMER0::ptr()).timerawl().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };
    // A35: WRPROT shows what RD does (both are PIO2's, so the same GPIO_CTRL words fit both).
    let wp_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_WRPROT as usize).gpio_ctrl().as_ptr() };

    // RD as PIO2 drives it, with its output enable from /ENBL; A35 holds both pins
    // high-impedance while /3.5DISK or /DRIVE2 say another drive is wanted.
    let pio = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let hiz = pio | OE_DISABLE << OEOVER_SHIFT;

    // Selected while `inp & sel_mask == sel_want`: /ENBL low, and for A35 /3.5DISK low and /DRIVE2
    // high.
    let (sel_mask, sel_want) = if A35 {
        (1u32 << PIN_ENBL | 1 << PIN_EN35 | 1 << PIN_DRIVE2, 1u32 << PIN_DRIVE2)
    } else {
        (1u32 << PIN_ENBL, 0)
    };
    let en35_mask = 1u32 << PIN_EN35;
    let a35_mask = 1u32 << PIN_EN35 | 1 << PIN_DRIVE2;
    let lstrb_mask = 1u32 << PIN_LSTRB;
    let wrreq_mask = 1u32 << PIN_WRREQ;

    let mut a35_on = false;
    unsafe { write_volatile(rd_ctrl, if A35 { hiz } else { pio }) };
    if A35 {
        unsafe { write_volatile(wp_ctrl, hiz) };
    }

    let mut track: u8 = 0;
    let mut dir_out = false;
    let mut motor_on = false;
    let mut tach = false;
    let mut tach_next = unsafe { read_volatile(timer) };
    let mut tach_edges = 0u32;
    let mut tach_flutter = 0u32;
    let mut disk_in = false;
    let mut ready = false;
    let mut playing = 0u32;
    let mut side = 0u8;
    let mut hd = false;
    let mut writable = false;
    // A35: the disk-switched latch, when the disk was last in, and when /3.5DISK was last low.
    let mut switched = false;
    let mut empty_at = tach_next;
    let mut en35_at = tach_next;
    // The drive mode (R11), R7 and KEEP_ROTATION_BIT are kept only in `status`, so they cost the
    // loop no registers.
    let mut status = status_bits(dir_out, track, motor_on, tach, false, disk_in, ready, hd, writable, !A35, false);
    // What R8/R9 show: the stream ([`MUX_DATA`]), or the index pulse while writing to an MFM
    // disk (bits 8 and 9). The table shows `shown`, made from it and `status`: as `Mux::new` left
    // it, every register 0 and R8/R9 the stream.
    let mut data = MUX_DATA;
    let mut shown = MUX_DATA;
    mux.show(&mut shown, status & 0xFFFF | data);
    let mut enabled_seen = 0u32;
    let mut pass = 0u32;

    // Write capture state.
    let mut writing = false;
    let mut closing = false;
    let mut capture_sm = 0usize;
    let mut opened_at = 0u32;
    let mut closed_at = 0u32;
    let mut cap_start = 0u32;
    let mut cap_pos = 0u32;
    let mut cap_info = 0u32;
    let mut total = 0u32;

    // The event log's view: selected, the register, changes this selection, disk in.
    #[cfg(feature = "diag")]
    let (mut log_en, mut log_r, mut log_changes, mut log_disk) = (false, 0u32, 0u32, false);

    loop {
        let inp = unsafe { read_volatile(gpio_in) };
        let r = (inp >> PIN_SEL) & 0xF;
        let enabled = inp & sel_mask == sel_want;

        // A35: RD and WRPROT are PIO2's while /3.5DISK and /DRIVE2 select this drive (PIO2
        // follows /ENBL itself).
        if A35 {
            let on = inp & a35_mask == 1 << PIN_DRIVE2;
            if on != a35_on {
                a35_on = on;
                let w = if on { pio } else { hiz };
                unsafe {
                    write_volatile(rd_ctrl, w);
                    write_volatile(wp_ctrl, w);
                }
            }
        }
        enabled_seen |= enabled as u32;

        // Head select: addressing R8 or R9 (with LSTRB low, so not a command) picks the side.
        if enabled && r & 0b1110 == 8 && inp & lstrb_mask == 0 {
            let sel = (r & 1) as u8;
            if sel != side {
                if cylinder_track(playing).is_some() {
                    let set = cylinder_set(playing);
                    let start = player.spin_start(status & KEEP_ROTATION_BIT != 0, set, side, set, sel);
                    player.play_cylinder(set, sel, start);
                }
                side = sel;
                SIDE.store(side, Relaxed);
            }
        }

        // A strobe PIO2 latched (on LSTRB's rising edge, with the lines then), while selected.
        if let Some(lines) = mux.strobe()
            && lines << PIN_SEL & sel_mask == sel_want
        {
            let r = lines & 0xF;
            let (latch, ca2) = (r as u8 & 0b111, r & 0b1000 != 0);
            match latch {
                0b000 => dir_out = ca2,
                0b010 if !ca2 => {
                    track = if dir_out {
                        track.saturating_sub(1)
                    } else {
                        (track + 1).min(crate::track::TRACKS - 1)
                    };
                    // The stream is now for the wrong track: not ready until core 0 catches up.
                    ready = false;
                    HEAD_TRACK.store(track, Relaxed);
                    STEPS.fetch_add(1, Relaxed);
                }
                0b100 => {
                    motor_on = !ca2;
                    MOTOR_ON.store(motor_on, Relaxed);
                }
                0b110 if ca2 => {
                    disk_in = false;
                    DISK_IN.store(false, Relaxed);
                    MAC_SAW_EMPTY.store(true, Relaxed);
                    EJECTS.fetch_add(1, Relaxed);
                    switched = true;
                }
                // 011 GCR/MFM mode (CA2 = 0: MFM). Only R11 (and R7) follow it; the stream
                // format is fixed by the image. The ROM's PowerUp reads R11 before every read
                // and write, and if it doesn't match the disk, it sets the mode and waits for
                // the motor again: 75 ms, plus 600 ms before writes. An 800K drive
                // has no MFM mode.
                0b011 if !A35 => status = status & !MFM_MODE_BIT | if ca2 { 0 } else { MFM_MODE_BIT },
                // 001 reset the eject latch: only the Apple 3.5 Drive's disk-switched latch.
                0b001 if A35 && ca2 => switched = false,
                _ => {}
            }
            ready &= motor_on;
            let r7 = status >> 7 & 1 != 0;
            let mfm_mode = status & MFM_MODE_BIT != 0;
            let keep = status & KEEP_ROTATION_BIT;
            status = status_bits(dir_out, track, motor_on, r7, mfm_mode, disk_in, ready, hd, writable, !A35, A35 && switched) | keep;
            mux.show(&mut shown, status & 0xFFFF | data);
            LAST_CMD.store((latch << 1) | ca2 as u8, Relaxed);
            STROBES.fetch_add(1, Relaxed);
            #[cfg(feature = "diag")]
            {
                let cmd = ((latch << 1) | ca2 as u8) as u32 & 0xF;
                CMD_COUNT[cmd as usize].fetch_add(1, Relaxed);
                log(unsafe { read_volatile(timer) }, LOG_CMD, cmd | (track as u32) << 8);
            }
        }

        pass = pass.wrapping_add(1);
        if pass & 0xF != 0 {
            continue;
        }

        // ---- Housekeeping, every 16th pass ----
        let now = unsafe { read_volatile(timer) };
        if now.wrapping_sub(tach_next) as i32 >= 0 {
            tach = !tach;
            // Shorten the period by 0–4 µs, a different amount each revolution (120 edges): the 64K
            // ROM's MakeSpdTbl crashes if two successive TACH measurements come out identical.
            tach_edges += 1;
            if tach_edges >= 120 {
                tach_edges = 0;
                tach_flutter = if tach_flutter >= 4 { 0 } else { tach_flutter + 1 };
            }
            let zone = (gcr::zone(track) as usize).min(TACH_HALF_US.len() - 1);
            tach_next = now.wrapping_add(TACH_HALF_US[zone] - tach_flutter);
        }

        // Write gate: /WRREQ low while selected.
        let gate = enabled && inp & wrreq_mask == 0;
        if closing && (gate || now.wrapping_sub(closed_at) >= CAPTURE_TAIL_US) {
            // Finish the last capture: stop the separator and queue what it wrote.
            closing = false;
            capture.close(capture_sm);
            let words = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            let head = CAP_HEAD.load(Relaxed);
            if head.wrapping_sub(CAP_TAIL.load(Acquire)) < CAPTURE_QUEUE as u32 {
                let k = head as usize % CAPTURE_QUEUE;
                CAP_START[k].store(cap_start, Relaxed);
                CAP_WORDS[k].store(words, Relaxed);
                CAP_POS[k].store(cap_pos, Relaxed);
                CAP_INFO[k].store(cap_info, Relaxed);
                CAP_SEQ[k].store(total, Relaxed);
                CAP_HEAD.store(head.wrapping_add(1), Release);
            } else {
                CAP_DROPPED.fetch_add(1, Relaxed);
            }
            total = total.wrapping_add(words);
            CAP_TOTAL.store(total, Release);
            WRITING.store(false, Release);
        }
        if gate && !writing && !closing {
            if writable && disk_in && cylinder_track(playing).is_some() {
                writing = true;
                capture_sm = MFM.load(Relaxed) as usize;
                capture.open(capture_sm);
                cap_start = capture.position();
                let set = cylinder_set(playing);
                // The cell under the head: just past the start of the word being shifted out.
                let len = CYL_LEN[set][side as usize].load(Relaxed);
                let next = player.next_word(set, side);
                cap_pos = ((next + len - 1) % len.max(1)) * 32 + 16;
                cap_info = playing | if side == 1 { CAP_SIDE } else { 0 };
                opened_at = now;
                WRITING.store(true, Relaxed);
            }
        } else if !gate && writing {
            writing = false;
            closing = true;
            closed_at = now;
            if now.wrapping_sub(opened_at) > FORMAT_GATE_US {
                // A format: the formatter reads the next header and wants sector 0 (GCR).
                cap_info |= CAP_FORMAT;
                player.play_cylinder(cylinder_set(playing), side, 0);
            }
        }
        if writing {
            let so_far = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            CAP_TOTAL.store(total.wrapping_add(so_far), Release);
        }

        let published = PUBLISHED.load(Acquire);
        if published != playing {
            // The same cylinder republished after a write keeps its rotational position. Another
            // cylinder starts from word 0, in a gap, or where the old one was (KEEP_ROTATION).
            let start = if cylinder_track(published).is_none() || cylinder_track(playing).is_none() {
                0
            } else if cylinder_track(published) == cylinder_track(playing) {
                player.next_word(cylinder_set(playing), side)
            } else {
                let keep = status & KEEP_ROTATION_BIT != 0;
                player.spin_start(keep, cylinder_set(playing), side, cylinder_set(published), side)
            };
            playing = published;
            player.play_cylinder(cylinder_set(playing), side, start);
            PLAYING.store(playing, Release);
        }
        if enabled_seen != 0 {
            SELECTED.store(true, Relaxed);
            enabled_seen = 0;
            // The Mac is sensing R1 and RD says no disk (from `disk_in` as of the last
            // housekeeping, not yet reloaded). Sampled only here, with a fresh read so the hot
            // path's values needn't stay live: the ROM holds the address for several µs, and
            // polls every 0.5 s while it thinks the drive is empty.
            let pins = unsafe { read_volatile(gpio_in) };
            if pins & sel_mask == sel_want && (pins >> PIN_SEL) & 0xF == 1 && !disk_in {
                MAC_SAW_EMPTY.store(true, Relaxed);
            }
        }
        if A35 {
            let now_in = DISK_IN.load(Relaxed);
            // The disk went (from the knob; EJECT sets it itself).
            switched |= disk_in && !now_in;
            // ProDOS 8 senses R1 only when it next reads the drive, which may be never; the latch
            // will tell it the disk changed. So after a while empty, another disk may go in.
            if now_in {
                empty_at = now;
            } else if now.wrapping_sub(empty_at) >= A35_EMPTY_US {
                MAC_SAW_EMPTY.store(true, Relaxed);
            }
            // The computer has turned to its 5.25" drives for a while: the motor stops.
            if inp & en35_mask == 0 {
                en35_at = now;
            } else if motor_on && now.wrapping_sub(en35_at) >= A35_MOTOR_OFF_US {
                motor_on = false;
                MOTOR_ON.store(false, Relaxed);
            }
        }
        disk_in = DISK_IN.load(Relaxed);
        hd = HD_MEDIA.load(Relaxed);
        writable = WRITABLE.load(Relaxed) && disk_in;
        ready = motor_on && disk_in && cylinder_track(playing) == Some(track);
        #[cfg(feature = "diag")]
        {
            if motor_on && enabled {
                REG_SAMPLES[r as usize].fetch_add(1, Relaxed);
                if !ready {
                    NOT_READY.fetch_add(1, Relaxed);
                }
            }
            // RD's level for the register, as the hot path shows it (R8/R9: the stream, 0 here).
            let level = if r & 0b1110 == 8 { 0 } else { status >> r & 1 };
            if enabled != log_en {
                log_en = enabled;
                if enabled {
                    log_changes = 0;
                    log(now, LOG_SEL, r | level << 4);
                } else {
                    log(now, LOG_DESEL, log_changes);
                }
                log_r = r;
            } else if enabled && r != log_r {
                log_r = r;
                log_changes += 1;
                if log_changes <= LOG_REG_MAX {
                    log(now, LOG_REG, r | level << 4);
                }
            }
            if disk_in != log_disk {
                log_disk = disk_in;
                log(now, LOG_DISK, disk_in as u32);
            }
        }
        // The index pulse: R7 in MFM mode, and R8/R9 while writing to an MFM disk (`capture_sm`
        // is the MFM separator then; a separate `mfm` local cost the hot path several
        // instructions in register spills).
        let mfm_mode = !A35 && status & MFM_MODE_BIT != 0;
        let index_writing = !A35 && writing && capture_sm == 1;
        let mut index_high = true;
        if mfm_mode || index_writing {
            // One revolution is the playing track's length in 1 µs half-cells.
            let len = CYL_LEN[cylinder_set(playing)][side as usize].load(Relaxed) * 32;
            let rev = if len == 0 { INDEX_PERIOD_US } else { len };
            index_high = now % rev >= INDEX_LOW_US;
        }
        let r7 = if mfm_mode { index_high } else { tach };
        status = status_bits(dir_out, track, motor_on, r7, mfm_mode, disk_in, ready, hd, writable, !A35, A35 && switched)
            | if KEEP_ROTATION.load(Relaxed) { KEEP_ROTATION_BIT } else { 0 };
        data = if !index_writing {
            MUX_DATA
        } else if index_high {
            0x300
        } else {
            0
        };
        mux.show(&mut shown, status & 0xFFFF | data);

        if pass & 0xFF == 0 {
            LOOPS.store(pass >> 8, Relaxed);
        }
    }
}

// ---- HD20 (DCD) ---------------------------------------------------------------------------
// In HD20 mode core 1 runs `run_dcd` instead of `run` (`dcd.rs` has the protocol). A command
// from the Mac is captured like a write: the GCR separator streams WR into RING, and core 1
// queues the capture in CAP_*. Core 0 decodes it and queues the reply (or, for a read, one reply
// per block) in DCD_TX; core 1 sends replies byte by byte through PIO0 SM0, with no DMA.

/// An image is loaded: answer as a DCD. Until then RD stays high-impedance, as if nothing were
/// connected.
pub static DCD_PRESENT: AtomicBool = AtomicBool::new(false);
/// Accept commands. Core 0 clears it when it swaps the image under a Mac that has used the old
/// one: the Mac would take the new volume for the one it has mounted. Core 1 sets it again when
/// the Mac probes for drives (state 5, R10), as it does when it starts up.
pub static DCD_LIVE: AtomicBool = AtomicBool::new(false);
/// Probes seen (state 5 entered while answering).
pub static DCD_PROBES: AtomicU32 = AtomicU32::new(0);
/// Bumped by core 1 whenever it drops the queued replies: the Mac sent another command, or
/// deselected the drive. Core 0 tags each reply with the value when its command arrived, and core
/// 1 sends only replies tagged with the current one.
pub static DCD_EPOCH: AtomicU32 = AtomicU32::new(0);
/// Replies queued at once: a read's next block is ready while the last one goes out.
pub const DCD_SLOTS: usize = 2;
/// Bytes per reply: up to 127 groups of 8.
pub const DCD_TX_BYTES: usize = 1_024;
/// Reply `k % DCD_SLOTS`: its length in bytes and its epoch.
pub static DCD_TX_LEN: [AtomicU32; DCD_SLOTS] = [const { AtomicU32::new(0) }; DCD_SLOTS];
pub static DCD_TX_EPOCH: [AtomicU32; DCD_SLOTS] = [const { AtomicU32::new(0) }; DCD_SLOTS];
/// Replies queued by core 0 / sent (or dropped) by core 1.
pub static DCD_TX_HEAD: AtomicU32 = AtomicU32::new(0);
pub static DCD_TX_TAIL: AtomicU32 = AtomicU32::new(0);

/// The reply buffers: encoded groups, without the leading `AA` (core 1 adds it).
pub struct DcdTx(UnsafeCell<[[u8; DCD_TX_BYTES]; DCD_SLOTS]>);

// Safety: core 0 writes slot `DCD_TX_HEAD % DCD_SLOTS` only while it isn't queued; core 1 reads
// queued slots only.
unsafe impl Sync for DcdTx {}

pub static DCD_TX: DcdTx = DcdTx(UnsafeCell::new([[0; DCD_TX_BYTES]; DCD_SLOTS]));

/// A reply slot is free for core 0.
pub fn dcd_slot_free() -> bool {
    DCD_TX_HEAD.load(Relaxed).wrapping_sub(DCD_TX_TAIL.load(Acquire)) < DCD_SLOTS as u32
}

/// Build a reply in the next free slot and queue it: `build` writes it and returns its length.
/// Call only when [`dcd_slot_free`]. `epoch` is [`DCD_EPOCH`] as of the command it answers.
pub fn dcd_queue(epoch: u32, build: impl FnOnce(&mut [u8; DCD_TX_BYTES]) -> usize) {
    let head = DCD_TX_HEAD.load(Relaxed);
    let k = head as usize % DCD_SLOTS;
    // Safety: the slot isn't queued (dcd_slot_free), so core 1 doesn't read it.
    let len = build(unsafe { &mut (*DCD_TX.0.get())[k] });
    DCD_TX_LEN[k].store(len.min(DCD_TX_BYTES) as u32, Relaxed);
    DCD_TX_EPOCH[k].store(epoch, Relaxed);
    DCD_TX_HEAD.store(head.wrapping_add(1), Release);
}

/// PIO0 SM0 registers for sending DCD replies from core 1, a byte per FIFO entry.
#[derive(Clone, Copy)]
pub struct DcdPort {
    txf: *mut u32,
    fstat: *const u32,
    fdebug: *mut u32,
    shiftctrl: *mut u32,
}

// Safety: only core 1 uses the registers once it has started.
unsafe impl Send for DcdPort {}

/// FSTAT.TXEMPTY / FSTAT.TXFULL / FDEBUG.TXSTALL for SM0.
const SM0_TXEMPTY: u32 = 1 << 24;
const SM0_TXFULL: u32 = 1 << 16;
const SM0_TXSTALL: u32 = 1 << 24;
/// SM_SHIFTCTRL.PULL_THRESH.
const PULL_THRESH_SHIFT: u32 = 25;

impl Stream {
    /// The registers `run_dcd` sends through, instead of the DMA.
    pub fn dcd_port(&self) -> DcdPort {
        let pio0 = unsafe { &*pac::PIO0::ptr() };
        DcdPort {
            txf: self.player.txf as *mut u32,
            fstat: pio0.fstat().as_ptr(),
            fdebug: pio0.fdebug().as_ptr(),
            shiftctrl: pio0.sm(0).sm_shiftctrl().as_ptr(),
        }
    }
}

impl DcdPort {
    /// Pull 8 bits at a time, so each FIFO entry (in its top byte) is one IWM byte. This also
    /// flushes the FIFO; the 8-entry joined FIFO holds a whole group.
    #[inline(always)]
    fn bytewise(&self) {
        unsafe {
            let shift = read_volatile(self.shiftctrl) & !(0x1F << PULL_THRESH_SHIFT);
            write_volatile(self.shiftctrl, (shift | 8 << PULL_THRESH_SHIFT) ^ FJOIN_TX);
            write_volatile(self.shiftctrl, shift | 8 << PULL_THRESH_SHIFT);
        }
    }

    /// Back to whole words ([`Player::play`]'s), after [`bytewise`](Self::bytewise) (HD20 +
    /// floppy mode). The next `play` flushes the FIFO.
    #[inline(always)]
    fn wordwise(&self) {
        unsafe {
            let shift = read_volatile(self.shiftctrl) & !(0x1F << PULL_THRESH_SHIFT);
            write_volatile(self.shiftctrl, shift); // 0 means 32
        }
    }

    #[inline(always)]
    fn push(&self, byte: u8) {
        unsafe { write_volatile(self.txf, u32::from(byte) << 24) };
    }

    #[inline(always)]
    fn empty(&self) -> bool {
        unsafe { read_volatile(self.fstat) & SM0_TXEMPTY != 0 }
    }

    #[inline(always)]
    fn full(&self) -> bool {
        unsafe { read_volatile(self.fstat) & SM0_TXFULL != 0 }
    }

    /// The state machine has run out of bytes (and finished the last one's cells) since
    /// [`clear_stall`](Self::clear_stall).
    #[inline(always)]
    fn stalled(&self) -> bool {
        unsafe { read_volatile(self.fdebug) & SM0_TXSTALL != 0 }
    }

    #[inline(always)]
    fn clear_stall(&self) {
        unsafe { write_volatile(self.fdebug, SM0_TXSTALL) };
    }

    #[inline(always)]
    fn flush(&self) {
        unsafe {
            let shift = read_volatile(self.shiftctrl);
            write_volatile(self.shiftctrl, shift ^ FJOIN_TX);
            write_volatile(self.shiftctrl, shift);
        }
    }
}

/// Byte `i` of reply slot `k`.
#[inline(always)]
fn dcd_byte(k: usize, i: u32) -> u8 {
    unsafe { read_volatile((DCD_TX.0.get() as *const u8).add(k % DCD_SLOTS * DCD_TX_BYTES + i as usize % DCD_TX_BYTES)) }
}

// `run_dcd` phases.
/// Nothing under way.
const DCD_IDLE: u8 = 0;
/// /HSHK asserted for a command: the Mac is writing (or about to).
const DCD_RECEIVING: u8 = 1;
/// /HSHK asserted for a reply, in state 2: waiting for the Mac to come and read it.
const DCD_OFFERED: u8 = 2;
/// The Mac is reading a reply.
const DCD_SENDING: u8 = 3;

/// Core 1 entry point in HD20 mode. Never returns; runs from RAM with interrupts off, like
/// [`run`].
///
/// The hot path is: sample the pins, set RD for the state (high-impedance unless selected, an
/// image is loaded and LSTRB hasn't passed the Mac on), follow the transfer.
#[unsafe(link_section = ".data.dcd_loop")]
#[inline(never)]
pub fn run_dcd(port: DcdPort, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let timer = unsafe { (*pac::TIMER0::ptr()).timerawl().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };

    let base = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let stream = base | OE_ENABLE << OEOVER_SHIFT | OUT_NORMAL << OUTOVER_SHIFT;
    let low = base | OE_ENABLE << OEOVER_SHIFT | OUT_LOW << OUTOVER_SHIFT;
    let high = base | OE_ENABLE << OEOVER_SHIFT | OUT_HIGH << OUTOVER_SHIFT;

    let enbl_mask = 1u32 << PIN_ENBL;
    let lstrb_mask = 1u32 << PIN_LSTRB;

    port.bytewise();
    let mut out = hiz;
    unsafe { write_volatile(rd_ctrl, out) };

    let mut phase = DCD_IDLE;
    // /HSHK asserted (RD low in states 2 and 3).
    let mut hshk = false;
    // CA2–CA0 as of the last pass, or 8 while not answering.
    let mut prev = 8u32;
    // A transaction just ended: wait for state 2 before taking state 3 as a new command.
    let mut need_idle = false;
    // LSTRB passed the Mac on to the next device: not answering until /ENBL goes high.
    let mut chained = false;
    let mut lstrb_hist = 0u32;
    let mut present = false;
    let mut live = false;
    let mut epoch = DCD_EPOCH.load(Relaxed);
    let mut pass = 0u32;
    let mut selected_seen = 0u32;

    // The reply being sent: its slot, length, bytes whose cells are out, the group under way.
    let mut slot = 0usize;
    let mut send_len = 0u32;
    let mut sent = 0u32;
    let mut queued = 0u32;
    let mut group_end = 0u32;
    let mut in_group = false;
    // An `AA` goes before the next group: at the start, and after a holdoff.
    let mut resync = true;
    let mut held = false;

    // The command being captured.
    let mut got_data = false;
    let mut closing = false;
    let mut closed_at = 0u32;
    let mut cap_start = 0u32;
    let mut total = 0u32;

    loop {
        let inp = unsafe { read_volatile(gpio_in) };
        let selected = inp & enbl_mask == 0;
        let on = selected && present && !chained;
        let state = (inp >> PIN_CA0) & 0b111;

        // RD: data in 0 and 1, /HSHK in 2 and 3, low in 4 and 5, high in 6 and 7.
        let want = if !on {
            hiz
        } else if state < 2 {
            stream
        } else if state < 4 {
            if hshk { low } else { high }
        } else if state < 6 {
            low
        } else {
            high
        };
        if want != out {
            out = want;
            unsafe { write_volatile(rd_ctrl, out) };
        }
        selected_seen |= selected as u32;

        // LSTRB (CA3) pulse: the Mac moves on to the next device in the chain.
        lstrb_hist = (lstrb_hist << 1 | (inp & lstrb_mask != 0) as u32) & 0b111;
        if !selected {
            chained = false;
        } else if lstrb_hist == 0b011 {
            chained = true;
        }

        if !on {
            if prev != 8 {
                // Deselected or passed on mid-transaction: it's over. Drop what was owed.
                if phase == DCD_RECEIVING {
                    capture.close(0);
                }
                if phase == DCD_SENDING {
                    port.flush();
                }
                phase = DCD_IDLE;
                hshk = false;
                in_group = false;
                need_idle = false;
                epoch = epoch.wrapping_add(1);
                DCD_EPOCH.store(epoch, Release);
                DCD_TX_TAIL.store(DCD_TX_HEAD.load(Acquire), Release);
                prev = 8;
            }
        } else {
            if state != prev {
                if state == 5 {
                    // The Mac is probing for drives (R10), as at startup.
                    DCD_PROBES.fetch_add(1, Relaxed);
                    live = true;
                    DCD_LIVE.store(true, Relaxed);
                }
                match phase {
                    DCD_RECEIVING => {
                        if state < 2 {
                            got_data = true;
                        } else if state == 3 && got_data {
                            // Done: release /HSHK, and let the separator finish the last word.
                            hshk = false;
                            closing = true;
                            closed_at = unsafe { read_volatile(timer) };
                            phase = DCD_IDLE;
                            need_idle = true;
                        } else if state != 3 {
                            // Gave up before sending, or went to sense a status bit.
                            capture.close(0);
                            hshk = false;
                            phase = DCD_IDLE;
                        }
                    }
                    DCD_OFFERED => {
                        if state == 1 {
                            phase = DCD_SENDING;
                            slot = DCD_TX_TAIL.load(Relaxed) as usize % DCD_SLOTS;
                            send_len = DCD_TX_LEN[slot].load(Relaxed);
                            sent = 0;
                            in_group = false;
                            resync = true;
                            held = false;
                        } else if state >= 4 {
                            hshk = false;
                            phase = DCD_IDLE;
                        }
                    }
                    DCD_SENDING => {
                        if state >= 2 {
                            // The Mac has read what it wanted (or gave up): the reply is done.
                            if sent < send_len {
                                port.flush();
                            }
                            DCD_TX_TAIL.store(DCD_TX_TAIL.load(Relaxed).wrapping_add(1), Release);
                            hshk = false;
                            in_group = false;
                            phase = DCD_IDLE;
                            need_idle = true;
                        } else if state == 0 {
                            held = true;
                        }
                    }
                    _ => {}
                }
                prev = state;
            }

            match phase {
                DCD_IDLE => {
                    if state == 2 {
                        need_idle = false;
                        let tail = DCD_TX_TAIL.load(Relaxed);
                        if tail != DCD_TX_HEAD.load(Acquire) {
                            let k = tail as usize % DCD_SLOTS;
                            if DCD_TX_EPOCH[k].load(Relaxed) == epoch {
                                // A reply is ready: ask the Mac to come and read it.
                                hshk = true;
                                phase = DCD_OFFERED;
                            } else {
                                DCD_TX_TAIL.store(tail.wrapping_add(1), Release);
                            }
                        }
                    } else if state == 3
                        && !need_idle
                        && live
                        && !closing
                        && CAP_HEAD.load(Relaxed) == CAP_TAIL.load(Acquire)
                    {
                        // HOST: a command. Whatever is still queued answers an older one.
                        epoch = epoch.wrapping_add(1);
                        DCD_EPOCH.store(epoch, Release);
                        DCD_TX_TAIL.store(DCD_TX_HEAD.load(Acquire), Release);
                        capture.open(0);
                        cap_start = capture.position();
                        hshk = true;
                        got_data = false;
                        phase = DCD_RECEIVING;
                    }
                }
                DCD_SENDING => {
                    if !in_group && sent < send_len && state == 1 {
                        // The last group is out (the state machine has stalled): start the next,
                        // with `AA` before it if this is the start or the Mac held off. The
                        // state machine takes the first byte at once; once it has, a stall means
                        // the group is out.
                        port.push(if resync { crate::dcd::SYNC } else { dcd_byte(slot, sent) });
                        while !port.empty() {}
                        port.clear_stall();
                        queued = if resync { sent } else { sent + 1 };
                        group_end = (sent + 8).min(send_len);
                        resync = false;
                        held = false;
                        in_group = true;
                    }
                    if in_group {
                        while queued < group_end && !port.full() {
                            port.push(dcd_byte(slot, queued));
                            queued += 1;
                        }
                        if queued == group_end {
                            if group_end == send_len {
                                // All queued: by the time the Mac goes to state 3, /HSHK is up.
                                hshk = false;
                            }
                            if port.stalled() {
                                in_group = false;
                                sent = group_end;
                                resync = held;
                            }
                        }
                    }
                    if !in_group && state == 0 {
                        // Holdoff: the next group waits for state 1, and starts with `AA`.
                        resync = true;
                    }
                }
                _ => {}
            }
        }

        pass = pass.wrapping_add(1);
        if pass & 0xF != 0 {
            continue;
        }

        // ---- Housekeeping, every 16th pass ----
        let now = unsafe { read_volatile(timer) };
        if closing && now.wrapping_sub(closed_at) >= CAPTURE_TAIL_US {
            // Queue the command for core 0.
            closing = false;
            capture.close(0);
            let words = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            let head = CAP_HEAD.load(Relaxed);
            let k = head as usize % CAPTURE_QUEUE;
            CAP_START[k].store(cap_start, Relaxed);
            CAP_WORDS[k].store(words, Relaxed);
            CAP_POS[k].store(0, Relaxed);
            CAP_INFO[k].store(0, Relaxed);
            CAP_SEQ[k].store(total, Relaxed);
            total = total.wrapping_add(words);
            CAP_TOTAL.store(total, Release);
            CAP_HEAD.store(head.wrapping_add(1), Release);
        }
        present = DCD_PRESENT.load(Relaxed);
        live = DCD_LIVE.load(Relaxed);
        if selected_seen != 0 {
            SELECTED.store(true, Relaxed);
            selected_seen = 0;
        }
        if pass & 0xFF == 0 {
            LOOPS.store(pass >> 8, Relaxed);
        }
    }
}

// ---- HD20 with a floppy behind it ------------------------------------------------------------
// In HD20 + floppy mode core 1 runs `run_chain`: an HD20 (as `run_dcd`) with a SuperDrive (as
// `run`) chained behind it, as on a real HD20's floppy port. When the Mac asserts /ENBL the HD20
// answers; a pulse on LSTRB (CA3) passes the Mac on to the next device, the floppy, which then
// answers until /ENBL goes high.
// With no HD20 image loaded the floppy answers at once, as if it were alone.
// The two share PIO0 SM0 (the floppy's track, or an HD20 reply a byte at a time) and the capture
// ring and queue (an HD20 command is a capture flagged CAP_DCD).

/// Core 1 entry point in HD20 + floppy mode. Never returns; runs from RAM with interrupts off,
/// like [`run`].
///
/// Two inner loops, each with its own hot path: the HD20's ([`run_dcd`]'s, with RD driven only for
/// a reply going out in states 0 and 1, since SM0 may be playing the floppy's track), until the
/// Mac passes it on; then the floppy's ([`run`]'s, a SuperDrive), until /ENBL goes high. The pass
/// pulse itself is no floppy command. SM0 is switched to bytes ([`DcdPort::bytewise`], the track
/// stopped) when an HD20 reply starts going out, and back to the track, at the word it had
/// reached, when the floppy is next selected: the disk stands still meanwhile, which the Mac can't
/// tell. Captures go through the GCR separator for the HD20 and the floppy's (GCR or MFM) for its
/// writes, the DMA pointed at whichever is opened ([`Capture::retarget`]).
#[unsafe(link_section = ".data.chain_loop")]
#[inline(never)]
pub fn run_chain(player: Player, port: DcdPort, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let timer = unsafe { (*pac::TIMER0::ptr()).timerawl().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };

    let base = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let stream = base | OE_ENABLE << OEOVER_SHIFT | OUT_NORMAL << OUTOVER_SHIFT;
    let low = base | OE_ENABLE << OEOVER_SHIFT | OUT_LOW << OUTOVER_SHIFT;
    let high = base | OE_ENABLE << OEOVER_SHIFT | OUT_HIGH << OUTOVER_SHIFT;

    let enbl_mask = 1u32 << PIN_ENBL;
    let lstrb_mask = 1u32 << PIN_LSTRB;
    let wrreq_mask = 1u32 << PIN_WRREQ;

    let mut out = hiz;
    unsafe { write_volatile(rd_ctrl, out) };

    // The floppy, as in `sony::<false>`.
    let mut track: u8 = 0;
    let mut dir_out = false;
    let mut motor_on = false;
    let mut tach = false;
    let mut tach_next = unsafe { read_volatile(timer) };
    let mut tach_edges = 0u32;
    let mut tach_flutter = 0u32;
    let mut disk_in = false;
    let mut ready = false;
    let mut playing = 0u32;
    let mut side = 0u8;
    let mut hd = false;
    let mut writable = false;
    let mut status = status_bits(dir_out, track, motor_on, tach, false, disk_in, ready, hd, writable, true, false);
    let mut data_out = stream;
    // SM0 is sending HD20 bytes (the track stopped), and the word to start the track at again.
    let mut bytes = false;
    let mut resume = 0u32;

    // The HD20, as in `run_dcd`.
    let mut phase = DCD_IDLE;
    let mut hshk = false;
    let mut prev = 8u32;
    let mut need_idle = false;
    let mut present = false;
    let mut live = false;
    let mut epoch = DCD_EPOCH.load(Relaxed);
    let mut got_data = false;
    let mut slot = 0usize;
    let mut send_len = 0u32;
    let mut sent = 0u32;
    let mut queued = 0u32;
    let mut group_end = 0u32;
    let mut in_group = false;
    let mut resync = true;
    let mut held = false;

    let mut lstrb_hist = 0u32;
    let mut seen = 0u32;
    let mut pass = 0u32;

    // The capture, the HD20's or the floppy's: one at a time. `capture_sm` is the separator open
    // (or closing); `dma_sm` the one DMA channel 2 reads.
    let mut writing = false;
    let mut closing = false;
    let mut capture_sm = 0usize;
    let mut dma_sm = 0usize;
    let mut opened_at = 0u32;
    let mut closed_at = 0u32;
    let mut cap_start = 0u32;
    let mut cap_pos = 0u32;
    let mut cap_info = 0u32;
    let mut total = 0u32;

    // Open separator `sm` on the ring, the DMA pointed at it.
    macro_rules! open_capture {
        ($sm:expr) => {{
            capture_sm = $sm;
            if dma_sm != capture_sm {
                capture.retarget(capture_sm);
                dma_sm = capture_sm;
            }
            capture.open(capture_sm);
            cap_start = capture.position();
        }};
    }
    // The capture closing, once its tail is in (or `$now_too`): stop it and queue it.
    macro_rules! finish_capture {
        ($now:expr, $now_too:expr) => {
            if closing && ($now_too || $now.wrapping_sub(closed_at) >= CAPTURE_TAIL_US) {
                closing = false;
                capture.close(capture_sm);
                let words = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
                let head = CAP_HEAD.load(Relaxed);
                if head.wrapping_sub(CAP_TAIL.load(Acquire)) < CAPTURE_QUEUE as u32 {
                    let k = head as usize % CAPTURE_QUEUE;
                    CAP_START[k].store(cap_start, Relaxed);
                    CAP_WORDS[k].store(words, Relaxed);
                    CAP_POS[k].store(cap_pos, Relaxed);
                    CAP_INFO[k].store(cap_info, Relaxed);
                    CAP_SEQ[k].store(total, Relaxed);
                    CAP_HEAD.store(head.wrapping_add(1), Release);
                } else {
                    CAP_DROPPED.fetch_add(1, Relaxed);
                }
                total = total.wrapping_add(words);
                CAP_TOTAL.store(total, Release);
                if cap_info & CAP_DCD == 0 {
                    WRITING.store(false, Release);
                }
            }
        };
    }
    // The HD20 deselected or passed on: whatever transaction was under way is over.
    macro_rules! dcd_drop {
        () => {
            if prev != 8 {
                if phase == DCD_RECEIVING {
                    capture.close(capture_sm);
                }
                if phase == DCD_SENDING {
                    port.flush();
                }
                phase = DCD_IDLE;
                hshk = false;
                in_group = false;
                need_idle = false;
                epoch = epoch.wrapping_add(1);
                DCD_EPOCH.store(epoch, Release);
                DCD_TX_TAIL.store(DCD_TX_HEAD.load(Acquire), Release);
                prev = 8;
            }
        };
    }

    loop {
        // ==== The HD20: until the Mac passes it on, or at once with no image ====
        loop {
            let inp = unsafe { read_volatile(gpio_in) };
            let selected = inp & enbl_mask == 0;
            let state = (inp >> PIN_CA0) & 0b111;
            lstrb_hist = (lstrb_hist << 1 | (inp & lstrb_mask != 0) as u32) & 0b111;
            if selected && (!present || lstrb_hist == 0b011) {
                // On to the floppy.
                dcd_drop!();
                break;
            }

            // RD: data in 0 and 1 (only a reply's: SM0 may be playing the floppy's track), /HSHK
            // in 2 and 3, low in 4 and 5, high in 6 and 7.
            let want = if !selected {
                hiz
            } else if state < 2 {
                if phase == DCD_SENDING { stream } else { high }
            } else if state < 4 {
                if hshk { low } else { high }
            } else if state < 6 {
                low
            } else {
                high
            };
            if want != out {
                out = want;
                unsafe { write_volatile(rd_ctrl, out) };
            }
            seen |= selected as u32;

            if !selected {
                dcd_drop!();
            } else {
                if state != prev {
                    if state == 5 {
                        DCD_PROBES.fetch_add(1, Relaxed);
                        live = true;
                        DCD_LIVE.store(true, Relaxed);
                    }
                    match phase {
                        DCD_RECEIVING => {
                            if state < 2 {
                                got_data = true;
                            } else if state == 3 && got_data {
                                hshk = false;
                                closing = true;
                                closed_at = unsafe { read_volatile(timer) };
                                phase = DCD_IDLE;
                                need_idle = true;
                            } else if state != 3 {
                                capture.close(capture_sm);
                                hshk = false;
                                phase = DCD_IDLE;
                            }
                        }
                        DCD_OFFERED => {
                            if state == 1 {
                                if !bytes {
                                    // The floppy's track stops (where it was) for the reply.
                                    if cylinder_track(playing).is_some() {
                                        resume = player.next_word(cylinder_set(playing), side);
                                    }
                                    player.stop();
                                    player.set_clkdiv(CLKDIV[1].load(Relaxed));
                                    port.bytewise();
                                    bytes = true;
                                }
                                phase = DCD_SENDING;
                                slot = DCD_TX_TAIL.load(Relaxed) as usize % DCD_SLOTS;
                                send_len = DCD_TX_LEN[slot].load(Relaxed);
                                sent = 0;
                                in_group = false;
                                resync = true;
                                held = false;
                            } else if state >= 4 {
                                hshk = false;
                                phase = DCD_IDLE;
                            }
                        }
                        DCD_SENDING => {
                            if state >= 2 {
                                if sent < send_len {
                                    port.flush();
                                }
                                DCD_TX_TAIL.store(DCD_TX_TAIL.load(Relaxed).wrapping_add(1), Release);
                                hshk = false;
                                in_group = false;
                                phase = DCD_IDLE;
                                need_idle = true;
                            } else if state == 0 {
                                held = true;
                            }
                        }
                        _ => {}
                    }
                    prev = state;
                }

                match phase {
                    DCD_IDLE => {
                        if state == 2 {
                            need_idle = false;
                            let tail = DCD_TX_TAIL.load(Relaxed);
                            if tail != DCD_TX_HEAD.load(Acquire) {
                                let k = tail as usize % DCD_SLOTS;
                                if DCD_TX_EPOCH[k].load(Relaxed) == epoch {
                                    hshk = true;
                                    phase = DCD_OFFERED;
                                } else {
                                    DCD_TX_TAIL.store(tail.wrapping_add(1), Release);
                                }
                            }
                        } else if state == 3
                            && !need_idle
                            && live
                            && !closing
                            && CAP_HEAD.load(Relaxed) == CAP_TAIL.load(Acquire)
                        {
                            // HOST: a command, through the GCR separator.
                            epoch = epoch.wrapping_add(1);
                            DCD_EPOCH.store(epoch, Release);
                            DCD_TX_TAIL.store(DCD_TX_HEAD.load(Acquire), Release);
                            open_capture!(0);
                            cap_pos = 0;
                            cap_info = CAP_DCD;
                            hshk = true;
                            got_data = false;
                            phase = DCD_RECEIVING;
                        }
                    }
                    DCD_SENDING => {
                        if !in_group && sent < send_len && state == 1 {
                            port.push(if resync { crate::dcd::SYNC } else { dcd_byte(slot, sent) });
                            while !port.empty() {}
                            port.clear_stall();
                            queued = if resync { sent } else { sent + 1 };
                            group_end = (sent + 8).min(send_len);
                            resync = false;
                            held = false;
                            in_group = true;
                        }
                        if in_group {
                            while queued < group_end && !port.full() {
                                port.push(dcd_byte(slot, queued));
                                queued += 1;
                            }
                            if queued == group_end {
                                if group_end == send_len {
                                    hshk = false;
                                }
                                if port.stalled() {
                                    in_group = false;
                                    sent = group_end;
                                    resync = held;
                                }
                            }
                        }
                        if !in_group && state == 0 {
                            resync = true;
                        }
                    }
                    _ => {}
                }
            }

            pass = pass.wrapping_add(1);
            if pass & 0xF != 0 {
                continue;
            }

            // ---- Housekeeping, every 16th pass ----
            let now = unsafe { read_volatile(timer) };
            finish_capture!(now, false);
            present = DCD_PRESENT.load(Relaxed);
            live = DCD_LIVE.load(Relaxed);
            if seen != 0 {
                SELECTED.store(true, Relaxed);
                seen = 0;
            }
            if pass & 0xFF == 0 {
                LOOPS.store(pass >> 8, Relaxed);
            }
        }

        // ==== The floppy, until /ENBL goes high ====
        if bytes {
            port.wordwise();
            player.set_clkdiv(CLKDIV[0].load(Relaxed));
            if cylinder_track(playing).is_some() {
                player.play_cylinder(cylinder_set(playing), side, resume);
            }
            bytes = false;
        }
        loop {
            let inp = unsafe { read_volatile(gpio_in) };
            let r = (inp >> PIN_SEL) & 0xF;
            let enabled = inp & enbl_mask == 0;
            if !enabled {
                break;
            }

            // Register mux (`run`'s).
            let want = if r & 0b1110 == 8 {
                data_out
            } else if status >> r & 1 != 0 {
                high
            } else {
                low
            };
            if want != out {
                out = want;
                unsafe { write_volatile(rd_ctrl, out) };
            }
            seen |= 1;

            // Head select.
            if r & 0b1110 == 8 && inp & lstrb_mask == 0 {
                let sel = (r & 1) as u8;
                if sel != side {
                    if cylinder_track(playing).is_some() {
                        let set = cylinder_set(playing);
                        let start = player.spin_start(status & KEEP_ROTATION_BIT != 0, set, side, set, sel);
                        player.play_cylinder(set, sel, start);
                    }
                    side = sel;
                    SIDE.store(side, Relaxed);
                }
            }

            // Commands: LSTRB low, high, high.
            lstrb_hist = (lstrb_hist << 1 | (inp & lstrb_mask != 0) as u32) & 0b111;
            if lstrb_hist == 0b011 {
                let (latch, ca2) = (r as u8 & 0b111, r & 0b1000 != 0);
                match latch {
                    0b000 => dir_out = ca2,
                    0b010 if !ca2 => {
                        track = if dir_out {
                            track.saturating_sub(1)
                        } else {
                            (track + 1).min(crate::track::TRACKS - 1)
                        };
                        ready = false;
                        HEAD_TRACK.store(track, Relaxed);
                        STEPS.fetch_add(1, Relaxed);
                    }
                    0b100 => {
                        motor_on = !ca2;
                        MOTOR_ON.store(motor_on, Relaxed);
                    }
                    0b110 if ca2 => {
                        disk_in = false;
                        DISK_IN.store(false, Relaxed);
                        MAC_SAW_EMPTY.store(true, Relaxed);
                        EJECTS.fetch_add(1, Relaxed);
                    }
                    0b011 => status = status & !MFM_MODE_BIT | if ca2 { 0 } else { MFM_MODE_BIT },
                    _ => {}
                }
                ready &= motor_on;
                let r7 = status >> 7 & 1 != 0;
                let mfm_mode = status & MFM_MODE_BIT != 0;
                let keep = status & KEEP_ROTATION_BIT;
                status = status_bits(dir_out, track, motor_on, r7, mfm_mode, disk_in, ready, hd, writable, true, false) | keep;
                LAST_CMD.store((latch << 1) | ca2 as u8, Relaxed);
                STROBES.fetch_add(1, Relaxed);
            }

            pass = pass.wrapping_add(1);
            if pass & 0xF != 0 {
                continue;
            }

            // ---- Housekeeping, every 16th pass (`run`'s) ----
            let now = unsafe { read_volatile(timer) };
            if now.wrapping_sub(tach_next) as i32 >= 0 {
                tach = !tach;
                tach_edges += 1;
                if tach_edges >= 120 {
                    tach_edges = 0;
                    tach_flutter = if tach_flutter >= 4 { 0 } else { tach_flutter + 1 };
                }
                let zone = (gcr::zone(track) as usize).min(TACH_HALF_US.len() - 1);
                tach_next = now.wrapping_add(TACH_HALF_US[zone] - tach_flutter);
            }

            let gate = inp & wrreq_mask == 0;
            finish_capture!(now, gate);
            if gate && !writing && !closing {
                if writable && disk_in && cylinder_track(playing).is_some() {
                    writing = true;
                    open_capture!(MFM.load(Relaxed) as usize);
                    let set = cylinder_set(playing);
                    let len = CYL_LEN[set][side as usize].load(Relaxed);
                    let next = player.next_word(set, side);
                    cap_pos = ((next + len - 1) % len.max(1)) * 32 + 16;
                    cap_info = playing | if side == 1 { CAP_SIDE } else { 0 };
                    opened_at = now;
                    WRITING.store(true, Relaxed);
                }
            } else if !gate && writing {
                writing = false;
                closing = true;
                closed_at = now;
                if now.wrapping_sub(opened_at) > FORMAT_GATE_US {
                    cap_info |= CAP_FORMAT;
                    player.play_cylinder(cylinder_set(playing), side, 0);
                }
            }
            if writing {
                let so_far = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
                CAP_TOTAL.store(total.wrapping_add(so_far), Release);
            }

            let published = PUBLISHED.load(Acquire);
            if published != playing {
                let start = if cylinder_track(published).is_none() || cylinder_track(playing).is_none() {
                    0
                } else if cylinder_track(published) == cylinder_track(playing) {
                    player.next_word(cylinder_set(playing), side)
                } else {
                    let keep = status & KEEP_ROTATION_BIT != 0;
                    player.spin_start(keep, cylinder_set(playing), side, cylinder_set(published), side)
                };
                playing = published;
                // At the loaded image's cell rate (core 0 doesn't touch SM0's: it may be sending
                // an HD20 reply at the GCR rate).
                player.set_clkdiv(CLKDIV[0].load(Relaxed));
                player.play_cylinder(cylinder_set(playing), side, start);
                PLAYING.store(playing, Release);
            }
            if seen != 0 {
                SELECTED.store(true, Relaxed);
                seen = 0;
                let pins = unsafe { read_volatile(gpio_in) };
                if pins & enbl_mask == 0 && (pins >> PIN_SEL) & 0xF == 1 && !disk_in {
                    MAC_SAW_EMPTY.store(true, Relaxed);
                }
            }
            disk_in = DISK_IN.load(Relaxed);
            hd = HD_MEDIA.load(Relaxed);
            writable = WRITABLE.load(Relaxed) && disk_in;
            ready = motor_on && disk_in && cylinder_track(playing) == Some(track);
            let mfm_mode = status & MFM_MODE_BIT != 0;
            let index_writing = writing && capture_sm == 1;
            let mut index_high = true;
            if mfm_mode || index_writing {
                let len = CYL_LEN[cylinder_set(playing)][side as usize].load(Relaxed) * 32;
                let rev = if len == 0 { INDEX_PERIOD_US } else { len };
                index_high = now % rev >= INDEX_LOW_US;
            }
            let r7 = if mfm_mode { index_high } else { tach };
            status = status_bits(dir_out, track, motor_on, r7, mfm_mode, disk_in, ready, hd, writable, true, false)
                | if KEEP_ROTATION.load(Relaxed) { KEEP_ROTATION_BIT } else { 0 };
            data_out = if !index_writing {
                stream
            } else if index_high {
                high
            } else {
                low
            };
            present = DCD_PRESENT.load(Relaxed);
            if pass & 0xFF == 0 {
                LOOPS.store(pass >> 8, Relaxed);
            }
        }
        // Deselected: RD lets go at once (the HD20's loop would on its first pass anyway), and a
        // write under way ends, as its gate has.
        out = hiz;
        unsafe { write_volatile(rd_ctrl, out) };
        if writing {
            let now = unsafe { read_volatile(timer) };
            writing = false;
            closing = true;
            closed_at = now;
            if now.wrapping_sub(opened_at) > FORMAT_GATE_US {
                cap_info |= CAP_FORMAT;
                player.play_cylinder(cylinder_set(playing), side, 0);
            }
        }
    }
}

// ---- Apple II (Disk II) ----------------------------------------------------------------------
// In Apple II mode core 1 runs `run_a2`: a Disk II, the 5.25" drive of the Apple II family. It
// has no register file. While /ENBL (the drive enable, which is also the motor) is low, RD
// carries the read pulses and WRPROT the write-protect switch; both are high-impedance
// otherwise, so drives can share the lines. The four phase lines drive the head's stepper
// magnets directly (`a2::step`). Writes are captured as on the Mac (GCR, NRZI), at the Apple II's
// cell. Core 0 builds the track heard at the head's quarter track (`Disk::a2_track`) and
// publishes it tagged with that quarter track.

/// Head position in quarter tracks (0–159), published by core 1.
pub static HEAD_QUARTER: AtomicU8 = AtomicU8::new(0);
/// The phase lines as last seen while enabled, bit `i` = PHi (for the display).
pub static PHASES: AtomicU8 = AtomicU8::new(0);
/// The buffer that plays while the head's track isn't ready: core 0's noise (as a real drive
/// reads between tracks or off unformatted media). Set by core 0 before core 1 starts.
pub static NOISE_PTR: AtomicU32 = AtomicU32::new(0);
pub static NOISE_LEN: AtomicU32 = AtomicU32::new(0);
/// Where a Disk II track was started: when (µs), at which buffer cell, and the track's cells,
/// rotation (cells), 2^32 / cells ([`CYL_BITS`] and friends) and cells a µs × 2^32 ([`CYL_Q32`]).
/// The disk's angle is worked out from the last one, also while noise plays between tracks.
#[derive(Clone, Copy)]
struct Spin {
    at: u32,
    cell: u32,
    bits: u32,
    rotc: u32,
    inv: u32,
    q32: u32,
}

impl Spin {
    /// Before any track: the disk's cell, at the angle of cell 0.
    #[inline(always)]
    fn new() -> Self {
        Spin { at: 0, cell: 0, bits: 1, rotc: 0, inv: 0, q32: A2_CELLS_Q32.load(Relaxed) }
    }

    /// Set `set`'s track, started at `cell` at `now`.
    #[inline(always)]
    fn of(set: usize, cell: u32, now: u32) -> Self {
        Spin {
            at: now,
            cell,
            bits: CYL_BITS[set].load(Relaxed),
            rotc: CYL_ROTC[set].load(Relaxed),
            inv: CYL_INV[set].load(Relaxed),
            q32: CYL_Q32[set].load(Relaxed),
        }
    }

    /// The buffer cell under the head at `now`.
    #[inline(always)]
    fn cell(&self, now: u32) -> u32 {
        let bits = self.bits.max(1);
        let turned = (u64::from(now.wrapping_sub(self.at)) * u64::from(self.q32) >> 32) as u32;
        (self.cell + turned % bits) % bits
    }

    /// The cell to start set `set`'s track at, at `now`: the same fraction of a revolution, as
    /// imaged, as the head is at on this one.
    #[inline(always)]
    fn start_in(&self, set: usize, now: u32) -> u32 {
        let imaged = (self.cell(now) + self.rotc) % self.bits.max(1);
        // imaged < bits and inv = 2^32 / bits: the product is the fraction, below 2^32.
        let fraction = imaged.wrapping_mul(self.inv);
        let bits = CYL_BITS[set].load(Relaxed).max(1);
        let there = (u64::from(fraction) * u64::from(bits) >> 32) as u32;
        (there + bits - CYL_ROTC[set].load(Relaxed) % bits) % bits
    }
}

/// Play set `set`'s Disk II track from `cell`, at its own cell rate (Apple II mode).
#[inline(always)]
fn play_track(player: &Player, set: usize, cell: u32) {
    player.set_clkdiv(CYL_CLKDIV[set].load(Relaxed));
    // Safety: core 0 doesn't touch a set while it is in PLAYING.
    unsafe { player.play_a2(CYL_PTR[set][0].load(Relaxed), CYL_LEN[set][0].load(Relaxed), CYL_BITS[set].load(Relaxed), cell) };
}

/// Play the noise buffer, as if a track of all but one of its cells (Apple II mode).
#[inline(always)]
fn play_noise(player: &Player) {
    let len = NOISE_LEN.load(Relaxed);
    unsafe { player.play_a2(NOISE_PTR.load(Relaxed), len, len * 32 - 1, 0) };
}

/// Core 1 entry point in Apple II mode. Never returns; runs from RAM with interrupts off, like
/// [`run`].
///
/// The hot path is: sample the pins, set RD and WRPROT for /ENBL, move the head when the phases
/// change. Write gates, newly published tracks and disk state are handled every 16th pass.
///
/// The disk turns whatever track is under the head. Core 1 knows where it is from the timer: the
/// cell a track started at, when, and the PIO's cell rate ([`Spin`]). A new track starts at the
/// same fraction of a revolution as imaged (through the buffers' rotations, [`CYL_ROTC`]), to the
/// cell, so a WOZ imaged with cross-track sync lines up as it did on the disk; a track
/// republished after a write carries on at the cell it was at. Tracks loop at exactly their
/// cells (`src/disk2.pio`, [`Player::play_a2`]).
#[unsafe(link_section = ".data.a2_loop")]
#[inline(never)]
pub fn run_a2(player: Player, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let timer = unsafe { (*pac::TIMER0::ptr()).timerawl().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };
    let wp_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_WRPROT as usize).gpio_ctrl().as_ptr() };

    let base = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let rd_hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let rd_stream = base | OE_ENABLE << OEOVER_SHIFT | OUT_NORMAL << OUTOVER_SHIFT;
    let rd_high = base | OE_ENABLE << OEOVER_SHIFT | OUT_HIGH << OUTOVER_SHIFT;
    let base = unsafe { read_volatile(wp_ctrl) } & !OVER_MASK;
    let wp_hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let wp_low = base | OE_ENABLE << OEOVER_SHIFT | OUT_LOW << OUTOVER_SHIFT;
    let wp_high = base | OE_ENABLE << OEOVER_SHIFT | OUT_HIGH << OUTOVER_SHIFT;

    let enbl_mask = 1u32 << PIN_ENBL;
    let wrreq_mask = 1u32 << PIN_WRREQ;
    // Selected: /ENBL low; and on a IIgs, /3.5DISK high, or it's talking to its 3.5" drives
    // (with /ENBL low too), and this drive mustn't step on their commands or answer for them on
    // WRPROT (their SENSE): with a writable disk the IIgs found a 3.5" drive here and never
    // booted slot 6. It's only checked once seen high: on a IIe's 20-pin cable that pin is
    // ground, and unwired it's pulled down.
    let en35_mask = 1u32 << PIN_EN35;
    let (mut sel_mask, mut sel_level) = (enbl_mask, 0u32);

    let (mut rd_out, mut wp_out) = (rd_hiz, wp_hiz);
    unsafe {
        write_volatile(rd_ctrl, rd_out);
        write_volatile(wp_ctrl, wp_out);
    }

    let mut quarter = 0u8;
    let mut phases = 0u32;
    let mut disk_in = false;
    let mut writable = false;
    // The last PUBLISHED value taken (echoed in PLAYING), and whether its buffer is playing: only
    // while it's for the head's quarter track. The noise buffer plays otherwise.
    let mut seen = 0u32;
    let mut on_track = false;
    // The last track started (the disk's angle), and whether it's what plays (not noise).
    let mut spin = Spin::new();
    let mut on_spin = false;
    let mut motor = false;
    let mut enabled_seen = 0u32;
    let mut pass = 0u32;
    play_noise(&player);

    // Write capture state.
    let mut writing = false;
    let mut closing = false;
    let mut opened_at = 0u32;
    let mut closed_at = 0u32;
    let mut cap_start = 0u32;
    let mut cap_pos = 0u32;
    let mut cap_info = 0u32;
    let mut total = 0u32;

    loop {
        let inp = unsafe { read_volatile(gpio_in) };
        let enabled = inp & sel_mask == sel_level;

        // RD and WRPROT only while enabled. No disk: no pulses, and write-protected.
        let (rd, wp) = if !enabled {
            (rd_hiz, wp_hiz)
        } else {
            (if disk_in { rd_stream } else { rd_high }, if writable { wp_low } else { wp_high })
        };
        if rd != rd_out {
            rd_out = rd;
            unsafe { write_volatile(rd_ctrl, rd_out) };
        }
        if wp != wp_out {
            wp_out = wp;
            unsafe { write_volatile(wp_ctrl, wp_out) };
        }
        enabled_seen |= enabled as u32;

        // The stepper's magnets are only powered while the drive is enabled.
        let ph = if enabled { (inp >> PIN_CA0) & 0b111 | (inp >> (PIN_LSTRB - 3)) & 0b1000 } else { 0 };
        if ph != phases {
            phases = ph;
            PHASES.store(ph as u8, Relaxed);
            let q = a2::step(quarter, ph);
            if q != quarter {
                quarter = q;
                HEAD_QUARTER.store(q, Relaxed);
                STEPS.fetch_add(1, Relaxed);
                // Off the track that was playing: noise until core 0 has this one.
                if on_track && !in_span(CYL_SPAN[cylinder_set(seen)].load(Relaxed), q) {
                    on_track = false;
                    on_spin = false;
                    play_noise(&player);
                }
            }
        }

        pass = pass.wrapping_add(1);
        if pass & 0xF != 0 {
            continue;
        }

        // ---- Housekeeping, every 16th pass ----
        let now = unsafe { read_volatile(timer) };
        if inp & en35_mask != 0 {
            // /3.5DISK is wired (a IIgs): select by it from now on.
            (sel_mask, sel_level) = (enbl_mask | en35_mask, en35_mask);
        }
        // The cell under the head, if a track is playing (the write position).
        let head_cell = if on_spin { spin.cell(now) } else { 0 };

        // Write gate: /WRREQ low while enabled.
        let gate = enabled && inp & wrreq_mask == 0;
        if closing && gate {
            // Open again within the tail: what's captured so far goes now, the rest follows on.
            closing = false;
            writing = true;
            let at = capture.position();
            queue_capture(cap_start, at.wrapping_sub(cap_start) % RING_WORDS as u32, cap_pos, cap_info, &mut total);
            cap_start = at;
            cap_pos = head_cell;
            cap_info = seen | CAP_REST;
            opened_at = now;
        } else if closing && now.wrapping_sub(closed_at) >= A2_CAPTURE_TAIL_US {
            closing = false;
            capture.close(0);
            let words = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            queue_capture(cap_start, words, cap_pos, cap_info, &mut total);
            WRITING.store(false, Release);
        }
        if gate && !writing && !closing {
            if writable && on_track {
                writing = true;
                capture.open(0);
                cap_start = capture.position();
                cap_pos = head_cell;
                cap_info = seen;
                opened_at = now;
                WRITING.store(true, Relaxed);
            }
        } else if !gate && writing {
            writing = false;
            closing = true;
            closed_at = now;
            if now.wrapping_sub(opened_at) > FORMAT_GATE_US {
                cap_info |= CAP_FORMAT; // a whole track: its address fields say what's what
            }
        }
        if writing {
            let so_far = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            CAP_TOTAL.store(total.wrapping_add(so_far), Release);
        }

        // The head's track, once core 0 has published it.
        let published = PUBLISHED.load(Acquire);
        let set = cylinder_set(published);
        let span = CYL_SPAN[set].load(Relaxed);
        let ours = cylinder_track(published).is_some() && in_span(span, quarter);
        if published != seen || ours != on_track {
            let bits = CYL_BITS[set].load(Relaxed);
            if ours && on_track && set == cylinder_set(seen) {
                // The same buffers, for another quarter track that hears them: play on.
            } else if ours && bits == 0 {
                // Nothing recorded here (a WOZ's blank track): noise, the disk's angle kept.
                on_spin = false;
                play_noise(&player);
            } else if ours {
                let cell = if on_spin && on_track && CYL_SPAN[cylinder_set(seen)].load(Relaxed) == span {
                    // Republished after a write: carry on at the cell it's at.
                    head_cell
                } else {
                    spin.start_in(set, now)
                };
                play_track(&player, set, cell);
                spin = Spin::of(set, cell, now);
                on_spin = true;
            } else if on_track {
                on_spin = false;
                play_noise(&player);
            }
            on_track = ours;
            seen = published;
            PLAYING.store(seen, Release);
        } else if on_spin && now.wrapping_sub(spin.at) >= 1 << 31 {
            // Half an hour on one track: count from now, before the timer's difference wraps.
            spin.cell = head_cell;
            spin.at = now;
        }

        disk_in = DISK_IN.load(Relaxed);
        writable = WRITABLE.load(Relaxed) && disk_in;
        // Nothing tells an Apple II a disk changed: it may go in at once.
        if !disk_in {
            MAC_SAW_EMPTY.store(true, Relaxed);
        }
        if enabled_seen != 0 {
            SELECTED.store(true, Relaxed);
        }
        if (enabled_seen != 0) != motor {
            motor = enabled_seen != 0;
            MOTOR_ON.store(motor, Relaxed);
        }
        enabled_seen = 0;
        if pass & 0xFF == 0 {
            LOOPS.store(pass >> 8, Relaxed);
        }
    }
}

// ---- Apple II, both drives ---------------------------------------------------------------------
// In dual mode core 1 runs `run_dual`: drive A, an Apple 3.5 Drive (as `run_a35`), and drive B, a
// Disk II (as `run_a2`), on one port, told apart by /3.5DISK as a IIgs's daisy chain is. The
// computer reads one drive at a time (it has one IWM), so they share the stream and the cylinder
// hand-off (PUBLISHED, CYL_*): core 0 builds the cylinder of the drive it's reading
// (STREAM_WANT), with CYL_DRIVE2 set for drive B. Core 1 answers both drives' status all the time,
// and plays the published buffers while their drive is the one selected, at its cell rate.

/// Drive B's disk is in, and accepts writes (drive A's are [`DISK_IN`] and [`WRITABLE`]). Nothing
/// tells an Apple II a 5.25" disk changed, so core 0 inserts it at once.
pub static B_DISK_IN: AtomicBool = AtomicBool::new(false);
pub static B_WRITABLE: AtomicBool = AtomicBool::new(false);
/// Drive B was selected lately (its motor runs while it is), for the display.
pub static B_MOTOR: AtomicBool = AtomicBool::new(false);
/// The drive the computer is reading, for core 0: 0 none yet, 1 drive A (selected with its motor
/// on: polling it for a disk doesn't count), 2 drive B (selected).
pub static STREAM_WANT: AtomicU8 = AtomicU8::new(0);
/// [`clkdiv_word`]s for drive A and B: PIO0 SM0 for their cells, PIO1 SM0 for their writes. Set
/// by core 0 (drive B's cell when its disk is loaded) before core 1 needs them. HD20 + floppy
/// mode uses `CLKDIV`: 0 the floppy's cells, 1 the HD20's (GCR).
pub static CLKDIV: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
pub static CAP_CLKDIV: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

/// µs per 32-cell word of drive A's 2.0425 µs cells (65.36), for where its disk has turned to.
const A35_WORD_US: u32 = 65;

// What `run_dual` plays: the published buffers for drive A or B, or noise at either's rate.
const S_A: u8 = 1;
const S_B: u8 = 2;
const S_NOISE_A: u8 = 3;
const S_NOISE_B: u8 = 4;

/// Core 1 entry point in dual mode (Apple II 5.25" and 3.5"). Never returns; runs from RAM with
/// interrupts off, like [`run`].
///
/// Selected (/ENBL low, /DRIVE2 high) with /3.5DISK low it's drive A, [`run_a35`]'s Apple 3.5
/// Drive: the register file on RD and WRPROT, LSTRB commands, head select. With /3.5DISK high it's
/// drive B, [`run_a2`]'s Disk II: read pulses on RD, write protect on WRPROT, the phases driving its
/// stepper. Each drive keeps its own state whichever is selected.
///
/// The stream follows the drive selected last: its published buffers if core 0 has built them
/// (drive B's only at its head's quarter track), else noise, at that drive's cell rate
/// ([`CLKDIV`]) and with its program ([`Player::use_program`]): drive A's words autopulled as in
/// `run_a35`, drive B's tracks looping at exactly their cells as in `run_a2`. Coming back to drive
/// A, its track starts where its disk has turned to with [`KEEP_ROTATION`], else from word 0 (as
/// `run` does after a step). Drive B's disk turns on whatever plays: its tracks start at the cell
/// it has turned to ([`Spin`]), as in `run_a2`.
#[unsafe(link_section = ".data.dual_loop")]
#[inline(never)]
pub fn run_dual(mut player: Player, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let timer = unsafe { (*pac::TIMER0::ptr()).timerawl().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };
    // Both pins are PIO0's (`Pulses::Dual` drives both), so the same words fit both.
    let wp_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_WRPROT as usize).gpio_ctrl().as_ptr() };

    let base = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let pulses = base | OE_ENABLE << OEOVER_SHIFT | OUT_NORMAL << OUTOVER_SHIFT;
    let low = base | OE_ENABLE << OEOVER_SHIFT | OUT_LOW << OUTOVER_SHIFT;
    let high = base | OE_ENABLE << OEOVER_SHIFT | OUT_HIGH << OUTOVER_SHIFT;

    // `inp & sel_mask`: drive A with /3.5DISK low, drive B with it high; /ENBL low and /DRIVE2 high
    // for both.
    let sel_mask = 1u32 << PIN_ENBL | 1 << PIN_EN35 | 1 << PIN_DRIVE2;
    let sel_a = 1u32 << PIN_DRIVE2;
    let sel_b = 1u32 << PIN_DRIVE2 | 1 << PIN_EN35;
    let en35_mask = 1u32 << PIN_EN35;
    let lstrb_mask = 1u32 << PIN_LSTRB;
    let wrreq_mask = 1u32 << PIN_WRREQ;

    let (mut rd_out, mut wp_out) = (hiz, hiz);
    unsafe {
        write_volatile(rd_ctrl, rd_out);
        write_volatile(wp_ctrl, wp_out);
    }

    // Drive A, the Apple 3.5 Drive (as in `sony::<true>`).
    let mut track: u8 = 0;
    let mut dir_out = false;
    let mut motor_on = false;
    let mut tach = false;
    let mut tach_next = unsafe { read_volatile(timer) };
    let mut tach_edges = 0u32;
    let mut tach_flutter = 0u32;
    let mut disk_in = false;
    let mut ready = false;
    let mut side = 0u8;
    let mut writable = false;
    let mut switched = false;
    let mut empty_at = tach_next;
    let mut en35_at = tach_next;
    let mut status = status_bits(dir_out, track, motor_on, tach, false, disk_in, ready, false, writable, false, false);
    let mut lstrb_hist = 0u32;

    // Drive B, the Disk II (as in `run_a2`), and what RD and WRPROT show while it's selected.
    let mut quarter = 0u8;
    let mut phases = 0u32;
    let mut b_disk_in = false;
    let mut b_writable = false;
    // The last track of drive B's started (its disk's angle).
    let mut spin = Spin::new();
    let (mut b_rd, mut b_wp) = (high, high);

    // The last PUBLISHED value taken (echoed in PLAYING), what the DMA plays, at what rate, and
    // with whose program (drive B's: `disk2.pio`).
    let mut playing = 0u32;
    let mut stream = S_NOISE_A;
    let mut prog_b = false;
    let mut div = CLKDIV[0].load(Relaxed);
    player.set_clkdiv(div);
    unsafe { player.play(NOISE_PTR.load(Relaxed), NOISE_LEN.load(Relaxed), 0) };
    // Drives selected in the last 16 passes (bit 0 A, bit 1 B); the one selected last (S_A or S_B,
    // or 0); and the one the computer is reading (for STREAM_WANT).
    let mut seen = 0u32;
    let mut last = 0u8;
    let mut want = 0u8;
    let mut pass = 0u32;

    // Write capture state.
    let mut writing = false;
    let mut closing = false;
    let mut opened_at = 0u32;
    let mut closed_at = 0u32;
    let mut cap_start = 0u32;
    let mut cap_pos = 0u32;
    let mut cap_info = 0u32;
    let mut total = 0u32;

    loop {
        let inp = unsafe { read_volatile(gpio_in) };
        let sel = inp & sel_mask;
        let r = (inp >> PIN_SEL) & 0xF;

        // Drive A: the register mux on both pins. Drive B: its pulses and its write protect.
        let (rd, wp) = if sel == sel_a {
            let v = if r & 0b1110 == 8 {
                pulses
            } else if status >> r & 1 != 0 {
                high
            } else {
                low
            };
            (v, v)
        } else if sel == sel_b {
            (b_rd, b_wp)
        } else {
            (hiz, hiz)
        };
        if rd != rd_out {
            rd_out = rd;
            unsafe { write_volatile(rd_ctrl, rd_out) };
        }
        if wp != wp_out {
            wp_out = wp;
            unsafe { write_volatile(wp_ctrl, wp_out) };
        }

        if sel == sel_a {
            seen |= 1;
            // Head select: R8 or R9 with LSTRB low.
            if r & 0b1110 == 8 && inp & lstrb_mask == 0 {
                let s = (r & 1) as u8;
                if s != side {
                    if stream == S_A {
                        let set = cylinder_set(playing);
                        let start = player.spin_start(status & KEEP_ROTATION_BIT != 0, set, side, set, s);
                        player.play_cylinder(set, s, start);
                    }
                    side = s;
                    SIDE.store(side, Relaxed);
                }
            }
        }

        // Drive A's commands: LSTRB low, high, high while it's selected.
        lstrb_hist = (lstrb_hist << 1 | (inp & lstrb_mask != 0) as u32) & 0b111;
        if lstrb_hist == 0b011 && sel == sel_a {
            let (latch, ca2) = (r as u8 & 0b111, r & 0b1000 != 0);
            match latch {
                0b000 => dir_out = ca2,
                0b010 if !ca2 => {
                    track = if dir_out {
                        track.saturating_sub(1)
                    } else {
                        (track + 1).min(crate::track::TRACKS - 1)
                    };
                    ready = false;
                    HEAD_TRACK.store(track, Relaxed);
                    STEPS.fetch_add(1, Relaxed);
                }
                0b100 => {
                    motor_on = !ca2;
                    MOTOR_ON.store(motor_on, Relaxed);
                }
                0b110 if ca2 => {
                    disk_in = false;
                    DISK_IN.store(false, Relaxed);
                    MAC_SAW_EMPTY.store(true, Relaxed);
                    EJECTS.fetch_add(1, Relaxed);
                    switched = true;
                }
                0b001 if ca2 => switched = false,
                _ => {}
            }
            ready &= motor_on;
            let r7 = status >> 7 & 1 != 0;
            let keep = status & KEEP_ROTATION_BIT;
            status = status_bits(dir_out, track, motor_on, r7, false, disk_in, ready, false, writable, false, switched) | keep;
            LAST_CMD.store((latch << 1) | ca2 as u8, Relaxed);
            STROBES.fetch_add(1, Relaxed);
        }

        // Drive B's stepper: its magnets are powered only while it's selected.
        let ph = if sel == sel_b { (inp >> PIN_CA0) & 0b111 | (inp >> (PIN_LSTRB - 3)) & 0b1000 } else { 0 };
        if ph != phases {
            phases = ph;
            PHASES.store(ph as u8, Relaxed);
            let q = a2::step(quarter, ph);
            if q != quarter {
                quarter = q;
                HEAD_QUARTER.store(q, Relaxed);
                STEPS.fetch_add(1, Relaxed);
                // Off the track that was playing: noise until core 0 has this one.
                if stream == S_B && !in_span(CYL_SPAN[cylinder_set(playing)].load(Relaxed), q) {
                    stream = S_NOISE_B;
                    play_noise(&player);
                }
            }
        }
        if sel == sel_b {
            seen |= 2;
        }

        pass = pass.wrapping_add(1);
        if pass & 0xF != 0 {
            continue;
        }

        // ---- Housekeeping, every 16th pass ----
        let now = unsafe { read_volatile(timer) };
        if now.wrapping_sub(tach_next) as i32 >= 0 {
            tach = !tach;
            tach_edges += 1;
            if tach_edges >= 120 {
                tach_edges = 0;
                tach_flutter = if tach_flutter >= 4 { 0 } else { tach_flutter + 1 };
            }
            let zone = (gcr::zone(track) as usize).min(TACH_HALF_US.len() - 1);
            tach_next = now.wrapping_add(TACH_HALF_US[zone] - tach_flutter);
        }

        // Write gate: /WRREQ low while either drive is selected.
        let (on_a, on_b) = (sel == sel_a, sel == sel_b);
        let gate = (on_a || on_b) && inp & wrreq_mask == 0;
        let b_writing = cap_info & CYL_DRIVE2 != 0;
        if closing && gate && on_b && b_writing && stream == S_B {
            // Drive B's gate open again within the tail: the rest follows on, as in `run_a2`.
            closing = false;
            writing = true;
            let at = capture.position();
            queue_capture(cap_start, at.wrapping_sub(cap_start) % RING_WORDS as u32, cap_pos, cap_info, &mut total);
            cap_start = at;
            cap_pos = spin.cell(now);
            cap_info = playing | CAP_REST;
            opened_at = now;
        } else if closing && (gate || now.wrapping_sub(closed_at) >= if b_writing { A2_CAPTURE_TAIL_US } else { CAPTURE_TAIL_US }) {
            closing = false;
            capture.close(0);
            let words = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            queue_capture(cap_start, words, cap_pos, cap_info, &mut total);
            WRITING.store(false, Release);
        }
        if gate && !writing && !closing {
            // Only onto the drive's own track, playing.
            let ok = if on_a { writable && disk_in && stream == S_A } else { b_writable && b_disk_in && stream == S_B };
            if ok {
                writing = true;
                capture.set_gcr_clkdiv(CAP_CLKDIV[on_b as usize].load(Relaxed));
                capture.open(0);
                cap_start = capture.position();
                cap_pos = if on_a {
                    // The cell under the head: just past the start of the word being shifted out.
                    let set = cylinder_set(playing);
                    let len = CYL_LEN[set][side as usize].load(Relaxed);
                    let next = player.next_word(set, side);
                    ((next + len - 1) % len.max(1)) * 32 + 16
                } else {
                    spin.cell(now)
                };
                cap_info = playing | if on_a && side == 1 { CAP_SIDE } else { 0 };
                opened_at = now;
                WRITING.store(true, Relaxed);
            }
        } else if !gate && writing {
            writing = false;
            closing = true;
            closed_at = now;
            if now.wrapping_sub(opened_at) > FORMAT_GATE_US {
                cap_info |= CAP_FORMAT;
                // Drive A's formatter wants sector 0 next (a Disk II's goes by address fields).
                if cap_info & CYL_DRIVE2 == 0 && stream == S_A {
                    player.play_cylinder(cylinder_set(playing), side, 0);
                }
            }
        }
        if writing {
            let so_far = capture.position().wrapping_sub(cap_start) % RING_WORDS as u32;
            CAP_TOTAL.store(total.wrapping_add(so_far), Release);
        }

        // The drive selected last, and the one the computer is reading.
        if on_a {
            last = S_A;
        } else if on_b {
            last = S_B;
        } else if seen & 2 != 0 {
            last = S_B;
        } else if seen & 1 != 0 {
            last = S_A;
        }
        let w = if last == S_B {
            2
        } else if last == S_A && motor_on {
            1
        } else {
            want
        };
        if w != want {
            want = w;
            STREAM_WANT.store(w, Relaxed);
        }

        // What the DMA should play: the selected drive's buffers once core 0 has published them,
        // else noise at its rate.
        let published = PUBLISHED.load(Acquire);
        let fresh = published != playing;
        let valid = cylinder_track(published).is_some();
        let for_b = published & CYL_DRIVE2 != 0;
        let keep = status & KEEP_ROTATION_BIT != 0;
        let set = cylinder_set(published);
        let target = match last {
            S_A if valid && !for_b => S_A,
            S_A => S_NOISE_A,
            // Nothing recorded at its quarter track (a WOZ's blank track): noise, the angle kept.
            S_B if valid && for_b && in_span(CYL_SPAN[set].load(Relaxed), quarter) && CYL_BITS[set].load(Relaxed) != 0 => S_B,
            S_B => S_NOISE_B,
            _ => stream,
        };
        // Drive B's same buffers, for another quarter track that hears them: play on.
        let same_buffers = stream == S_B && target == S_B && set == cylinder_set(playing);
        if target != stream || (fresh && (target == S_A || target == S_B) && !same_buffers) {
            let to_b = target == S_B || target == S_NOISE_B;
            // Drive B's tracks each at their own cell (`play_track` sets it), its noise at the disk's.
            let d = if target == S_B { CYL_CLKDIV[set].load(Relaxed) } else { CLKDIV[to_b as usize].load(Relaxed) };
            if d != div {
                div = d;
                player.set_clkdiv(d);
            }
            let switch = to_b != prog_b;
            if switch {
                player.use_program(to_b);
                prog_b = to_b;
            }
            // Worked out before PLAYING says the old set is free.
            if target == S_A {
                let start = if stream == S_A && same_cylinder(playing, published) {
                    // Republished after a write: carry on where it was.
                    player.next_word(cylinder_set(playing), side)
                } else if stream == S_A {
                    // Its head moved on.
                    player.spin_start(keep, cylinder_set(playing), side, set, side)
                } else if keep {
                    // Back to it: where its disk has turned to.
                    let len = CYL_LEN[set][side as usize].load(Relaxed).max(1);
                    let at = now / A35_WORD_US % len;
                    (at + len - CYL_ROT[set][side as usize].load(Relaxed) % len) % len
                } else {
                    0
                };
                player.play_cylinder(set, side, start);
            } else if target == S_B {
                let cell = if stream == S_B && CYL_SPAN[cylinder_set(playing)].load(Relaxed) == CYL_SPAN[set].load(Relaxed) {
                    // Republished after a write: carry on at the cell it's at.
                    spin.cell(now)
                } else {
                    // Its head moved, or back to it: where its disk has turned to.
                    spin.start_in(set, now)
                };
                play_track(&player, set, cell);
                spin = Spin::of(set, cell, now);
            } else if to_b {
                play_noise(&player);
            } else {
                unsafe { player.play(NOISE_PTR.load(Relaxed), NOISE_LEN.load(Relaxed), 0) };
            }
            if switch && !to_b {
                player.resume();
            }
            stream = target;
        } else if now.wrapping_sub(spin.at) >= 1 << 31 {
            // Half an hour since drive B's track started: count from now, before the timer's
            // difference wraps.
            spin.cell = spin.cell(now);
            spin.at = now;
        }
        if fresh {
            playing = published;
            PLAYING.store(playing, Release);
        }

        // Drive A's disk (as in `sony::<true>`).
        if seen & 1 != 0 {
            // The computer is sensing R1 and it says no disk (see `run`).
            let pins = unsafe { read_volatile(gpio_in) };
            if pins & sel_mask == sel_a && (pins >> PIN_SEL) & 0xF == 1 && !disk_in {
                MAC_SAW_EMPTY.store(true, Relaxed);
            }
        }
        let now_in = DISK_IN.load(Relaxed);
        switched |= disk_in && !now_in;
        if now_in {
            empty_at = now;
        } else if now.wrapping_sub(empty_at) >= A35_EMPTY_US {
            MAC_SAW_EMPTY.store(true, Relaxed);
        }
        if inp & en35_mask == 0 {
            en35_at = now;
        } else if motor_on && now.wrapping_sub(en35_at) >= A35_MOTOR_OFF_US {
            motor_on = false;
            MOTOR_ON.store(false, Relaxed);
        }
        disk_in = now_in;
        writable = WRITABLE.load(Relaxed) && disk_in;
        let mine = cylinder_track(playing).is_some() && playing & CYL_DRIVE2 == 0;
        ready = motor_on && disk_in && mine && cylinder_track(playing) == Some(track);
        status = status_bits(dir_out, track, motor_on, tach, false, disk_in, ready, false, writable, false, switched)
            | if KEEP_ROTATION.load(Relaxed) { KEEP_ROTATION_BIT } else { 0 };

        // Drive B's disk (as in `run_a2`). No disk: no pulses, and write-protected.
        b_disk_in = B_DISK_IN.load(Relaxed);
        b_writable = B_WRITABLE.load(Relaxed) && b_disk_in;
        b_rd = if b_disk_in { pulses } else { high };
        b_wp = if b_writable { low } else { high };

        if seen != 0 {
            SELECTED.store(true, Relaxed);
        }
        if (seen & 2 != 0) != B_MOTOR.load(Relaxed) {
            B_MOTOR.store(seen & 2 != 0, Relaxed);
        }
        seen = 0;
        if pass & 0xFF == 0 {
            LOOPS.store(pass >> 8, Relaxed);
        }
    }
}

// ---- SmartPort -------------------------------------------------------------------------------
// In SmartPort mode core 1 runs `run_smartport` (`smartport.rs` has the protocol). It receives the
// host's packets through the GCR separator (cells pushed 8 at a time, framed as they arrive),
// checks them, does the REQ/ACK handshake, and hands each transaction (a command, and its data
// packet if it has one) to core 0 in SP_MSG; core 0 answers in SP_MSG.tx, which core 1 sends on RD
// a byte per FIFO entry, as for HD20 replies.

/// The units' bus IDs (`81`…), 0 for none yet: core 0 assigns them on INIT, core 1 clears them on
/// a bus reset and goes by them to tell which packets are the units'.
pub static SP_IDS: [AtomicU8; smartport::UNITS] = [const { AtomicU8::new(0) }; smartport::UNITS];
/// Bus resets seen, packets for the units with a bad checksum, transactions the host gave up on.
pub static SP_RESETS: AtomicU32 = AtomicU32::new(0);
pub static SP_BAD: AtomicU32 = AtomicU32::new(0);
pub static SP_ABORTS: AtomicU32 = AtomicU32::new(0);
/// Transactions handed to core 0 (each bump is one, its packets in SP_MSG), and the last one core
/// 0 has answered (its reply in SP_MSG.tx, [`SP_TX_LEN`] bytes; 0 for no reply).
pub static SP_SEQ: AtomicU32 = AtomicU32::new(0);
pub static SP_REPLIED: AtomicU32 = AtomicU32::new(0);
pub static SP_CMD_LEN: AtomicU32 = AtomicU32::new(0);
pub static SP_DATA_LEN: AtomicU32 = AtomicU32::new(0);
pub static SP_TX_LEN: AtomicU32 = AtomicU32::new(0);

/// The packets of a transaction from `C3` to `C8`, and the reply from sync.
pub struct SpMsg {
    pub cmd: [u8; 48],
    pub data: [u8; smartport::MAX_PACKET],
    pub tx: [u8; smartport::MAX_REPLY],
}

pub struct SpBuffers(UnsafeCell<SpMsg>);

// Safety: core 1 writes `cmd`/`data` only while receiving, before bumping SP_SEQ; core 0 copies
// them after it and checks SP_SEQ again (and the checksums). Core 0 writes `tx` before SP_REPLIED,
// and core 1 reads it only after.
unsafe impl Sync for SpBuffers {}

pub static SP_MSG: SpBuffers = SpBuffers(UnsafeCell::new(SpMsg {
    cmd: [0; 48],
    data: [0; smartport::MAX_PACKET],
    tx: [0; smartport::MAX_REPLY],
}));

impl SpBuffers {
    pub fn get(&self) -> *mut SpMsg {
        self.0.get()
    }
}

/// The command in a packet's first data byte (bit 7 from the first top-bits byte), and its DEST.
#[inline(always)]
fn sp_command(p: &[u8]) -> (u8, u8) {
    match (p.get(1), p.get(8), p.get(9)) {
        (Some(&dest), Some(&msb), Some(&b)) => ((msb << 1) & 0x80 | b & 0x7F, dest),
        _ => (0xFF, 0),
    }
}

/// One of the units has bus ID `id`; or with `id` 0, one has none yet (it will take an INIT).
#[inline(always)]
fn sp_ours(id: u8) -> bool {
    let mut found = false;
    for i in &SP_IDS {
        found |= i.load(Relaxed) == id;
    }
    found
}

/// Core 1 entry point in SmartPort mode. Never returns; runs from RAM with interrupts off, like
/// [`run`]. `port` sends replies on RD (PIO0 SM0 at 4 µs cells, a byte per FIFO entry); the GCR
/// separator, at the Apple II's write cell with 8-cell pushes, captures the host's packets.
///
/// The bus is the units' while PH1 and PH3 are high and /3.5DISK isn't low (on a IIgs that means
/// it's talking to its 3.5" drives, whose LSTRB pulses can raise PH3). ACK is WRPROT: low, or
/// high-impedance. RD is driven only while a reply goes out.
#[unsafe(link_section = ".data.sp_loop")]
#[inline(never)]
pub fn run_smartport(port: DcdPort, capture: Capture) -> ! {
    cortex_m::interrupt::disable();

    let gpio_in = unsafe { (*pac::SIO::ptr()).gpio_in().as_ptr() } as *const u32;
    let rd_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_RD as usize).gpio_ctrl().as_ptr() };
    let ack_ctrl = unsafe { (*pac::IO_BANK0::ptr()).gpio(PIN_WRPROT as usize).gpio_ctrl().as_ptr() };

    let base = unsafe { read_volatile(rd_ctrl) } & !OVER_MASK;
    let rd_hiz = base | OE_DISABLE << OEOVER_SHIFT;
    let rd_pulses = base | OE_ENABLE << OEOVER_SHIFT | OUT_NORMAL << OUTOVER_SHIFT;
    let base = unsafe { read_volatile(ack_ctrl) } & !OVER_MASK;
    let ack_up = base | OE_DISABLE << OEOVER_SHIFT;
    let ack_low = base | OE_ENABLE << OEOVER_SHIFT | OUT_LOW << OUTOVER_SHIFT;

    let req_mask = 1u32 << PIN_CA0;
    let en35_mask = 1u32 << PIN_EN35;

    port.bytewise();
    let (mut rd_out, mut ack_out) = (rd_hiz, ack_up);
    unsafe {
        write_volatile(rd_ctrl, rd_out);
        write_volatile(ack_ctrl, ack_out);
    }

    let msg = SP_MSG.get();
    let mut link = Handshake::new();
    let mut framer = Framer::new();
    // The next ring word to frame (each holds 8 cells in its low byte).
    let mut rx_word = 0u32;
    let mut seq = SP_SEQ.load(Relaxed);
    let mut resetting = false;
    // The reply being sent: its length and the bytes queued so far.
    let mut send_len = 0u32;
    let mut queued = 0u32;
    let mut pass = 0u32;
    let mut seen = 0u32;

    loop {
        let inp = unsafe { read_volatile(gpio_in) };
        let ph = (inp >> PIN_CA0) & 0b111 | (inp >> (PIN_LSTRB - 3)) & 0b1000;
        let enabled = ph & 0b1010 == 0b1010 && inp & en35_mask != 0;
        let req = inp & req_mask != 0;

        // A bus reset (PH0 and PH2 high, PH1 and PH3 low): every ID is forgotten.
        if ph == 0b0101 {
            if !resetting {
                resetting = true;
                for i in &SP_IDS {
                    i.store(0, Relaxed);
                }
                SP_RESETS.fetch_add(1, Relaxed);
            }
        } else {
            resetting = false;
        }

        // Frame what has come in, and see whether it's a packet for the units.
        let mut rx = Rx::Nothing;
        let receiving = matches!(link.phase, Phase::RxCommand | Phase::RxData);
        if receiving {
            let command = link.phase == Phase::RxCommand;
            let buf: &mut [u8] = unsafe { if command { &mut (*msg).cmd } else { &mut (*msg).data } };
            let at = capture.position();
            let mut done = Framed::More;
            while rx_word != at && done == Framed::More {
                let w = RING.word(rx_word as usize);
                rx_word = (rx_word + 1) % RING_WORDS as u32;
                let mut k = 8;
                while k > 0 && done == Framed::More {
                    k -= 1;
                    done = framer.cell(w >> k & 1 != 0, buf);
                }
            }
            rx = match done {
                Framed::Packet(n) => {
                    let p = &buf[..n.min(buf.len())];
                    let (code, dest) = sp_command(p);
                    // An INIT is for whichever unit has no ID yet.
                    let ours = if command && code == smartport::INIT { sp_ours(0) } else { sp_ours(dest) };
                    if !ours {
                        Rx::NotOurs
                    } else if !smartport::checksum_ok(p, !command) {
                        SP_BAD.fetch_add(1, Relaxed);
                        Rx::NotOurs
                    } else {
                        if command {
                            SP_CMD_LEN.store(n as u32, Relaxed);
                            SP_DATA_LEN.store(0, Relaxed);
                        } else {
                            SP_DATA_LEN.store(n as u32, Relaxed);
                        }
                        Rx::Ours { data_follows: command && smartport::has_data(code) }
                    }
                }
                Framed::Bad => Rx::NotOurs,
                Framed::More => Rx::Nothing,
            };
        }
        let replied = (link.phase == Phase::Busy && SP_REPLIED.load(Acquire) == seq).then(|| {
            send_len = SP_TX_LEN.load(Relaxed).min(smartport::MAX_REPLY as u32);
            send_len > 0
        });
        let sent = link.phase == Phase::Sending && queued == send_len && port.stalled();

        let step = link.step(enabled, req, rx, replied, sent);
        if step.close {
            capture.close(0);
        }
        if step.open {
            capture.open(0);
            rx_word = capture.position();
            framer = Framer::new();
        }
        if step.publish {
            seq = seq.wrapping_add(1);
            SP_SEQ.store(seq, Release);
        }
        if step.flush {
            port.flush();
        }
        if step.abort {
            SP_ABORTS.fetch_add(1, Relaxed);
        }
        let rd = if step.driving { rd_pulses } else { rd_hiz };
        if rd != rd_out {
            rd_out = rd;
            unsafe { write_volatile(rd_ctrl, rd_out) };
        }
        if step.send {
            // The reply, a byte per FIFO entry. Once the state machine has taken the first, a
            // stall means the last is out.
            port.push(unsafe { (*msg).tx[0] });
            while !port.empty() {}
            port.clear_stall();
            queued = 1;
        }
        if link.phase == Phase::Sending {
            while queued < send_len && !port.full() {
                port.push(unsafe { (*msg).tx[queued as usize % smartport::MAX_REPLY] });
                queued += 1;
            }
        }
        let ack = if step.ack_low { ack_low } else { ack_up };
        if ack != ack_out {
            ack_out = ack;
            unsafe { write_volatile(ack_ctrl, ack_out) };
        }
        seen |= enabled as u32;

        pass = pass.wrapping_add(1);
        if pass & 0xFF == 0 {
            if seen != 0 {
                SELECTED.store(true, Relaxed);
                seen = 0;
            }
            LOOPS.store(pass >> 8, Relaxed);
        }
    }
}
