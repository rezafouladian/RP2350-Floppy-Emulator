//! Host tool: an Apple IIe with a Disk II card in slot 6, booting a 5.25" WOZ, to see what a
//! disk's loader does with the drive this firmware makes.
//!
//! Two drives:
//! - `ideal`: the usual rules for emulators: the head's bit position carried across
//!   tracks to the bit, a track there the moment the head is, the MC3470's fake bits, INFO's
//!   optimal bit timing.
//! - `fw`: this firmware's Disk II, built from its own code (`a2`, `moof`, `weak`, `woz`): each
//!   track rotated into its longest gap and looped at exactly its cells (`src/disk2.pio`), weak
//!   runs randomized every 50 ms, core 0 taking a while to read and publish a track after a step
//!   (the SD read, a piece of the OLED's screen), and core 1's rules (`drive::run_a2`): noise off
//!   the published track's quarter tracks, a new track started at the cell the disk has turned
//!   to (`drive::Spin`, its integer arithmetic), at the PIO's real divided clock.
//!
//! A disk that boots on `ideal` and not on `fw` is failing on something the firmware does. (Not
//! the other way round always: `ideal` plays Border Zone at its 3.5 µs optimal timing, which a
//! real Apple II's boot ROM can't read.)
//!
//! The machine: an NMOS 6502 with cycle counts (each instruction's data access on its last
//! cycle), the original IIe's ROMs, main and aux RAM with the language card, the IIe soft
//! switches, a keyboard, paddles at mid-range. The Disk II's sequencer is the P6 PROM's own table
//! stepped at 2 MHz (two steps a CPU cycle), its boot ROM the P5's. Writes aren't emulated. ROM
//! paths are below; the screen is saved as a PNG.
//!
//! `rustc --edition 2024 -O -A dead_code tools/a2emu.rs -o target/a2emu`
//! `target/a2emu <disk.woz> [--drive ideal|fw] [--secs N] [--key SECS:TEXT]... [--png FILE]
//!  [--trace FILE] [--dump HEXADDR:HEXLEN] [--sd-ms N] [--rotation-ms N] [--display-ms N]
//!  [--no-span] [--weak8] [--no-weak] [--cell-ns N]`
//! `--key` text takes `\r` and `\xNN`; keys go in 0.3 s apart.

#[path = "../src/a2r.rs"]
mod a2r;
#[path = "../src/a2.rs"]
mod a2;
#[path = "../src/gcr.rs"]
mod gcr;
#[path = "../src/mfm.rs"]
mod mfm;
#[path = "../src/moof.rs"]
mod moof;
#[path = "../src/track.rs"]
mod track;
#[path = "../src/weak.rs"]
mod weak;
#[path = "../src/woz.rs"]
mod woz;
#[path = "../src/writes.rs"]
mod writes;

use std::collections::VecDeque;
use track::{BLOCK, MAX_TRACK_BLOCKS, MAX_TRACK_WORDS, TrackBuf};

const ROM_CD: &str = "~/Downloads/A2_ROMsCollection/3420135A.BIN";
const ROM_EF: &str = "~/Downloads/A2_ROMsCollection/3420134A.BIN";
const ROM_P5: &str = "~/Downloads/diskii/Apple Disk II 16 Sector Interface Card ROM P5 - 341-0027.bin";
const ROM_P6: &str = "~/Downloads/diskii/Apple Disk II 16 Sector Interface Card ROM P6 - 341-0028.bin";

/// 14.31818 MHz / 14 (the occasional long cycle ignored); the sequencer runs at twice it.
const CPU_HZ: f64 = 1_022_727.0;
const TICK_S: f64 = 1.0 / (2.0 * CPU_HZ);

fn path(p: &str) -> String {
    p.replacen('~', &std::env::var("HOME").unwrap(), 1)
}

// ---- 6502 ---------------------------------------------------------------------------------------

const FC: u8 = 0x01;
const FZ: u8 = 0x02;
const FI: u8 = 0x04;
const FD: u8 = 0x08;
const FB: u8 = 0x10;
const FU: u8 = 0x20;
const FV: u8 = 0x40;
const FN: u8 = 0x80;

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Imp,
    Acc,
    Imm,
    Zp,
    Zpx,
    Zpy,
    Izx,
    Izy,
    Abs,
    Abx,
    Aby,
    Ind,
    Rel,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Op {
    Adc, And, Asl, Bcc, Bcs, Beq, Bit, Bmi, Bne, Bpl, Brk, Bvc, Bvs, Clc, Cld, Cli, Clv, Cmp, Cpx,
    Cpy, Dec, Dex, Dey, Eor, Inc, Inx, Iny, Jmp, Jsr, Lda, Ldx, Ldy, Lsr, Nop, Ora, Pha, Php, Pla,
    Plp, Rol, Ror, Rti, Rts, Sbc, Sec, Sed, Sei, Sta, Stx, Sty, Tax, Tay, Tsx, Txa, Txs, Tya,
    // Undocumented, as loaders use them.
    Lax, Sax, Dcp, Isc, Slo, Rla, Sre, Rra, Anc, Alr, Arr, Jam,
}

#[derive(Clone, Copy)]
struct Instr {
    op: Op,
    mode: Mode,
    cycles: u8,
    /// One more cycle when indexing crosses a page.
    page: bool,
}

fn instr_table() -> [Instr; 256] {
    use Mode::*;
    use Op::*;
    let mut t = [Instr { op: Jam, mode: Imp, cycles: 2, page: false }; 256];
    let mut set = |code: u8, op: Op, mode: Mode, cycles: u8, page: bool| t[code as usize] = Instr { op, mode, cycles, page };
    // The eight addressing modes of the ALU group, per opcode row.
    let alu = |set: &mut dyn FnMut(u8, Op, Mode, u8, bool), base: u8, op: Op, store: bool| {
        set(base + 0x09, op, Imm, 2, false);
        set(base + 0x05, op, Zp, 3, false);
        set(base + 0x15, op, Zpx, 4, false);
        set(base + 0x0D, op, Abs, 4, false);
        set(base + 0x1D, op, Abx, if store { 5 } else { 4 }, !store);
        set(base + 0x19, op, Aby, if store { 5 } else { 4 }, !store);
        set(base + 0x01, op, Izx, 6, false);
        set(base + 0x11, op, Izy, if store { 6 } else { 5 }, !store);
    };
    alu(&mut set, 0x00, Ora, false);
    alu(&mut set, 0x20, And, false);
    alu(&mut set, 0x40, Eor, false);
    alu(&mut set, 0x60, Adc, false);
    alu(&mut set, 0x80, Sta, true);
    alu(&mut set, 0xA0, Lda, false);
    alu(&mut set, 0xC0, Cmp, false);
    alu(&mut set, 0xE0, Sbc, false);
    t[0x89] = Instr { op: Nop, mode: Imm, cycles: 2, page: false };
    let mut set = |code: u8, op: Op, mode: Mode, cycles: u8, page: bool| t[code as usize] = Instr { op, mode, cycles, page };
    // Shifts and INC/DEC.
    for (base, op) in [(0x00u8, Asl), (0x20, Rol), (0x40, Lsr), (0x60, Ror)] {
        set(base + 0x0A, op, Acc, 2, false);
        set(base + 0x06, op, Zp, 5, false);
        set(base + 0x16, op, Zpx, 6, false);
        set(base + 0x0E, op, Abs, 6, false);
        set(base + 0x1E, op, Abx, 7, false);
    }
    for (base, op) in [(0xC0u8, Dec), (0xE0, Inc)] {
        set(base + 0x06, op, Zp, 5, false);
        set(base + 0x16, op, Zpx, 6, false);
        set(base + 0x0E, op, Abs, 6, false);
        set(base + 0x1E, op, Abx, 7, false);
    }
    for (code, op) in [(0x10u8, Bpl), (0x30, Bmi), (0x50, Bvc), (0x70, Bvs), (0x90, Bcc), (0xB0, Bcs), (0xD0, Bne), (0xF0, Beq)] {
        set(code, op, Rel, 2, false);
    }
    set(0x24, Bit, Zp, 3, false);
    set(0x2C, Bit, Abs, 4, false);
    set(0x00, Brk, Imp, 7, false);
    for (code, op) in [
        (0x18u8, Clc), (0xD8, Cld), (0x58, Cli), (0xB8, Clv), (0x38, Sec), (0xF8, Sed), (0x78, Sei), (0xCA, Dex),
        (0x88, Dey), (0xE8, Inx), (0xC8, Iny), (0xAA, Tax), (0xA8, Tay), (0xBA, Tsx), (0x8A, Txa), (0x9A, Txs),
        (0x98, Tya), (0xEA, Nop),
    ] {
        set(code, op, Imp, 2, false);
    }
    set(0xE0, Cpx, Imm, 2, false);
    set(0xE4, Cpx, Zp, 3, false);
    set(0xEC, Cpx, Abs, 4, false);
    set(0xC0, Cpy, Imm, 2, false);
    set(0xC4, Cpy, Zp, 3, false);
    set(0xCC, Cpy, Abs, 4, false);
    set(0x4C, Jmp, Abs, 3, false);
    set(0x6C, Jmp, Ind, 5, false);
    set(0x20, Jsr, Abs, 6, false);
    set(0xA2, Ldx, Imm, 2, false);
    set(0xA6, Ldx, Zp, 3, false);
    set(0xB6, Ldx, Zpy, 4, false);
    set(0xAE, Ldx, Abs, 4, false);
    set(0xBE, Ldx, Aby, 4, true);
    set(0xA0, Ldy, Imm, 2, false);
    set(0xA4, Ldy, Zp, 3, false);
    set(0xB4, Ldy, Zpx, 4, false);
    set(0xAC, Ldy, Abs, 4, false);
    set(0xBC, Ldy, Abx, 4, true);
    set(0x48, Pha, Imp, 3, false);
    set(0x08, Php, Imp, 3, false);
    set(0x68, Pla, Imp, 4, false);
    set(0x28, Plp, Imp, 4, false);
    set(0x40, Rti, Imp, 6, false);
    set(0x60, Rts, Imp, 6, false);
    set(0x86, Stx, Zp, 3, false);
    set(0x96, Stx, Zpy, 4, false);
    set(0x8E, Stx, Abs, 4, false);
    set(0x84, Sty, Zp, 3, false);
    set(0x94, Sty, Zpx, 4, false);
    set(0x8C, Sty, Abs, 4, false);
    // Undocumented.
    for code in [0x1Au8, 0x3A, 0x5A, 0x7A, 0xDA, 0xFA] {
        set(code, Nop, Imp, 2, false);
    }
    for code in [0x80u8, 0x82, 0xC2, 0xE2] {
        set(code, Nop, Imm, 2, false);
    }
    for code in [0x04u8, 0x44, 0x64] {
        set(code, Nop, Zp, 3, false);
    }
    for code in [0x14u8, 0x34, 0x54, 0x74, 0xD4, 0xF4] {
        set(code, Nop, Zpx, 4, false);
    }
    set(0x0C, Nop, Abs, 4, false);
    for code in [0x1Cu8, 0x3C, 0x5C, 0x7C, 0xDC, 0xFC] {
        set(code, Nop, Abx, 4, true);
    }
    set(0xEB, Sbc, Imm, 2, false);
    set(0xA7, Lax, Zp, 3, false);
    set(0xB7, Lax, Zpy, 4, false);
    set(0xAF, Lax, Abs, 4, false);
    set(0xBF, Lax, Aby, 4, true);
    set(0xA3, Lax, Izx, 6, false);
    set(0xB3, Lax, Izy, 5, true);
    set(0x87, Sax, Zp, 3, false);
    set(0x97, Sax, Zpy, 4, false);
    set(0x8F, Sax, Abs, 4, false);
    set(0x83, Sax, Izx, 6, false);
    for (base, op) in [(0x00u8, Slo), (0x20, Rla), (0x40, Sre), (0x60, Rra), (0xC0, Dcp), (0xE0, Isc)] {
        set(base + 0x07, op, Zp, 5, false);
        set(base + 0x17, op, Zpx, 6, false);
        set(base + 0x0F, op, Abs, 6, false);
        set(base + 0x1F, op, Abx, 7, false);
        set(base + 0x1B, op, Aby, 7, false);
        set(base + 0x03, op, Izx, 8, false);
        set(base + 0x13, op, Izy, 8, false);
    }
    set(0x0B, Anc, Imm, 2, false);
    set(0x2B, Anc, Imm, 2, false);
    set(0x4B, Alr, Imm, 2, false);
    set(0x6B, Arr, Imm, 2, false);
    t
}

trait Bus {
    /// A read or write at CPU cycle `cyc` (the instruction's last cycle).
    fn rd(&mut self, a: u16, cyc: u64) -> u8;
    fn wr(&mut self, a: u16, v: u8, cyc: u64);
}

struct Cpu {
    a: u8,
    x: u8,
    y: u8,
    s: u8,
    p: u8,
    pc: u16,
    cyc: u64,
    table: [Instr; 256],
}

impl Cpu {
    fn new() -> Self {
        Cpu { a: 0, x: 0, y: 0, s: 0xFF, p: FU | FI, pc: 0, cyc: 0, table: instr_table() }
    }

    fn reset(&mut self, b: &mut impl Bus) {
        self.s = 0xFD;
        self.p = FU | FI;
        self.pc = u16::from_le_bytes([b.rd(0xFFFC, self.cyc), b.rd(0xFFFD, self.cyc)]);
    }

    fn nz(&mut self, v: u8) {
        self.p = self.p & !(FN | FZ) | v & FN | if v == 0 { FZ } else { 0 };
    }

    fn push(&mut self, b: &mut impl Bus, v: u8) {
        b.wr(0x100 | self.s as u16, v, self.cyc);
        self.s = self.s.wrapping_sub(1);
    }

    fn pull(&mut self, b: &mut impl Bus) -> u8 {
        self.s = self.s.wrapping_add(1);
        b.rd(0x100 | self.s as u16, self.cyc)
    }

    fn fetch(&mut self, b: &mut impl Bus) -> u8 {
        let v = b.rd(self.pc, self.cyc);
        self.pc = self.pc.wrapping_add(1);
        v
    }

    fn adc(&mut self, v: u8) {
        let c = (self.p & FC) as u16;
        if self.p & FD != 0 {
            let bin = (self.a as u16 + v as u16 + c) as u8;
            let mut lo = (self.a & 0x0F) as u16 + (v & 0x0F) as u16 + c;
            let mut hi = (self.a >> 4) as u16 + (v >> 4) as u16;
            if lo > 9 {
                lo += 6;
            }
            if lo > 0x0F {
                hi += 1;
            }
            let mid = ((hi << 4) | (lo & 0x0F)) as u8;
            self.p &= !(FN | FZ | FV | FC);
            self.p |= mid & FN | if bin == 0 { FZ } else { 0 };
            if !(self.a ^ v) & (self.a ^ mid) & 0x80 != 0 {
                self.p |= FV;
            }
            if hi > 9 {
                hi += 6;
            }
            if hi > 0x0F {
                self.p |= FC;
            }
            self.a = ((hi << 4) | (lo & 0x0F)) as u8;
        } else {
            let r = self.a as u16 + v as u16 + c;
            let r8 = r as u8;
            self.p &= !(FV | FC);
            if !(self.a ^ v) & (self.a ^ r8) & 0x80 != 0 {
                self.p |= FV;
            }
            if r > 0xFF {
                self.p |= FC;
            }
            self.a = r8;
            self.nz(r8);
        }
    }

    fn sbc(&mut self, v: u8) {
        let borrow = 1 - (self.p & FC) as i16;
        let r = self.a as i16 - v as i16 - borrow;
        let r8 = r as u8;
        let a = self.a;
        self.p &= !(FV | FC);
        if (a ^ v) & (a ^ r8) & 0x80 != 0 {
            self.p |= FV;
        }
        if r >= 0 {
            self.p |= FC;
        }
        self.nz(r8);
        if self.p & FD != 0 {
            let mut lo = (a & 0x0F) as i16 - (v & 0x0F) as i16 - borrow;
            let mut hi = (a >> 4) as i16 - (v >> 4) as i16;
            if lo < 0 {
                lo -= 6;
                hi -= 1;
            }
            if hi < 0 {
                hi -= 6;
            }
            self.a = ((hi << 4) | (lo & 0x0F)) as u8;
        } else {
            self.a = r8;
        }
    }

    fn cmp(&mut self, r: u8, v: u8) {
        let d = r.wrapping_sub(v);
        self.p = self.p & !FC | if r >= v { FC } else { 0 };
        self.nz(d);
    }

    /// One instruction. Returns false on a JAM.
    fn step(&mut self, b: &mut impl Bus) -> bool {
        use Mode::*;
        use Op::*;
        let start_pc = self.pc;
        let code = self.fetch(b);
        let ins = self.table[code as usize];
        let mut cycles = ins.cycles as u64;
        // The operand's address (and whether indexing crossed a page).
        let (addr, cross) = match ins.mode {
            Imp | Acc => (0u16, false),
            Imm => {
                let a = self.pc;
                self.pc = self.pc.wrapping_add(1);
                (a, false)
            }
            Zp => (self.fetch(b) as u16, false),
            Zpx => (self.fetch(b).wrapping_add(self.x) as u16, false),
            Zpy => (self.fetch(b).wrapping_add(self.y) as u16, false),
            Abs => {
                let lo = self.fetch(b);
                let hi = self.fetch(b);
                (u16::from_le_bytes([lo, hi]), false)
            }
            Abx | Aby => {
                let lo = self.fetch(b);
                let hi = self.fetch(b);
                let base = u16::from_le_bytes([lo, hi]);
                let a = base.wrapping_add(if ins.mode == Abx { self.x } else { self.y } as u16);
                (a, a & 0xFF00 != base & 0xFF00)
            }
            Izx => {
                let z = self.fetch(b).wrapping_add(self.x);
                let lo = b.rd(z as u16, self.cyc);
                let hi = b.rd(z.wrapping_add(1) as u16, self.cyc);
                (u16::from_le_bytes([lo, hi]), false)
            }
            Izy => {
                let z = self.fetch(b);
                let lo = b.rd(z as u16, self.cyc);
                let hi = b.rd(z.wrapping_add(1) as u16, self.cyc);
                let base = u16::from_le_bytes([lo, hi]);
                let a = base.wrapping_add(self.y as u16);
                (a, a & 0xFF00 != base & 0xFF00)
            }
            Ind => {
                let lo = self.fetch(b);
                let hi = self.fetch(b);
                let p = u16::from_le_bytes([lo, hi]);
                let q = (p & 0xFF00) | (p.wrapping_add(1) & 0x00FF);
                (u16::from_le_bytes([b.rd(p, self.cyc), b.rd(q, self.cyc)]), false)
            }
            Rel => {
                let off = self.fetch(b) as i8;
                (self.pc.wrapping_add(off as u16), false)
            }
        };
        if ins.page && cross {
            cycles += 1;
        }
        // The data access happens on the instruction's last cycle.
        let at = self.cyc + cycles - 1;
        let mut ok = true;
        macro_rules! load {
            () => {
                b.rd(addr, at)
            };
        }
        macro_rules! rmw {
            ($f:expr) => {{
                if ins.mode == Acc {
                    let v = $f(self, self.a);
                    self.a = v;
                    v
                } else {
                    let old = b.rd(addr, at);
                    b.wr(addr, old, at);
                    let v = $f(self, old);
                    b.wr(addr, v, at);
                    v
                }
            }};
        }
        let asl = |c: &mut Cpu, v: u8| {
            c.p = c.p & !FC | v >> 7;
            let r = v << 1;
            c.nz(r);
            r
        };
        let lsr = |c: &mut Cpu, v: u8| {
            c.p = c.p & !FC | v & 1;
            let r = v >> 1;
            c.nz(r);
            r
        };
        let rol = |c: &mut Cpu, v: u8| {
            let r = v << 1 | c.p & FC;
            c.p = c.p & !FC | v >> 7;
            c.nz(r);
            r
        };
        let ror = |c: &mut Cpu, v: u8| {
            let r = v >> 1 | (c.p & FC) << 7;
            c.p = c.p & !FC | v & 1;
            c.nz(r);
            r
        };
        let branch = |c: &mut Cpu, cond: bool, cycles: &mut u64| {
            if cond {
                *cycles += 1;
                if c.pc & 0xFF00 != addr & 0xFF00 {
                    *cycles += 1;
                }
                c.pc = addr;
            }
        };
        match ins.op {
            Adc => {
                let v = load!();
                self.adc(v)
            }
            Sbc => {
                let v = load!();
                self.sbc(v)
            }
            And => {
                self.a &= load!();
                self.nz(self.a)
            }
            Ora => {
                self.a |= load!();
                self.nz(self.a)
            }
            Eor => {
                self.a ^= load!();
                self.nz(self.a)
            }
            Lda => {
                self.a = load!();
                self.nz(self.a)
            }
            Ldx => {
                self.x = load!();
                self.nz(self.x)
            }
            Ldy => {
                self.y = load!();
                self.nz(self.y)
            }
            Lax => {
                self.a = load!();
                self.x = self.a;
                self.nz(self.a)
            }
            Sta => b.wr(addr, self.a, at),
            Stx => b.wr(addr, self.x, at),
            Sty => b.wr(addr, self.y, at),
            Sax => b.wr(addr, self.a & self.x, at),
            Cmp => {
                let v = load!();
                self.cmp(self.a, v)
            }
            Cpx => {
                let v = load!();
                self.cmp(self.x, v)
            }
            Cpy => {
                let v = load!();
                self.cmp(self.y, v)
            }
            Bit => {
                let v = load!();
                self.p = self.p & !(FN | FV | FZ) | v & (FN | FV) | if self.a & v == 0 { FZ } else { 0 };
            }
            Asl => {
                rmw!(asl);
            }
            Lsr => {
                rmw!(lsr);
            }
            Rol => {
                rmw!(rol);
            }
            Ror => {
                rmw!(ror);
            }
            Inc => {
                rmw!(|c: &mut Cpu, v: u8| {
                    let r = v.wrapping_add(1);
                    c.nz(r);
                    r
                });
            }
            Dec => {
                rmw!(|c: &mut Cpu, v: u8| {
                    let r = v.wrapping_sub(1);
                    c.nz(r);
                    r
                });
            }
            Slo => {
                let r = rmw!(asl);
                self.a |= r;
                self.nz(self.a)
            }
            Rla => {
                let r = rmw!(rol);
                self.a &= r;
                self.nz(self.a)
            }
            Sre => {
                let r = rmw!(lsr);
                self.a ^= r;
                self.nz(self.a)
            }
            Rra => {
                let r = rmw!(ror);
                self.adc(r)
            }
            Dcp => {
                let r = rmw!(|_c: &mut Cpu, v: u8| v.wrapping_sub(1));
                self.cmp(self.a, r)
            }
            Isc => {
                let r = rmw!(|_c: &mut Cpu, v: u8| v.wrapping_add(1));
                self.sbc(r)
            }
            Anc => {
                self.a &= load!();
                self.nz(self.a);
                self.p = self.p & !FC | self.a >> 7;
            }
            Alr => {
                self.a &= load!();
                self.a = lsr(self, self.a);
            }
            Arr => {
                self.a &= load!();
                self.a = self.a >> 1 | (self.p & FC) << 7;
                self.nz(self.a);
                self.p = self.p & !(FC | FV) | (self.a >> 6) & 1 | ((self.a >> 6 ^ self.a >> 5) & 1) << 6;
            }
            Bcc => branch(self, self.p & FC == 0, &mut cycles),
            Bcs => branch(self, self.p & FC != 0, &mut cycles),
            Beq => branch(self, self.p & FZ != 0, &mut cycles),
            Bne => branch(self, self.p & FZ == 0, &mut cycles),
            Bmi => branch(self, self.p & FN != 0, &mut cycles),
            Bpl => branch(self, self.p & FN == 0, &mut cycles),
            Bvc => branch(self, self.p & FV == 0, &mut cycles),
            Bvs => branch(self, self.p & FV != 0, &mut cycles),
            Brk => {
                let ret = self.pc.wrapping_add(1);
                self.push(b, (ret >> 8) as u8);
                self.push(b, ret as u8);
                self.push(b, self.p | FB | FU);
                self.p |= FI;
                self.pc = u16::from_le_bytes([b.rd(0xFFFE, at), b.rd(0xFFFF, at)]);
            }
            Clc => self.p &= !FC,
            Cld => self.p &= !FD,
            Cli => self.p &= !FI,
            Clv => self.p &= !FV,
            Sec => self.p |= FC,
            Sed => self.p |= FD,
            Sei => self.p |= FI,
            Dex => {
                self.x = self.x.wrapping_sub(1);
                self.nz(self.x)
            }
            Dey => {
                self.y = self.y.wrapping_sub(1);
                self.nz(self.y)
            }
            Inx => {
                self.x = self.x.wrapping_add(1);
                self.nz(self.x)
            }
            Iny => {
                self.y = self.y.wrapping_add(1);
                self.nz(self.y)
            }
            Tax => {
                self.x = self.a;
                self.nz(self.x)
            }
            Tay => {
                self.y = self.a;
                self.nz(self.y)
            }
            Tsx => {
                self.x = self.s;
                self.nz(self.x)
            }
            Txa => {
                self.a = self.x;
                self.nz(self.a)
            }
            Txs => self.s = self.x,
            Tya => {
                self.a = self.y;
                self.nz(self.a)
            }
            Nop => {
                if ins.mode != Imp && ins.mode != Imm {
                    let _ = load!();
                }
            }
            Jmp => self.pc = addr,
            Jsr => {
                let ret = self.pc.wrapping_sub(1);
                self.push(b, (ret >> 8) as u8);
                self.push(b, ret as u8);
                self.pc = addr;
            }
            Rts => {
                let lo = self.pull(b);
                let hi = self.pull(b);
                self.pc = u16::from_le_bytes([lo, hi]).wrapping_add(1);
            }
            Rti => {
                self.p = self.pull(b) | FU;
                let lo = self.pull(b);
                let hi = self.pull(b);
                self.pc = u16::from_le_bytes([lo, hi]);
            }
            Pha => self.push(b, self.a),
            Php => self.push(b, self.p | FB | FU),
            Pla => {
                self.a = self.pull(b);
                self.nz(self.a)
            }
            Plp => self.p = self.pull(b) | FU,
            Jam => {
                self.pc = start_pc;
                ok = false;
            }
        }
        self.cyc += cycles;
        ok
    }
}

// ---- Disk II card -------------------------------------------------------------------------------

/// A drive on the Disk II's cable.
trait Drive {
    /// The card's motor (drive enable) and phases changed, at `t` seconds. Phases are only
    /// powered while enabled.
    fn inputs(&mut self, t: f64, enabled: bool, phases: u32);
    /// Is a read pulse on RD at `t` (called with `t` rising, every sequencer step)?
    fn pulse(&mut self, t: f64) -> bool;
    fn write_protected(&self) -> bool;
    fn quarter(&self) -> u8;
    /// Something to say about the drive, for the report.
    fn note(&self) -> String {
        String::new()
    }
}

struct Disk2 {
    /// P6's table with its address and data lines put in order (see `~/Downloads/diskii`).
    p6: [u8; 256],
    state: u8,
    reg: u8,
    bus: u8,
    q6: bool,
    q7: bool,
    phases: u32,
    motor: bool,
    /// The motor stays on ~1 s after it's turned off (the card's 556).
    motor_off_tick: Option<u64>,
    drive2: bool,
    tick: u64,
    drive: Box<dyn Drive>,
    /// Head positions over time: (seconds, quarter).
    trace: Vec<(f64, u8)>,
}

impl Disk2 {
    fn new(p6_file: &[u8], drive: Box<dyn Drive>) -> Self {
        // file address: S3 S2 S1 /RP Q7 Q6 MSB S0; data: NS3 NS2 NS0 NS1 cmd.
        let mut p6 = [0u8; 256];
        for state in 0..16u8 {
            for rp in 0..2u8 {
                for q7 in 0..2u8 {
                    for q6 in 0..2u8 {
                        for msb in 0..2u8 {
                            let file = (state & 0xE) << 4 | (1 - rp) << 4 | q7 << 3 | q6 << 2 | msb << 1 | (state & 1);
                            let d = p6_file[file as usize];
                            let next = (d & 0xC0) >> 4 | (d & 0x10) >> 3 | (d & 0x20) >> 5;
                            p6[(state << 4 | rp << 3 | q7 << 2 | q6 << 1 | msb) as usize] = next << 4 | d & 0x0F;
                        }
                    }
                }
            }
        }
        Disk2 { p6, state: 0, reg: 0, bus: 0, q6: false, q7: false, phases: 0, motor: false, motor_off_tick: None, drive2: false, tick: 0, drive, trace: vec![(0.0, 0)] }
    }

    fn enabled(&self, tick: u64) -> bool {
        !self.drive2 && (self.motor || self.motor_off_tick.is_some_and(|t| ((tick - t) as f64) * TICK_S < 1.0))
    }

    fn catch_up(&mut self, to: u64) {
        while self.tick < to {
            let t = self.tick as f64 * TICK_S;
            let on = self.enabled(self.tick);
            let rp = self.drive.pulse(t) && on;
            let wp = self.drive.write_protected() as u8;
            let addr = self.state << 4 | (rp as u8) << 3 | (self.q7 as u8) << 2 | (self.q6 as u8) << 1 | self.reg >> 7;
            let d = self.p6[addr as usize];
            let cmd = d & 0x0F;
            if cmd & 8 == 0 {
                self.reg = 0;
            } else {
                match cmd & 3 {
                    1 => self.reg = self.reg << 1 | (cmd >> 2) & 1,
                    2 => self.reg = self.reg >> 1 | wp << 7,
                    3 => self.reg = self.bus,
                    _ => {}
                }
            }
            self.state = d >> 4;
            self.tick += 1;
            // The motor timing out.
            if !on && self.motor_off_tick.is_some() && !self.motor {
                self.motor_off_tick = None;
                self.drive.inputs(t, false, 0);
            }
        }
    }

    fn access(&mut self, n: u16, write: Option<u8>, cyc: u64) -> u8 {
        let tick = cyc * 2 + 1;
        self.catch_up(tick);
        let value = self.reg;
        match n {
            0..=7 => {
                let bit = 1 << (n >> 1);
                if n & 1 != 0 { self.phases |= bit } else { self.phases &= !bit }
            }
            8 => {
                if self.motor {
                    self.motor = false;
                    self.motor_off_tick = Some(tick);
                }
            }
            9 => {
                self.motor = true;
                self.motor_off_tick = None;
            }
            0xA => self.drive2 = false,
            0xB => self.drive2 = true,
            0xC => self.q6 = false,
            0xD => self.q6 = true,
            0xE => self.q7 = false,
            _ => self.q7 = true,
        }
        if let Some(v) = write {
            self.bus = v;
        }
        let t = tick as f64 * TICK_S;
        let on = self.enabled(tick);
        let before = self.drive.quarter();
        self.drive.inputs(t, on, if on { self.phases } else { 0 });
        let after = self.drive.quarter();
        if after != before {
            self.trace.push((t, after));
        }
        if n & 1 == 0 { value } else { 0xFF }
    }
}

// ---- The drives ---------------------------------------------------------------------------------

/// A disk: a WOZ (its flux tracks converted at load), or an A2R, each quarter track's capture
/// made a revolution at load as the firmware does when it reads one (`Cylinder::read_a2r`).
struct Image {
    file: Vec<u8>,
    woz: Option<woz::Woz>,
    /// Per TRK record (WOZ) or quarter track (A2R): cells converted from flux, packed MSB first,
    /// their count, and the rotation the firmware gives them (an A2R's splice; else found).
    converted: Vec<Option<(Vec<u8>, u32, Option<u32>)>>,
    synchronized: bool,
}

fn pack(cells: &[bool]) -> Vec<u8> {
    let mut b = vec![0u8; cells.len().div_ceil(8) + 8];
    for (i, &c) in cells.iter().enumerate() {
        if c {
            b[i / 8] |= 0x80 >> (i % 8);
        }
    }
    b
}

/// Flux bytes to cells, at the cell measured on their first two blocks' worth (`read_flux_bytes`),
/// noting the cells at `watch` ticks.
fn flux_cells(data: &[u8], skip: usize, nominal: u32, watch: u64) -> (Vec<bool>, usize) {
    let head = (2 * BLOCK - skip % BLOCK).min(data.len());
    let mut conv = moof::FluxCells::with_cell(moof::measure_cell(&data[..head], nominal));
    conv.watch(watch);
    let mut ones = vec![];
    conv.feed(data, |i, _| ones.push(i));
    let n = conv.cells().min(204_800);
    let mut cells = vec![false; n];
    for i in ones.into_iter().filter(|&i| i < n) {
        cells[i] = true;
    }
    (cells, conv.at_watch())
}

impl Image {
    fn load(p: &str) -> Image {
        let file = std::fs::read(p).unwrap_or_else(|e| panic!("{p}: {e}"));
        if file.starts_with(b"A2R") {
            let mut ix = a2r::Indexer::new();
            for c in file.chunks(BLOCK) {
                ix.feed(c);
            }
            let r = ix.finish().unwrap_or_else(|e| panic!("{p}: {e}"));
            let converted = (0..160u8)
                .map(|q| {
                    let cap = r.capture(q)?;
                    let data = &file[cap.offset as usize..(cap.offset + cap.len) as usize];
                    let (cells, at_loop) = flux_cells(data, cap.offset as usize, r.nominal_cell(), u64::from(cap.loop_ticks));
                    let packed = pack(&cells);
                    let byte = |i: usize| packed.get(i).copied().unwrap_or(0);
                    let (at, n) = if cap.solved {
                        (0, cells.len())
                    } else {
                        const START: usize = 64; // `Cylinder`'s `A2R_START`
                        let ticks = if cap.loop_ticks > 0 { cap.loop_ticks } else { r.revolution_ticks() };
                        let estimate = if at_loop > 0 { at_loop } else { (u64::from(ticks) * 100 / u64::from(r.nominal_cell())) as usize };
                        let p = a2r::find_loop(byte, cells.len(), START, estimate, at_loop > 0).min(cells.len().saturating_sub(START));
                        a2r::splice_point(byte, cells.len(), START, p)
                    };
                    (n > 0).then(|| (pack(&cells[at..at + n]), n as u32, Some(at as u32)))
                })
                .collect();
            return Image { file, woz: None, converted, synchronized: r.synchronized };
        }
        let h: [u8; woz::HEADER_BYTES] = file[..woz::HEADER_BYTES].try_into().unwrap();
        let v1 = woz::Woz::version_of(&h) == Some(1);
        let mut bits = [0u16; woz::QUARTERS];
        if v1 {
            for (j, b) in bits.iter_mut().enumerate() {
                let at = woz::V1_OFFSET + j * woz::V1_RECORD + 6_648;
                if at + 2 <= file.len() {
                    *b = u16::from_le_bytes([file[at], file[at + 1]]);
                }
            }
        }
        let chunk: Option<[u8; BLOCK]> = moof::Moof::flux_block(&h).and_then(|b| {
            let mut c = [0u8; BLOCK];
            let src = file.get(b * BLOCK..)?;
            let k = src.len().min(BLOCK);
            c[..k].copy_from_slice(&src[..k]);
            Some(c)
        });
        let w = woz::Woz::parse(&h, &bits, chunk.as_ref(), file.len().div_ceil(BLOCK), MAX_TRACK_BLOCKS).expect("WOZ");
        let converted = (0..160u8)
            .map(|k| {
                let t = w.trk(k).filter(|t| t.flux)?;
                let from = t.start as usize * BLOCK;
                let (cells, _) = flux_cells(&file[from..from + t.bits as usize], from, 3_200, 0);
                (!cells.is_empty()).then(|| (pack(&cells), cells.len() as u32, None))
            })
            .collect();
        let synchronized = w.synchronized;
        Image { file, woz: Some(w), converted, synchronized }
    }

    /// What's heard at quarter track `q`: a TRK record, an A2R's capture, or `a2::BLANK`.
    fn track(&self, q: u8) -> u8 {
        match &self.woz {
            Some(w) => w.track(q),
            None => if self.converted.get(q as usize).is_some_and(|c| c.is_some()) { q } else { a2::BLANK },
        }
    }

    /// The cells of track `key` and its bit count.
    fn trk(&self, key: u8) -> Option<(&[u8], u32)> {
        if let Some(Some((cells, n, _))) = self.converted.get(key as usize) {
            return Some((cells, *n));
        }
        let w = self.woz.as_ref()?;
        let t = w.trk(key).filter(|t| t.bits > 0 && !t.flux)?;
        let start = if w.version == 1 { woz::V1_OFFSET + key as usize * woz::V1_RECORD } else { t.start as usize * BLOCK };
        Some((&self.file[start..], t.bits))
    }

    /// An A2R track's rotation, its splice: the firmware plays it from there, not rotated into a gap.
    fn fixed_rotation(&self, key: u8) -> Option<u32> {
        self.converted.get(key as usize).and_then(|c| c.as_ref()).and_then(|c| c.2)
    }

    /// One cell in 125 ns ticks (INFO's optimal bit timing; 4 µs for an A2R).
    fn bit_timing(&self) -> u8 {
        self.woz.as_ref().map_or(32, |w| w.bit_timing)
    }

    /// `Disk::cell_hz`: a WOZ's tracks' average, an A2R's 4 µs.
    fn cell_hz(&self) -> u32 {
        self.woz.as_ref().map_or(4_000_000, |w| w.cell_hz())
    }

    fn write_protected(&self) -> bool {
        self.woz.as_ref().is_none_or(|w| w.write_protected)
    }

    fn writable(&self) -> bool {
        self.woz.as_ref().is_some_and(|w| w.writable())
    }
}

/// What a WOZ-reference emulator does.
struct Ideal {
    img: Image,
    quarter: u8,
    key: u8,
    /// Bit position in the current track (or the 51,200-bit stand-in for an empty one).
    pos: u32,
    cell_s: f64,
    next_cell: f64,
    bit: bool,
    window: u8,
    rng: weak::Rng,
    /// No MC3470: a run of zeros plays as zeros.
    no_fake: bool,
}

impl Ideal {
    fn new(img: Image) -> Self {
        let cell_s = img.bit_timing() as f64 * 125e-9;
        let key = img.track(0);
        Ideal { img, quarter: 0, key, pos: 0, cell_s, next_cell: 0.0, bit: false, window: 0, rng: weak::Rng::new(1), no_fake: false }
    }

    fn len(&self, key: u8) -> u32 {
        self.img.trk(key).map_or(51_200, |(_, b)| b)
    }
}

impl Drive for Ideal {
    fn inputs(&mut self, _t: f64, _enabled: bool, phases: u32) {
        let q = a2::step(self.quarter, phases);
        if q != self.quarter {
            self.quarter = q;
            let key = self.img.track(q);
            if key != self.key {
                self.pos = (self.pos as u64 * self.len(key) as u64 / self.len(self.key) as u64) as u32;
                self.key = key;
            }
        }
    }

    fn pulse(&mut self, t: f64) -> bool {
        while t >= self.next_cell {
            let len = self.len(self.key);
            self.pos = (self.pos + 1) % len;
            let stored = match self.img.trk(self.key) {
                Some((d, _)) => d[self.pos as usize / 8] >> (7 - self.pos % 8) & 1 != 0,
                None => false,
            };
            // The MC3470: noise once the last 4 cells had no flux.
            self.window = (self.window << 1 | stored as u8) & 0xF;
            self.bit = if self.no_fake {
                stored
            } else if self.window == 0 {
                self.rng.next() % 10 < 3
            } else {
                self.window & 2 != 0
            };
            self.next_cell += self.cell_s;
        }
        self.bit && t - (self.next_cell - self.cell_s) < 1e-6
    }

    fn write_protected(&self) -> bool {
        self.img.write_protected()
    }

    fn quarter(&self) -> u8 {
        self.quarter
    }
}

/// A buffer core 1 can play.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Src {
    Noise,
    Set(usize),
}

/// `drive::Spin`: where a track was started, for the disk's angle.
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
    fn cell(&self, now: u32) -> u32 {
        let bits = self.bits.max(1);
        let turned = (u64::from(now.wrapping_sub(self.at)) * u64::from(self.q32) >> 32) as u32;
        (self.cell + turned % bits) % bits
    }
}

/// The PIO clock divisor (16.8) the firmware sets for `pio_hz` at 150 MHz.
const SYS_HZ: u64 = 150_000_000;

fn divisor(pio_hz: u32) -> u64 {
    (SYS_HZ * 256 + pio_hz as u64 / 2) / pio_hz as u64
}

/// What this firmware does (see `drive::run_a2` and `main`'s loop): one published buffer set at a
/// time, built by core 0 some time after the head gets there, played by core 1 through
/// `src/disk2.pio`: exactly a track's cells a revolution, started at the cell the disk has
/// turned to.
struct Firmware {
    img: Image,
    /// One cell of what plays, at the PIO's real clock; and the disk's (`A2_CELLS_Q32`).
    cell_s: f64,
    q32: u32,
    /// Per set, its track's cell and cells a µs × 2^32 (`CYL_CLKDIV`, `CYL_Q32`: a track's cells
    /// in 200 ms, `woz::track_hz`); and a cell to play every track at instead (`--cell-ns`).
    rate: [(f64, u32); 2],
    fixed: Option<f64>,
    // Core 0's buffers, per set: cells (0 = the noise buffer), words, rotation in cells, 2^32 /
    // cells, span, weak spots.
    bufs: Vec<Box<TrackBuf>>,
    bits: [u32; 2],
    rotc: [u32; 2],
    inv: [u32; 2],
    spans: [u32; 2],
    weak: [weak::WeakList; 2],
    noise: Vec<u32>,
    rotations: Vec<Option<u32>>,
    // PUBLISHED and PLAYING: (set, quarter), or nothing yet.
    published: Option<(usize, u8)>,
    seen: Option<(usize, u8)>,
    // Core 0: the key of the cylinder it holds, and a build in progress (done at, key, head).
    built: Option<u8>,
    building: Option<(f64, u8, u8)>,
    core0_free_at: f64,
    next_core0: f64,
    next_weak: f64,
    next_display: f64,
    /// Pieces of the OLED's screen still to send (8 a redraw, one a main-loop pass).
    display_pieces: u32,
    rng: weak::Rng,
    // Core 1.
    quarter: u8,
    on_track: bool,
    spin: Spin,
    on_spin: bool,
    next_housekeeping: f64,
    // The PIO: what plays, the next cell of its loop, when that cell starts.
    src: Src,
    pos: u32,
    next_cell: f64,
    bit: bool,
    bit_at: f64,
    // Settings.
    sd_s: f64,
    rotation_s: f64,
    display_s: f64,
    /// Before quarter-track spans: noise on every step until core 0 republishes.
    no_span: bool,
    /// Weak-bit runs (zero cells long, cells kept).
    weak_run: (usize, usize),
    // Counts, for the report.
    noise_cells: u64,
    builds: u32,
}

impl Firmware {
    fn new(img: Image, sd_ms: f64, rotation_ms: f64, display_ms: f64) -> Self {
        let cell_hz = img.cell_hz();
        let div = divisor(cell_hz);
        let mut noise = vec![0u32; 2_048];
        let mut rng = weak::Rng::new(0x1234);
        for w in noise.iter_mut() {
            *w = rng.noise();
        }
        Firmware {
            cell_s: 16.0 * div as f64 / 256.0 / SYS_HZ as f64,
            q32: ((SYS_HZ as u128) << 40).div_euclid(div as u128 * 16 * 1_000_000) as u32,
            rate: [(0.0, 0); 2],
            fixed: None,
            img,
            bufs: vec![Box::new([0; MAX_TRACK_WORDS]), Box::new([0; MAX_TRACK_WORDS])],
            bits: [0; 2],
            rotc: [0; 2],
            inv: [0; 2],
            spans: [0; 2],
            weak: [weak::WeakList::new(), weak::WeakList::new()],
            noise,
            rotations: vec![None; 256],
            published: None,
            seen: None,
            built: None,
            building: None,
            core0_free_at: 0.0,
            next_core0: 0.0,
            next_weak: 0.0,
            next_display: 0.25,
            display_pieces: 0,
            rng,
            quarter: 0,
            on_track: false,
            spin: Spin { at: 0, cell: 0, bits: 1, rotc: 0, inv: 0, q32: 0 },
            on_spin: false,
            next_housekeeping: 0.0,
            src: Src::Noise,
            pos: 0,
            next_cell: 0.0,
            bit: false,
            bit_at: 0.0,
            sd_s: sd_ms / 1e3,
            rotation_s: rotation_ms / 1e3,
            display_s: display_ms / 1e3,
            no_span: false,
            weak_run: weak::A2_RUN,
            noise_cells: 0,
            builds: 0,
        }
    }

    /// The cells a source loops: a track's, or all but one of the noise buffer's.
    fn loop_bits(&self, src: Src) -> u32 {
        match src {
            Src::Set(s) if self.bits[s] > 0 => self.bits[s],
            _ => self.noise.len() as u32 * 32 - 1,
        }
    }

    fn cell_bit(&self, src: Src, i: u32) -> bool {
        let w = match src {
            Src::Set(s) if self.bits[s] > 0 => self.bufs[s][i as usize / 32],
            _ => self.noise[i as usize / 32],
        };
        w >> (31 - i % 32) & 1 != 0
    }

    /// `Player::play_a2`: the state machine restarted, the first cell now (a track at its own
    /// cell, `play_track`; noise at what played last).
    fn play(&mut self, src: Src, cell: u32, t: f64) {
        if let Src::Set(s) = src {
            self.cell_s = self.fixed.unwrap_or(self.rate[s].0);
        }
        self.src = src;
        self.pos = if cell < self.loop_bits(src) { cell } else { 0 };
        self.next_cell = t;
    }

    fn in_span(span: u32, q: u8) -> bool {
        (q as u32).wrapping_sub(span & 0xFF) <= (span >> 8 & 0xFF).wrapping_sub(span & 0xFF)
    }

    fn span(&self, head: u8) -> u32 {
        if self.no_span {
            return head as u32 * 0x101;
        }
        let key = self.img.track(head);
        let (mut lo, mut hi) = (head, head);
        while lo > 0 && self.img.track(lo - 1) == key {
            lo -= 1;
        }
        while hi < 159 && self.img.track(hi + 1) == key {
            hi += 1;
        }
        lo as u32 | (hi as u32) << 8
    }

    /// `Spin::start_in`.
    fn start_in(&self, set: usize, now: u32) -> u32 {
        let s = &self.spin;
        let imaged = (s.cell(now) + s.rotc) % s.bits.max(1);
        let fraction = imaged.wrapping_mul(s.inv);
        let bits = self.bits[set].max(1);
        let there = (u64::from(fraction) * u64::from(bits) >> 32) as u32;
        (there + bits - self.rotc[set] % bits) % bits
    }

    /// Core 1's housekeeping (every ~4 µs).
    fn housekeeping(&mut self, t: f64) {
        let now = (t * 1e6) as u32;
        let Some((set, _)) = self.published else { return };
        let span = self.spans[set];
        let ours = Self::in_span(span, self.quarter);
        if self.published != self.seen || ours != self.on_track {
            let seen_set = self.seen.map(|s| s.0);
            let bits = self.bits[set];
            if ours && self.on_track && Some(set) == seen_set {
            } else if ours && bits == 0 {
                self.on_spin = false;
                self.play(Src::Noise, 0, t);
            } else if ours {
                let cell = if self.on_spin && self.on_track && seen_set.is_some_and(|s| self.spans[s] == span) {
                    self.spin.cell(now)
                } else {
                    self.start_in(set, now)
                };
                self.play(Src::Set(set), cell, t);
                self.spin = Spin { at: now, cell, bits, rotc: self.rotc[set], inv: self.inv[set], q32: self.rate[set].1 };
                self.on_spin = true;
            } else if self.on_track {
                self.on_spin = false;
                self.play(Src::Noise, 0, t);
            }
            self.on_track = ours;
            self.seen = self.published;
        }
    }

    /// Core 0's loop, at `t`.
    fn core0(&mut self, t: f64) {
        if t < self.core0_free_at {
            return;
        }
        // A build finishing: into the free set, published.
        if let Some((done, key, head)) = self.building {
            if t < done {
                return;
            }
            self.building = None;
            let set = match self.published {
                Some((s, _)) => s ^ 1,
                None => 0,
            };
            self.weak[set].clear();
            match self.img.trk(key) {
                Some((data, bits)) => {
                    // An A2R's track as cut at its splice, which is its rotation; else into its gap.
                    let (load_at, rot) = match self.img.fixed_rotation(key) {
                        Some(splice) => (0, splice),
                        None => {
                            let r = *self.rotations[key as usize].get_or_insert_with(|| moof::find_rotation(|i| data[i], bits));
                            (r, r)
                        }
                    };
                    moof::load_track(|i| data[i], bits, load_at, moof::Pad::None, &mut self.bufs[set]);
                    self.weak[set].find_runs(&self.bufs[set][..], bits as usize, self.weak_run);
                    self.bits[set] = bits;
                    self.rotc[set] = rot;
                    self.inv[set] = ((1u64 << 32) / bits as u64) as u32;
                    let div = divisor(woz::track_hz(bits, self.img.cell_hz()));
                    self.rate[set] = (16.0 * div as f64 / 256.0 / SYS_HZ as f64, ((SYS_HZ as u128) << 40).div_euclid(div as u128 * 16 * 1_000_000) as u32);
                    if let Some(c) = self.fixed {
                        self.rate[set] = (c, (4_294_967_296e-6 / c) as u32);
                    }
                }
                None => {
                    self.bits[set] = 0;
                    self.rotc[set] = 0;
                    self.inv[set] = 0;
                }
            }
            self.spans[set] = self.span(head);
            self.published = Some((set, head));
            self.built = Some(key);
            self.builds += 1;
            return;
        }
        // The display: drawn every 250 ms, sent a piece a pass (`display_s` is a piece's time).
        if t >= self.next_display {
            self.next_display = t + 0.25;
            self.display_pieces = 8;
        }
        if self.display_pieces > 0 {
            self.display_pieces -= 1;
            self.core0_free_at = t + self.display_s;
        }
        // Weak bits and noise, every 50 ms.
        if t >= self.next_weak {
            self.next_weak = t + 0.05;
            if self.seen == self.published && let Some((set, _)) = self.published {
                let weak = std::mem::replace(&mut self.weak[set], weak::WeakList::new());
                weak.randomize(&mut self.bufs[set][..], &mut self.rng);
                self.weak[set] = weak;
            }
            for i in 0..self.noise.len() {
                self.noise[i] = self.rng.noise();
            }
        }
        // Follow the head.
        let caught_up = self.seen == self.published;
        let head = self.quarter;
        let key = self.img.track(head);
        if self.built != Some(key) && caught_up {
            let first = key != a2::BLANK && self.rotations[key as usize].is_none();
            let cost = if key == a2::BLANK { 0.0002 } else { self.sd_s + if first { self.rotation_s } else { 0.0 } + 0.0005 };
            self.building = Some((t + cost, key, head));
        } else if caught_up && self.built == Some(key) && self.published.is_some_and(|(_, q)| q != head) {
            let set = self.published.unwrap().0;
            self.spans[set] = self.span(head);
            self.published = Some((set, head));
        }
    }
}

impl Drive for Firmware {
    fn inputs(&mut self, t: f64, _enabled: bool, phases: u32) {
        let q = a2::step(self.quarter, phases);
        if q != self.quarter {
            self.quarter = q;
            if self.on_track && let Some((set, _)) = self.seen && !Self::in_span(self.spans[set], q) {
                self.on_track = false;
                self.on_spin = false;
                self.play(Src::Noise, 0, t);
            }
        }
    }

    fn pulse(&mut self, t: f64) -> bool {
        while t >= self.next_housekeeping {
            let at = self.next_housekeeping;
            self.housekeeping(at);
            self.next_housekeeping += 4e-6;
        }
        while t >= self.next_core0 {
            let at = self.next_core0;
            self.core0(at);
            self.next_core0 += 50e-6;
        }
        while t >= self.next_cell {
            self.bit = self.cell_bit(self.src, self.pos);
            self.bit_at = self.next_cell;
            self.pos = (self.pos + 1) % self.loop_bits(self.src);
            if !self.on_track {
                self.noise_cells += 1;
            }
            self.next_cell += self.cell_s;
        }
        // RD low 1 µs, 3/4 of the way into the cell (cycles 12–15).
        let into = t - self.bit_at;
        self.bit && into >= self.cell_s * 0.75 && into < self.cell_s
    }

    fn write_protected(&self) -> bool {
        !self.img.writable()
    }

    fn quarter(&self) -> u8 {
        self.quarter
    }

    fn note(&self) -> String {
        format!("{} track builds, {:.1} s of noise played", self.builds, self.noise_cells as f64 * self.cell_s)
    }
}

// ---- The IIe ------------------------------------------------------------------------------------

struct Machine {
    main: Vec<u8>,
    aux: Vec<u8>,
    /// Language card bank 1 ($D000–$DFFF), main and aux; bank 2 and $E000 up live in `main`/`aux`.
    bank1: [Vec<u8>; 2],
    rom: Vec<u8>,
    p5: Vec<u8>,
    store80: bool,
    ramrd: bool,
    ramwrt: bool,
    intcxrom: bool,
    altzp: bool,
    slotc3rom: bool,
    intc8rom: bool,
    col80: bool,
    altchar: bool,
    text: bool,
    mixed: bool,
    page2: bool,
    hires: bool,
    lc_read: bool,
    lc_write: bool,
    lc_prewrite: bool,
    lc_bank2: bool,
    kbd: u8,
    keys: VecDeque<(u64, u8)>,
    paddle_at: u64,
    disk: Disk2,
}

impl Machine {
    fn lc_ram(&mut self, a: u16) -> &mut u8 {
        let z = self.altzp as usize;
        if a < 0xE000 && !self.lc_bank2 {
            &mut self.bank1[z][(a - 0xD000) as usize]
        } else if z == 1 {
            &mut self.aux[a as usize]
        } else {
            &mut self.main[a as usize]
        }
    }

    fn aux_for(&self, a: u16, write: bool) -> bool {
        if a < 0x200 {
            return self.altzp;
        }
        if self.store80 && ((0x400..0x800).contains(&a) || (self.hires && (0x2000..0x4000).contains(&a))) {
            return self.page2;
        }
        if write { self.ramwrt } else { self.ramrd }
    }

    fn lc_switch(&mut self, n: u16, read: bool) {
        self.lc_bank2 = n & 8 == 0;
        self.lc_read = matches!(n & 3, 0 | 3);
        if n & 1 != 0 {
            if read && self.lc_prewrite {
                self.lc_write = true;
            }
            self.lc_prewrite = read;
        } else {
            self.lc_write = false;
            self.lc_prewrite = false;
        }
    }

    fn display(&mut self, n: u16) {
        match n {
            0x50 => self.text = false,
            0x51 => self.text = true,
            0x52 => self.mixed = false,
            0x53 => self.mixed = true,
            0x54 => self.page2 = false,
            0x55 => self.page2 = true,
            0x56 => self.hires = false,
            0x57 => self.hires = true,
            _ => {}
        }
    }

    fn io(&mut self, a: u16, write: Option<u8>, cyc: u64) -> u8 {
        let n = a & 0xFF;
        let flag = |b: bool, k: u8| if b { 0x80 | k & 0x7F } else { k & 0x7F };
        match n {
            0x00..=0x0F => {
                if write.is_some() {
                    let on = n & 1 != 0;
                    match n >> 1 {
                        0 => self.store80 = on,
                        1 => self.ramrd = on,
                        2 => self.ramwrt = on,
                        3 => self.intcxrom = on,
                        4 => self.altzp = on,
                        5 => self.slotc3rom = on,
                        6 => self.col80 = on,
                        _ => self.altchar = on,
                    }
                    0
                } else {
                    self.kbd
                }
            }
            0x10 => {
                self.kbd &= 0x7F;
                self.kbd
            }
            0x11..=0x1F if write.is_none() => {
                let k = self.kbd;
                let vbl = cyc % 17_030 >= 12_480;
                match n {
                    0x11 => flag(self.lc_bank2, k),
                    0x12 => flag(self.lc_read, k),
                    0x13 => flag(self.ramrd, k),
                    0x14 => flag(self.ramwrt, k),
                    0x15 => flag(self.intcxrom, k),
                    0x16 => flag(self.altzp, k),
                    0x17 => flag(self.slotc3rom, k),
                    0x18 => flag(self.store80, k),
                    0x19 => flag(!vbl, k),
                    0x1A => flag(self.text, k),
                    0x1B => flag(self.mixed, k),
                    0x1C => flag(self.page2, k),
                    0x1D => flag(self.hires, k),
                    0x1E => flag(self.altchar, k),
                    _ => flag(self.col80, k),
                }
            }
            0x10..=0x1F => {
                self.kbd &= 0x7F;
                0
            }
            0x50..=0x57 => {
                self.display(n);
                0
            }
            0x61..=0x63 => 0,
            0x64..=0x67 => {
                if cyc - self.paddle_at < 1_400 { 0x80 } else { 0 }
            }
            0x70 => {
                self.paddle_at = cyc;
                0
            }
            0x80..=0x8F => {
                self.lc_switch(n & 0xF, write.is_none());
                0
            }
            0xE0..=0xEF => self.disk.access(n & 0xF, write, cyc),
            _ => 0,
        }
    }

    fn slot_rom(&mut self, a: u16) -> u8 {
        if a == 0xCFFF {
            self.intc8rom = false;
        }
        let rom = self.rom[(a - 0xC000) as usize];
        if self.intcxrom {
            return rom;
        }
        match a >> 8 {
            0xC3 if !self.slotc3rom => {
                self.intc8rom = true;
                rom
            }
            0xC6 => self.p5[(a & 0xFF) as usize],
            0xC8..=0xCF if self.intc8rom => rom,
            _ => 0xFF,
        }
    }
}

impl Bus for Machine {
    fn rd(&mut self, a: u16, cyc: u64) -> u8 {
        match a {
            0xC000..=0xC0FF => self.io(a, None, cyc),
            0xC100..=0xCFFF => self.slot_rom(a),
            0xD000..=0xFFFF => {
                if self.lc_read { *self.lc_ram(a) } else { self.rom[(a - 0xC000) as usize] }
            }
            _ => {
                if self.aux_for(a, false) { self.aux[a as usize] } else { self.main[a as usize] }
            }
        }
    }

    fn wr(&mut self, a: u16, v: u8, cyc: u64) {
        match a {
            0xC000..=0xC0FF => {
                self.io(a, Some(v), cyc);
            }
            0xC100..=0xCFFF => {
                if a == 0xCFFF {
                    self.intc8rom = false;
                }
            }
            0xD000..=0xFFFF => {
                if self.lc_write {
                    *self.lc_ram(a) = v;
                }
            }
            _ => {
                if self.aux_for(a, true) { self.aux[a as usize] = v } else { self.main[a as usize] = v }
            }
        }
    }
}

// ---- The screen ---------------------------------------------------------------------------------

fn text_row(m: &Machine, row: usize) -> String {
    let base = if m.page2 && !m.store80 { 0x800 } else { 0x400 } + (row % 8) * 0x80 + (row / 8) * 0x28;
    (0..40)
        .map(|c| {
            let v = m.main[base + c];
            let low = v & 0x3F;
            if v >= 0xE0 {
                (v & 0x7F) as char
            } else if low < 0x20 {
                (low + 0x40) as char
            } else {
                low as char
            }
        })
        .collect()
}

fn hires_pixels(m: &Machine) -> Vec<u8> {
    let base = if m.page2 && !m.store80 { 0x4000 } else { 0x2000 };
    let mut px = vec![0u8; 280 * 192];
    for y in 0..192 {
        let line = base + (y % 8) * 0x400 + ((y / 8) % 8) * 0x80 + (y / 64) * 0x28;
        for xb in 0..40 {
            let v = m.main[line + xb];
            for k in 0..7 {
                if v >> k & 1 != 0 {
                    px[y * 280 + xb * 7 + k] = 255;
                }
            }
        }
    }
    px
}

fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
    }
    !c
}

/// A grayscale PNG, uncompressed.
fn png(path: &str, w: usize, h: usize, px: &[u8]) {
    let mut raw = Vec::with_capacity((w + 1) * h);
    for y in 0..h {
        raw.push(0);
        raw.extend_from_slice(&px[y * w..(y + 1) * w]);
    }
    let mut z = vec![0x78, 0x01];
    for (k, chunk) in raw.chunks(65_535).enumerate() {
        let last = (k + 1) * 65_535 >= raw.len();
        z.push(last as u8);
        z.extend_from_slice(&(chunk.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(chunk.len() as u16)).to_le_bytes());
        z.extend_from_slice(chunk);
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &v in &raw {
        a = (a + v as u32) % 65_521;
        b = (b + a) % 65_521;
    }
    z.extend_from_slice(&(b << 16 | a).to_be_bytes());
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut chunk = |kind: &[u8], data: &[u8]| {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut c = kind.to_vec();
        c.extend_from_slice(data);
        out.extend_from_slice(&c);
        out.extend_from_slice(&crc32(&c).to_be_bytes());
    };
    let mut ihdr = (w as u32).to_be_bytes().to_vec();
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 0, 0, 0, 0]);
    chunk(b"IHDR", &ihdr);
    chunk(b"IDAT", &z);
    chunk(b"IEND", &[]);
    std::fs::write(path, out).unwrap();
}

/// The screen as shown: hi-res (with its text lines when mixed) as a PNG, or the text.
fn snapshot(m: &Machine, png_path: Option<&str>) -> String {
    let mut s = String::new();
    if !m.text && m.hires {
        s += &format!("[hi-res page {}{}]\n", if m.page2 { 2 } else { 1 }, if m.mixed { ", mixed" } else { "" });
        if let Some(p) = png_path {
            png(p, 280, 192, &hires_pixels(m));
            s += &format!("[saved {p}]\n");
        }
        if m.mixed {
            for row in 20..24 {
                s += &text_row(m, row);
                s.push('\n');
            }
        }
    } else if !m.text {
        s += "[lo-res]\n";
    } else {
        for row in 0..24 {
            s += &text_row(m, row);
            s.push('\n');
        }
    }
    s
}

// ---- Main ---------------------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut disk = None;
    let mut drive = "fw".to_string();
    let mut secs = 30.0f64;
    let mut keys: Vec<(f64, String)> = vec![];
    let mut png_path = None;
    // Core 0's costs: a WOZ track's SD read, a first `find_rotation` (0: `Cylinder::prepare`
    // finds them all at load), a piece of the OLED's screen.
    let (mut sd_ms, mut rot_ms, mut disp_ms) = (4.0, 0.0, 0.9);
    let mut dumps: Vec<(u16, u16)> = vec![];
    let (mut no_span, mut weak8) = (false, false);
    let mut trace_path: Option<String> = None;
    let mut cell_ns: Option<f64> = None;
    let mut no_weak = false;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--drive" => {
                drive = args[i + 1].clone();
                i += 1;
            }
            "--secs" => {
                secs = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--key" => {
                let (t, k) = args[i + 1].split_once(':').expect("--key SECS:TEXT");
                // `\r` for Return, `\xNN` for any other code.
                let mut text = String::new();
                let mut rest = k;
                while let Some(at) = rest.find('\\') {
                    text += &rest[..at];
                    let esc = &rest[at + 1..];
                    if let Some(hex) = esc.strip_prefix('x') {
                        text.push(u8::from_str_radix(&hex[..2], 16).unwrap() as char);
                        rest = &hex[2..];
                    } else {
                        text.push(if esc.starts_with('r') { '\r' } else { esc.chars().next().unwrap() });
                        rest = &esc[1..];
                    }
                }
                text += rest;
                keys.push((t.parse().unwrap(), text));
                i += 1;
            }
            "--png" => {
                png_path = Some(args[i + 1].clone());
                i += 1;
            }
            "--sd-ms" => {
                sd_ms = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--rotation-ms" => {
                rot_ms = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--no-span" => no_span = true,
            "--no-weak" => no_weak = true,
            "--cell-ns" => {
                cell_ns = Some(args[i + 1].parse().unwrap());
                i += 1;
            }
            "--trace" => {
                trace_path = Some(args[i + 1].clone());
                i += 1;
            }
            "--weak8" => weak8 = true,
            "--dump" => {
                let (a, n) = args[i + 1].split_once(':').expect("--dump HEXADDR:HEXLEN");
                dumps.push((u16::from_str_radix(a, 16).unwrap(), u16::from_str_radix(n, 16).unwrap()));
                i += 1;
            }
            "--display-ms" => {
                disp_ms = args[i + 1].parse().unwrap();
                i += 1;
            }
            a => disk = Some(a.to_string()),
        }
        i += 1;
    }
    let disk = disk.expect("a2emu <disk.woz> [--drive ideal|fw] [--secs N] [--key SECS:TEXT] [--png FILE]");
    let img = Image::load(&disk);
    let d: Box<dyn Drive> = match drive.as_str() {
        "ideal" => {
            let mut d = Ideal::new(img);
            if let Some(ns) = cell_ns {
                d.cell_s = ns * 1e-9;
            }
            d.no_fake = no_weak;
            Box::new(d)
        }
        _ => {
            let mut f = Firmware::new(img, sd_ms, rot_ms, disp_ms);
            f.no_span = no_span;
            if let Some(ns) = cell_ns {
                f.cell_s = ns * 1e-9;
                f.q32 = (4_294_967_296_000.0 / ns) as u32;
                f.fixed = Some(ns * 1e-9);
            }
            if weak8 {
                f.weak_run = weak::GCR_RUN;
            }
            if no_weak {
                f.weak_run = (100_000, 0);
            }
            Box::new(f)
        }
    };
    let read = |p: &str| std::fs::read(path(p)).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut rom = read(ROM_CD);
    rom.extend(read(ROM_EF));
    assert_eq!(rom.len(), 0x4000);
    let mut m = Machine {
        main: vec![0; 0x10000],
        aux: vec![0; 0x10000],
        bank1: [vec![0; 0x1000], vec![0; 0x1000]],
        rom,
        p5: read(ROM_P5),
        store80: false,
        ramrd: false,
        ramwrt: false,
        intcxrom: false,
        altzp: false,
        slotc3rom: false,
        intc8rom: false,
        col80: false,
        altchar: false,
        text: true,
        mixed: false,
        page2: false,
        hires: false,
        lc_read: false,
        lc_write: false,
        lc_prewrite: false,
        lc_bank2: true,
        kbd: 0,
        keys: VecDeque::new(),
        paddle_at: 0,
        disk: Disk2::new(&read(ROM_P6), d),
    };
    for (t, text) in &keys {
        for (k, ch) in text.chars().enumerate() {
            m.keys.push_back((((t + k as f64 * 0.3) * CPU_HZ) as u64, ch.to_ascii_uppercase() as u8));
        }
    }
    let mut cpu = Cpu::new();
    cpu.reset(&mut m);
    let end = (secs * CPU_HZ) as u64;
    let mut boots = 0;
    let mut pcs = std::collections::HashMap::new();
    while cpu.cyc < end {
        if cpu.pc == 0xC600 {
            boots += 1;
        }
        if cpu.cyc > end - (CPU_HZ as u64) {
            *pcs.entry(cpu.pc).or_insert(0u32) += 1;
        }
        if !cpu.step(&mut m) {
            println!("JAM at ${:04X} after {:.2} s", cpu.pc, cpu.cyc as f64 / CPU_HZ);
            break;
        }
        if let Some(&(at, k)) = m.keys.front()
            && cpu.cyc >= at
            && m.kbd & 0x80 == 0
        {
            m.kbd = k | 0x80;
            m.keys.pop_front();
            println!("key {:02X} at {:.2} s, PC ${:04X}", k, cpu.cyc as f64 / CPU_HZ, cpu.pc);
        }
    }
    m.disk.catch_up(cpu.cyc * 2);
    // Report.
    println!("{disk}, {drive} drive, {secs} s: booted from $C600 {boots} time(s); {}", m.disk.drive.note());
    let mut hot: Vec<_> = pcs.into_iter().collect();
    hot.sort_by(|a, b| b.1.cmp(&a.1));
    println!("busiest PCs in the last second: {}", hot.iter().take(6).map(|(pc, n)| format!("${pc:04X}×{n}")).collect::<Vec<_>>().join(" "));
    // The head's path, in tracks: its first and last moves, and how far it went.
    let mut moves: Vec<(f64, u8)> = vec![];
    for &(t, q) in &m.disk.trace {
        if moves.last().is_none_or(|&(_, l)| l != q) {
            moves.push((t, q));
        }
    }
    if let Some(p) = &trace_path {
        let lines: Vec<String> = moves.iter().map(|&(t, q)| format!("{t:.6} {q}")).collect();
        std::fs::write(p, lines.join("\n") + "\n").unwrap();
    }
    let show = |v: &[(f64, u8)]| v.iter().map(|&(t, q)| format!("{:.2}@{:.2}s", q as f64 / 4.0, t)).collect::<Vec<_>>().join(" ");
    let far = moves.iter().map(|m| m.1).max().unwrap_or(0) as f64 / 4.0;
    println!("head: {} moves, out to track {far}; first: {}", moves.len(), show(&moves[..moves.len().min(40)]));
    if moves.len() > 40 {
        println!("head, last: {}", show(&moves[moves.len().saturating_sub(40).max(40)..]));
    }
    print!("{}", snapshot(&m, png_path.as_deref()));
    // Memory as the CPU sees it now.
    for (a, n) in dumps {
        for row in (a as u32..a as u32 + n as u32).step_by(16) {
            let bytes: Vec<String> = (row..(row + 16).min(a as u32 + n as u32)).map(|x| format!("{:02X}", m.rd(x as u16, cpu.cyc))).collect();
            println!("{row:04X}: {}", bytes.join(" "));
        }
    }
}
