//! Minimal PIO interpreter for the playback program in `src/disk2.pio`, for host tests, with the
//! DMA that feeds it (`drive::Stream` in Apple II mode: the track's words, then the revolution's
//! cell count, then back to the first word) and core 1's way of starting a track
//! (`drive::Player::play_a2`). Supports exactly what that program uses: `pull block`,
//! `mov y, osr`, `out x|null, n`, `jmp [!x|y--|!osre] label`, side-set (one pin, not optional)
//! and delays. The OSR shifts left; no autopull; the TX FIFO is joined (8 deep).

#[derive(Clone, Copy, Debug)]
enum Op {
    Pull,
    MovYOsr,
    OutX(u32),
    OutNull(u32),
    Jmp(usize),
    JmpNotX(usize),
    JmpYDec(usize),
    JmpNotOsre(usize),
}

#[derive(Clone, Copy)]
struct Ins {
    op: Op,
    side: bool,
    delay: u32,
}

pub struct Program {
    ins: Vec<Ins>,
    pub labels: std::collections::HashMap<String, usize>,
}

/// Parse one `.program` from a .pio source.
pub fn parse(source: &str, name: &str) -> Program {
    let mut lines = vec![];
    let mut inside = false;
    for raw in source.lines() {
        let line = raw.split(';').next().unwrap().trim();
        if let Some(n) = line.strip_prefix(".program") {
            inside = n.trim() == name;
            continue;
        }
        if inside && !line.is_empty() && !line.starts_with('.') {
            lines.push(line.to_string());
        }
    }
    assert!(!lines.is_empty(), "program {name} not found");
    let mut labels = std::collections::HashMap::new();
    let mut count = 0;
    for l in &lines {
        if let Some(label) = l.strip_suffix(':') {
            labels.insert(label.trim_start_matches("public ").to_string(), count);
        } else {
            count += 1;
        }
    }
    let target = |t: &str| *labels.get(t.trim()).unwrap_or_else(|| panic!("label {t}"));
    let mut ins = vec![];
    for l in &lines {
        if l.ends_with(':') {
            continue;
        }
        // `op args side N [d]`
        let (body, delay) = match l.split_once('[') {
            Some((b, d)) => (b.trim(), d.trim_end_matches(']').trim().parse().unwrap()),
            None => (l.as_str(), 0),
        };
        let (body, side) = body.rsplit_once(" side ").expect("side-set on every instruction");
        let side = side.trim() == "1";
        let (mnemonic, args) = body.trim().split_once(char::is_whitespace).unwrap();
        let args: Vec<&str> = args.split(',').map(str::trim).collect();
        let op = match (mnemonic, args.as_slice()) {
            ("pull", ["block"]) => Op::Pull,
            ("mov", ["y", "osr"]) => Op::MovYOsr,
            ("out", ["x", n]) => Op::OutX(n.parse().unwrap()),
            ("out", ["null", n]) => Op::OutNull(n.parse().unwrap()),
            ("jmp", [rest]) => match rest.split_once(char::is_whitespace) {
                Some(("!x", t)) => Op::JmpNotX(target(t)),
                Some(("y--", t)) => Op::JmpYDec(target(t)),
                Some(("!osre", t)) => Op::JmpNotOsre(target(t)),
                _ => Op::Jmp(target(rest)),
            },
            _ => panic!("unsupported: {l}"),
        };
        ins.push(Ins { op, side, delay });
    }
    Program { ins, labels }
}

/// The state machine, its FIFO and the DMA chain, as `drive::Stream` sets them up.
pub struct Player<'a> {
    prog: &'a Program,
    pc: usize,
    x: u32,
    y: u32,
    osr: u32,
    /// Bits shifted out of the OSR (32 = empty).
    shifted: u32,
    delay: u32,
    stalled: bool,
    pub rd: bool,
    fifo: std::collections::VecDeque<u32>,
    enabled: bool,
    // DMA: the buffer being played and the next word, the words a revolution takes, and the
    // count channel's word; `count_due` once the data channel has finished.
    buf: &'a [u32],
    next: usize,
    words: usize,
    count_word: u32,
    count_due: bool,
}

impl<'a> Player<'a> {
    pub fn new(prog: &'a Program) -> Self {
        Player {
            prog,
            pc: 0,
            x: 0,
            y: 0,
            osr: 0,
            shifted: 32,
            delay: 0,
            stalled: false,
            rd: true,
            fifo: Default::default(),
            enabled: false,
            buf: &[],
            next: 0,
            words: 0,
            count_word: 0,
            count_due: false,
        }
    }

    /// What `play_a2` does: start `buf` (a revolution of `bits` cells in `bits / 32 + 1` words)
    /// at cell `cell`.
    pub fn play(&mut self, buf: &'a [u32], bits: u32, cell: u32) {
        // Disable, abort the DMA, empty the FIFO, restart the state machine.
        self.enabled = false;
        self.fifo.clear();
        self.shifted = 32;
        self.delay = 0;
        self.stalled = false;
        // The cells left in this revolution, then the DMA from the cell's word on.
        self.fifo.push_back(bits - cell - 1);
        self.buf = buf;
        self.words = bits as usize / 32 + 1;
        self.next = cell as usize / 32;
        self.count_word = bits - 1;
        self.count_due = false;
        self.dma();
        self.exec(Op::Pull);
        self.exec(Op::MovYOsr);
        self.exec(Op::Pull);
        if cell % 32 != 0 {
            self.exec(Op::OutNull(cell % 32));
        }
        self.pc = self.prog.labels["cell"];
        self.enabled = true;
    }

    /// Keep the FIFO full: the data channel's words, then the count channel's, then the control
    /// channel restarts the data channel at the buffer's start.
    fn dma(&mut self) {
        while self.fifo.len() < 8 && !self.buf.is_empty() {
            if self.count_due {
                self.fifo.push_back(self.count_word);
                self.count_due = false;
                self.next = 0;
            } else {
                self.fifo.push_back(self.buf[self.next]);
                self.next += 1;
                if self.next == self.words {
                    self.count_due = true;
                }
            }
        }
    }

    /// Run one instruction's effect (no timing), as SMx_INSTR does. Returns a jump target.
    fn exec(&mut self, op: Op) -> Option<usize> {
        match op {
            Op::Pull => {
                let w = self.fifo.pop_front().expect("pull from an empty FIFO");
                self.osr = w;
                self.shifted = 0;
            }
            Op::MovYOsr => self.y = self.osr,
            Op::OutX(n) | Op::OutNull(n) => {
                let n = if n == 0 { 32 } else { n };
                let v = if n == 32 { self.osr } else { self.osr >> (32 - n) };
                self.osr = if n == 32 { 0 } else { self.osr << n };
                self.shifted = (self.shifted + n).min(32);
                if let Op::OutX(_) = op {
                    self.x = v;
                }
            }
            Op::Jmp(t) => return Some(t),
            Op::JmpNotX(t) => return (self.x == 0).then_some(t),
            Op::JmpYDec(t) => {
                let taken = self.y != 0;
                self.y = self.y.wrapping_sub(1);
                return taken.then_some(t);
            }
            Op::JmpNotOsre(t) => return (self.shifted < 32).then_some(t),
        }
        None
    }

    /// One PIO cycle. Returns RD after it.
    pub fn cycle(&mut self) -> bool {
        self.dma();
        if !self.enabled {
            return self.rd;
        }
        if self.delay > 0 {
            self.delay -= 1;
            return self.rd;
        }
        let ins = self.prog.ins[self.pc];
        if let Op::Pull = ins.op
            && self.fifo.is_empty()
        {
            self.stalled = true;
            return self.rd;
        }
        self.stalled = false;
        self.rd = ins.side;
        let jump = self.exec(ins.op);
        self.pc = jump.unwrap_or(self.pc + 1);
        self.delay = ins.delay;
        self.rd
    }

    pub fn stalled(&self) -> bool {
        self.stalled
    }
}
