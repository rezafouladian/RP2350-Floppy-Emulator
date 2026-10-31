//! Minimal PIO interpreter for the write-capture programs in `src/capture.pio`, for host tests.
//! Supports exactly what those programs use: `set x|y, n`, `jmp [pin|x--] label`,
//! `in y|null, n`, labels and `.program`. One instruction per cycle; `jmp pin` sees the pin
//! through the 2-cycle input synchroniser. ISR shifts left with autopush at 32.

#[derive(Clone, Copy, Debug)]
enum Op {
    SetX(u32),
    SetY(u32),
    Jmp(usize),
    JmpPin(usize),
    JmpXDec(usize),
    InY(u32),
    InNull(u32),
}

pub struct Program {
    ops: Vec<Op>,
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
        if inside && !line.is_empty() {
            lines.push(line.to_string());
        }
    }
    assert!(!lines.is_empty(), "program {name} not found");
    let mut labels = std::collections::HashMap::new();
    let mut count = 0;
    for l in &lines {
        if let Some(label) = l.strip_suffix(':') {
            labels.insert(label.to_string(), count);
        } else {
            count += 1;
        }
    }
    let target = |t: &str| *labels.get(t.trim()).unwrap_or_else(|| panic!("label {t}"));
    let mut ops = vec![];
    for l in &lines {
        if l.ends_with(':') {
            continue;
        }
        let (mnemonic, args) = l.split_once(char::is_whitespace).unwrap();
        let args: Vec<&str> = args.split(',').map(str::trim).collect();
        let op = match (mnemonic, args.as_slice()) {
            ("set", ["x", n]) => Op::SetX(n.parse().unwrap()),
            ("set", ["y", n]) => Op::SetY(n.parse().unwrap()),
            ("in", ["y", n]) => Op::InY(n.parse().unwrap()),
            ("in", ["null", n]) => Op::InNull(n.parse().unwrap()),
            ("jmp", [rest]) => match rest.split_once(char::is_whitespace) {
                Some(("pin", t)) => Op::JmpPin(target(t)),
                Some(("x--", t)) => Op::JmpXDec(target(t)),
                None => Op::Jmp(target(rest)),
                _ => panic!("unsupported jmp {l}"),
            },
            _ => panic!("unsupported instruction {l}"),
        };
        ops.push(op);
    }
    Program { ops }
}

/// Run `program` for `cycles` cycles with `pin(t)` giving the WR level at cycle `t`.
/// Returns the captured cells (one bool per bit shifted in), in order, including only bits
/// that reached a completed 32-bit push.
pub fn run(program: &Program, cycles: u64, pin: impl Fn(u64) -> bool) -> Vec<bool> {
    let (mut pc, mut x, mut y) = (0usize, 0u32, 0u32);
    let mut isr = 0u32;
    let mut count = 0;
    let mut out = vec![];
    let mut shift_in = |bits: u32, n: u32, out: &mut Vec<bool>| {
        for i in (0..n).rev() {
            isr = isr << 1 | (bits >> i & 1);
            count += 1;
            if count == 32 {
                for k in (0..32).rev() {
                    out.push(isr >> k & 1 != 0);
                }
                count = 0;
                isr = 0;
            }
        }
    };
    for t in 0..cycles {
        let synced = if t >= 2 { pin(t - 2) } else { true };
        let mut next = pc + 1;
        match program.ops[pc] {
            Op::SetX(n) => x = n,
            Op::SetY(n) => y = n,
            Op::Jmp(a) => next = a,
            Op::JmpPin(a) => {
                if synced {
                    next = a;
                }
            }
            Op::JmpXDec(a) => {
                if x != 0 {
                    next = a;
                }
                x = x.wrapping_sub(1);
            }
            Op::InY(n) => shift_in(y, n, &mut out),
            Op::InNull(n) => shift_in(0, n, &mut out),
        }
        pc = next;
        assert!(pc < program.ops.len(), "fell off the end of the program");
    }
    out
}
