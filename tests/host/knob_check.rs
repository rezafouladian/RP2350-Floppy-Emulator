//! Host-side check of the KY-040 decoder in `src/knob.rs`: detents in both directions at
//! various speeds, contact bounce, missed samples, and the push switch.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/knob_check.rs -o target/knob_check && target/knob_check

#[path = "../../src/knob.rs"]
mod knob;

use knob::{Knob, PRESS_MS};

/// `CLK DT` states of one clockwise detent after leaving rest; anticlockwise is the reverse.
const CW: [(bool, bool); 4] = [(false, true), (false, false), (true, false), (true, true)];
const CCW: [(bool, bool); 4] = [(true, false), (false, false), (false, true), (true, true)];

struct Sim {
    knob: Knob,
    pins: (bool, bool),
    sw: bool,
    detents: i32,
    presses: u32,
    rng: u32,
}

impl Sim {
    fn new() -> Self {
        Self { knob: Knob::new(), pins: (true, true), sw: true, detents: 0, presses: 0, rng: 1 }
    }

    fn sample(&mut self, n: usize) {
        for _ in 0..n {
            let e = self.knob.sample(self.pins.0, self.pins.1, self.sw);
            self.detents += i32::from(e.detent);
            self.presses += u32::from(e.pressed);
        }
    }

    fn random(&mut self) -> u32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        self.rng
    }

    /// Move to `to`, holding it `hold` samples. With `bounce`, the changing pin chatters
    /// between old and new for a few samples first.
    fn go(&mut self, to: (bool, bool), hold: usize, bounce: bool) {
        if bounce {
            let from = self.pins;
            for _ in 0..self.random() % 4 {
                self.pins = to;
                self.sample(1);
                self.pins = from;
                self.sample(1);
            }
        }
        self.pins = to;
        self.sample(hold);
    }

    fn turn(&mut self, states: &[(bool, bool)], hold: usize, bounce: bool) {
        for &s in states {
            self.go(s, hold, bounce);
        }
    }
}

fn main() {
    let mut failures = 0;
    let mut expect = |name: &str, got: i64, want: i64| {
        if got == want {
            println!("ok   {name}: {got}");
        } else {
            println!("FAIL {name}: got {got}, want {want}");
            failures += 1;
        }
    };

    for hold in [1, 2, 5, 30] {
        let mut sim = Sim::new();
        for _ in 0..10 {
            sim.turn(&CW, hold, false);
        }
        expect(&format!("10 clockwise detents, {hold} ms per state"), sim.detents.into(), 10);
        for _ in 0..4 {
            sim.turn(&CCW, hold, false);
        }
        expect(&format!("then 4 anticlockwise, {hold} ms per state"), sim.detents.into(), 6);
    }

    let mut sim = Sim::new();
    for _ in 0..500 {
        sim.turn(&CW, 2, true);
    }
    for _ in 0..200 {
        sim.turn(&CCW, 3, true);
    }
    expect("500 cw, 200 ccw, with contact bounce", sim.detents.into(), 300);

    // A sample that misses a state: 11 → 01 → 10 (00 skipped) → 11.
    let mut sim = Sim::new();
    sim.turn(&[(false, true), (true, false), (true, true)], 3, false);
    expect("clockwise with a missed state", sim.detents.into(), 1);

    // Half a detent and back, as when the knob is nudged without clicking over.
    let mut sim = Sim::new();
    sim.turn(&[(false, true), (false, false), (false, true), (true, true)], 5, true);
    expect("half a detent and back", sim.detents.into(), 0);
    sim.turn(&[(false, true), (true, true)], 5, true);
    expect("a quarter and back", sim.detents.into(), 0);

    // Switch: bounce on press and release, held for 300 ms.
    let mut sim = Sim::new();
    for _ in 0..3 {
        for _ in 0..5 {
            sim.sw = false;
            sim.sample(1);
            sim.sw = true;
            sim.sample(1);
        }
        sim.sw = false;
        sim.sample(300);
        for _ in 0..5 {
            sim.sw = true;
            sim.sample(1);
            sim.sw = false;
            sim.sample(1);
        }
        sim.sw = true;
        sim.sample(200);
    }
    expect("3 bouncy presses", sim.presses.into(), 3);

    let mut sim = Sim::new();
    sim.sw = false;
    sim.sample(PRESS_MS as usize - 1);
    sim.sw = true;
    sim.sample(100);
    expect("a glitch shorter than PRESS_MS", sim.presses.into(), 0);
    sim.sw = false;
    sim.sample(5_000);
    expect("held for 5 s counts once", sim.presses.into(), 1);

    if failures > 0 {
        println!("{failures} FAILED");
        std::process::exit(1);
    }
    println!("all passed");
}
