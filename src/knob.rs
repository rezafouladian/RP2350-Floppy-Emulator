//! KY-040 rotary encoder with push switch: the image selection control.
//!
//! The pins are sampled every millisecond (core 0's SysTick, see `main.rs`) and fed to
//! [`Knob::sample`]. The module has its own pull-ups, so CLK, DT and SW idle high, and the switch
//! pulls SW low.
//!
//! - **Rotation:** a KY-040 detent is one whole quadrature cycle, from rest (both high) through
//!   all four states and back. Each valid one-state move counts ±1, and a detent is counted on
//!   the return to rest once the count reaches ±2. Contact bounce shows up as moves back and forth
//!   that cancel, and a sample that misses one state still lands on the right side.
//! - **Switch:** a press is SW low for [`PRESS_MS`] samples in a row after it was released.
//!
//! No hardware dependencies, so the host test compiles it directly.

// GPIO header pins. Power the module from 3.3 V (header pin 17), not 5 V: its pull-ups go to its
// + pin, and the RP2350's inputs are only 5 V tolerant while the board is powered.
//
//   KY-040   GPIO  Header pin
//   CLK      22    15
//   DT       23    16
//   SW       24    18
//   +        -     17 (3.3 V)
//   GND      -     14 or 20
pub const PIN_CLK: u8 = 22;
pub const PIN_DT: u8 = 23;
pub const PIN_SW: u8 = 24;

/// Clockwise moves through `CLK DT` = 11 → 01 → 00 → 10 → 11. Indexed by
/// `previous << 2 | current`; a jump across two states (both pins changed) counts 0.
const MOVE: [i8; 16] = [
    0, -1, 1, 0, //  from 00: to 01 is back, to 10 is on
    1, 0, 0, -1, //  from 01: to 00 is on, to 11 is back
    -1, 0, 0, 1, //  from 10: to 00 is back, to 11 is on
    0, 1, -1, 0, //  from 11: to 01 is on, to 10 is back
];

const REST: u8 = 0b11;

/// Samples (ms) the switch must read the same before a change counts.
pub const PRESS_MS: u8 = 20;

pub struct Knob {
    state: u8,
    moves: i8,
    /// Debounced switch level: true = pressed.
    down: bool,
    /// Samples in a row that disagreed with `down`.
    changing: u8,
}

/// What one sample produced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// Detents turned: +1 clockwise, −1 anticlockwise, 0 none.
    pub detent: i8,
    pub pressed: bool,
}

impl Knob {
    pub const fn new() -> Self {
        Self { state: REST, moves: 0, down: false, changing: 0 }
    }

    /// Feed one sample of the pins (true = high).
    pub fn sample(&mut self, clk: bool, dt: bool, sw: bool) -> Event {
        let state = (clk as u8) << 1 | dt as u8;
        self.moves = self.moves.saturating_add(MOVE[(self.state << 2 | state) as usize]);
        self.state = state;
        let mut detent = 0;
        if state == REST {
            if self.moves >= 2 {
                detent = 1;
            } else if self.moves <= -2 {
                detent = -1;
            }
            self.moves = 0;
        }

        let mut pressed = false;
        if !sw == self.down {
            self.changing = 0;
        } else {
            self.changing += 1;
            if self.changing >= PRESS_MS {
                self.changing = 0;
                self.down = !sw;
                pressed = self.down;
            }
        }
        Event { detent, pressed }
    }
}
