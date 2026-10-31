//! Disk images made at test time, so the host tests need no sample images. Each is random sector
//! data laid out the way real disks are (not the way the firmware builds its tracks), with the
//! traits of the real disks the tests were first written against:
//!
//! - Mac GCR (`mac_disk`, `mac_track`): 400K/800K, the Mac's 2:1 or the IIgs's 4:1 interleave,
//!   gaps of 10-bit `FF` sync, 10-bit `AA`s (Dollars and Sense) or a repeating pattern
//!   (`FD EA AF DE AA`, SimCity), and per-sector oddities (`Odd`): a missing data field, a damaged
//!   data mark (`D5 CC AD`, Macintosh Pascal), a bad data checksum (The Oregon Trail's IIgs
//!   disk), timing records each after 16 no-flux cells (Skyfox `B9 CD`…`AB 99`, MacWars `CB D9`…
//!   `E9 BA`), a long no-flux stretch (Q-Sheet, Deluxe Music).
//! - `aaow`: The Ancient Art of War's cross-track protection (track 79 has no sectors, only a
//!   nibble pattern and `AA FF FF` at a set angle from track 78's sector 0), synchronized.
//! - `oids`: an 800K MOOF whose track 1 side 0 is flux with fuzzy reversals in sector 5's data
//!   field, its FLUX chunk in the file's partial last block (Oids).
//! - `labelwriter`: an 800K MOOF at Fast Imager's standard track lengths and an A2R 3 capture
//!   of the same disk as its duplicator wrote it (longer tracks) on a drive off speed.
//! - MFM (`mfm_track`, `mfm_disk`): 1.44 MB tracks of 192,992 half-cells (as `Install.moof`).
//! - Containers: MOOF, 3.5" and 5.25" WOZ 2 (`container`), DiskCopy 4.2 with or without tags
//!   (`diskcopy`) and `.2mg` (`two_mg`).
//! - Apple II 5.25" (`a2_track`, `dos33_image`, `prodos_image`, `woz525`): DOS 3.3 with its VTOC
//!   and catalog chain, ProDOS with its volume directory, 6-and-2.
//!
//! The Mac's 6-and-2 sector body and address field are `gcr`'s (`track_check` checks them against
//! reference vectors); everything else here is written independently of the firmware.

use crate::gcr;
use crate::track::BLOCK;

pub fn rng(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

pub fn random_bytes(n: usize, seed: u64) -> Vec<u8> {
    let mut s = seed | 1;
    (0..n).map(|_| rng(&mut s) as u8).collect()
}

/// Cells being laid down, MSB first.
#[derive(Clone, Default)]
pub struct Cells(pub Vec<bool>);

impl Cells {
    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn zeros(&mut self, n: usize) {
        self.0.extend(std::iter::repeat_n(false, n));
    }

    /// A nibble led by `lead` 0s.
    pub fn nib(&mut self, lead: usize, b: u8) {
        self.zeros(lead);
        self.0.extend((0..8).rev().map(|k| b >> k & 1 != 0));
    }

    pub fn nibs(&mut self, bs: &[u8]) {
        for &b in bs {
            self.nib(0, b);
        }
    }

    /// `n` 10-bit self-sync `FF`s.
    pub fn sync(&mut self, n: usize) {
        for _ in 0..n {
            self.nib(0, 0xFF);
            self.zeros(2);
        }
    }

    /// Exactly `k` cells of `b` nibbles, each led by 0, 1 or 2 0s (`k` ≥ 32, or most others).
    pub fn fill(&mut self, b: u8, k: usize) {
        let (long, mid, short) = (0..=k / 10)
            .rev()
            .find_map(|c| {
                let r = k - 10 * c;
                (0..=r / 9).find(|&m| (r - 9 * m) % 8 == 0).map(|m| (c, m, (r - 9 * m) / 8))
            })
            .unwrap_or_else(|| panic!("{k} cells can't be whole nibbles"));
        for (n, lead) in [(short, 0), (mid, 1), (long, 2)] {
            for _ in 0..n {
                self.nib(lead, b);
            }
        }
    }
}

/// Cells packed MSB first, as MOOF and WOZ store them.
pub fn pack(cells: &[bool]) -> Vec<u8> {
    let mut b = vec![0u8; cells.len().div_ceil(8)];
    for (i, &c) in cells.iter().enumerate() {
        b[i / 8] |= (c as u8) << (7 - i % 8);
    }
    b
}

/// Nibbles framed as an IWM does, from cell 0 over `cells` once: (nibble, cell just after it).
pub fn frame(cells: &[bool]) -> Vec<(u8, usize)> {
    let mut out = vec![];
    let mut sh = 0u8;
    for (i, &c) in cells.iter().enumerate() {
        sh = sh << 1 | c as u8;
        if sh & 0x80 != 0 {
            out.push((sh, i + 1));
            sh = 0;
        }
    }
    out
}

// ---- Mac GCR -------------------------------------------------------------------------------

/// What a gap is filled with.
#[derive(Clone, Copy)]
pub enum Fill {
    /// 10-bit `FF` self-sync, as most disks.
    Sync,
    /// 10-bit `AA`s (Dollars and Sense).
    Aa,
    /// A repeating run of 8-bit nibbles (SimCity's `FD EA AF DE AA`).
    Pattern(&'static [u8]),
}

/// A sector's place on the track, besides the usual address and data fields.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Odd {
    Normal,
    /// The address field and no data field: fill where it would be.
    NoData,
    /// The data mark damaged: `D5 CC AD` (Macintosh Pascal's track 1).
    BadMark,
    /// The data field's checksum wrong, its nibbles all legal and its trailer right.
    BadSum,
    /// In place of the data field, timing records: `mark`, a 4-and-4 counter and `tail`, each
    /// followed by 16 no-flux cells (Skyfox, MacWars).
    Records { mark: [u8; 2], tail: [u8; 2] },
    /// In place of the data field, a stretch of this many no-flux cells, then fill (Q-Sheet,
    /// Deluxe Music).
    NoFlux(usize),
}

/// A data field's cells: mark, sector, 703 nibbles, `DE AA FF`.
const DATA_FIELD_CELLS: usize = 710 * 8;

/// A gap's nibbles in turn; a pattern goes on where the last gap left it.
struct Gap {
    fill: Fill,
    at: usize,
}

impl Gap {
    fn put(&mut self, c: &mut Cells, units: usize) {
        for _ in 0..units {
            match self.fill {
                Fill::Sync => c.sync(1),
                Fill::Aa => c.nib(2, 0xAA),
                Fill::Pattern(p) => {
                    c.nib(0, p[self.at % p.len()]);
                    self.at += 1;
                }
            }
        }
    }

    /// Fill up to `cells` long: units, then whole nibbles to the cell.
    fn to(&mut self, c: &mut Cells, cells: usize) {
        while cells - c.len() > 48 {
            self.put(c, 1);
        }
        let b = if let Fill::Aa = self.fill { 0xAA } else { 0xFF };
        c.fill(b, cells - c.len());
    }
}

/// A Mac GCR track side: physical sectors in `order`, each led by `gap` fill units (the first
/// also by the end gap, which is `end` cells), address field, an `FF` and 5 fill units, data
/// field. `sector(s)` gives sector `s`'s 524 bytes (tags first).
pub fn mac_track(
    track: u8,
    side: u8,
    format: u8,
    order: &[u8],
    fill: Fill,
    gap: usize,
    end: usize,
    sector: impl Fn(u8) -> [u8; 524],
    odd: impl Fn(u8) -> Odd,
) -> Vec<bool> {
    let mut c = Cells::default();
    let mut g = Gap { fill, at: 0 };
    for &s in order {
        g.put(&mut c, gap);
        c.nibs(&[0xD5, 0xAA, 0x96]);
        c.nibs(&gcr::address_field(track, s, side, format));
        c.nibs(&[0xDE, 0xAA, 0xFF]);
        g.put(&mut c, 5);
        let start = c.len();
        let d = sector(s);
        let mut body = vec![];
        gcr::encode_sector(d[..12].try_into().unwrap(), &d[12..], |n| body.push(n));
        match odd(s) {
            Odd::Normal | Odd::BadMark | Odd::BadSum => {
                let o = odd(s);
                c.nibs(if o == Odd::BadMark { &[0xD5, 0xCC, 0xAD] } else { &[0xD5, 0xAA, 0xAD] });
                c.nib(0, gcr::ENCODE[s as usize]);
                if o == Odd::BadSum {
                    let k = body.len() - 1;
                    body[k] = gcr::ENCODE[(gcr::decode6(body[k]).unwrap() as usize + 1) % 64];
                }
                c.nibs(&body);
                c.nibs(&[0xDE, 0xAA, 0xFF]);
            }
            Odd::NoData => g.to(&mut c, start + DATA_FIELD_CELLS),
            Odd::Records { mark, tail } => {
                // A few nibbles first, as Skyfox has, then records until the field's room is
                // used.
                c.nibs(&[0xAB, 0xCD, 0xEF, 0xEF, 0xFF, 0xAF, 0xAF, 0xEF, 0xFE, 0xBF]);
                let mut n = 0u8;
                while c.len() + 64 + 16 <= start + DATA_FIELD_CELLS {
                    c.zeros(16);
                    c.nibs(&mark);
                    c.nibs(&[n >> 1 | 0xAA, n | 0xAA]);
                    c.nibs(&tail);
                    n += 1;
                }
                g.to(&mut c, start + DATA_FIELD_CELLS);
            }
            Odd::NoFlux(k) => {
                c.zeros(k);
                g.to(&mut c, start + DATA_FIELD_CELLS);
            }
        }
    }
    let total = c.len() + end;
    g.to(&mut c, total);
    c.0
}

/// Physical sector order at `factor`:1 interleave: sector s goes in slot factor·s mod n, or the
/// next free one.
pub fn interleave(spt: usize, factor: usize) -> Vec<u8> {
    let mut slots = vec![None; spt];
    for s in 0..spt {
        let mut k = factor * s % spt;
        while slots[k].is_some() {
            k = (k + 1) % spt;
        }
        slots[k] = Some(s as u8);
    }
    slots.into_iter().map(Option::unwrap).collect()
}

/// A disk's track sides and what they hold.
pub struct MacDisk {
    pub sides: u8,
    /// The address fields' format byte.
    pub format: u8,
    /// The sector data in block order: what decodes back.
    pub raw: Vec<u8>,
    /// Per track × 2 + side; empty for a blank side.
    pub tracks: Vec<Vec<bool>>,
    pub mfm: bool,
    pub write_protected: bool,
    pub synchronized: bool,
}

/// A GCR disk of `raw` (400K or 800K, block order): every track side by `mac_track`, the end gap
/// `end(track, side)` cells.
pub fn mac_disk(
    raw: &[u8],
    apple2: bool,
    fill: Fill,
    end: impl Fn(u8, u8) -> usize,
    odd: impl Fn(u8, u8, u8) -> Odd,
) -> MacDisk {
    let sides = (raw.len() / 409_600) as u8;
    assert!(raw.len() == sides as usize * 409_600 && (1..=2).contains(&sides));
    let format = gcr::format(sides, apple2);
    let mut tracks = vec![vec![]; 160];
    for t in 0..80u8 {
        let spt = gcr::sectors_per_track(t) as usize;
        let order = interleave(spt, if apple2 { 4 } else { 2 });
        for side in 0..sides {
            let first = gcr::first_block(t, sides) + side as usize * spt;
            let sector = |s: u8| {
                let mut d = [0u8; 524];
                d[12..].copy_from_slice(&raw[(first + s as usize) * BLOCK..][..BLOCK]);
                d
            };
            tracks[2 * t as usize + side as usize] = mac_track(t, side, format, &order, fill, 42, end(t, side), sector, |s| odd(t, side, s));
        }
    }
    MacDisk { sides, format, raw: raw.to_vec(), tracks, mfm: false, write_protected: false, synchronized: true }
}

/// End gaps a little different on every track, as on a duplicated disk.
pub fn varied_ends(seed: u64) -> impl Fn(u8, u8) -> usize {
    move |t, side| {
        let mut s = seed ^ (u64::from(t) << 8 | u64::from(side)).wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
        rng(&mut s);
        200 + (rng(&mut s) % 500) as usize
    }
}

/// Macintosh Pascal: 400K, write-protected, track 1 sector 3's data mark damaged (799 of 800
/// data fields decode).
pub fn pascal() -> MacDisk {
    let raw = random_bytes(409_600, 0xA5CA1);
    let mut d = mac_disk(&raw, false, Fill::Sync, varied_ends(1), |t, _, s| if (t, s) == (1, 3) { Odd::BadMark } else { Odd::Normal });
    d.write_protected = true;
    d
}

/// Skyfox: 400K, track 1 sector 8 has an address field and in place of its data field ~85
/// timing records, each after 16 no-flux cells (799 of 800).
pub fn skyfox() -> MacDisk {
    let raw = random_bytes(409_600, 0x5C7F0);
    let records = Odd::Records { mark: [0xB9, 0xCD], tail: [0xAB, 0x99] };
    let mut d = mac_disk(&raw, false, Fill::Sync, varied_ends(2), |t, _, s| if (t, s) == (1, 8) { records } else { Odd::Normal });
    d.write_protected = true;
    d
}

/// Dollars and Sense: 400K, gaps of 10-bit `AA`s, track 1 sector 10 without a data field (799).
pub fn dollars() -> MacDisk {
    let raw = random_bytes(409_600, 0xD011A5);
    mac_disk(&raw, false, Fill::Aa, varied_ends(3), |t, _, s| if (t, s) == (1, 10) { Odd::NoData } else { Odd::Normal })
}

/// SimCity: 800K, gaps of a repeating `FD EA AF DE AA` (a data trailer's `DE AA` in every gap).
pub fn simcity() -> MacDisk {
    let raw = random_bytes(819_200, 0x51AC17);
    mac_disk(&raw, false, Fill::Pattern(&[0xFD, 0xEA, 0xAF, 0xDE, 0xAA]), varied_ends(4), |_, _, _| Odd::Normal)
}

/// MacWars and Q-Sheet: 400K, track 14 sector 4 holds timing records (`CB D9`, counter, `E9 BA`
/// after 16 no-flux cells), track 41 sector 2 a 33-cell no-flux stretch after its address field
/// (798 of 800).
pub fn macwars() -> MacDisk {
    let raw = random_bytes(409_600, 0x3AC3A5);
    let records = Odd::Records { mark: [0xCB, 0xD9], tail: [0xE9, 0xBA] };
    mac_disk(&raw, false, Fill::Sync, varied_ends(5), |t, _, s| match (t, s) {
        (14, 4) => records,
        (41, 2) => Odd::NoFlux(33),
        _ => Odd::Normal,
    })
}

/// The tracks sides with weak runs (no-flux stretches of 8 cells or more) in the disks above.
pub const WEAK_SIDES: [(&str, u8, u8); 3] = [("Skyfox", 1, 0), ("MacWars", 14, 0), ("MacWars", 41, 0)];

/// The Ancient Art of War: 400K, synchronized, tracks 0–78 normal, track 79 with no sectors:
/// a nibble pattern (no `FF` run longer than the 3 before the mark, so the rotation falls just
/// before it) and one `AA FF FF`, placed so that 41.95 ms after track 78's sector 0 address field
/// (the protection's step and wait) about 700 nibbles go by before it. A revolution of 79 is
/// ~5,670 nibbles. Its check wants 512–895, then 5,376–5,807 per revolution.
pub fn aaow() -> MacDisk {
    let raw = random_bytes(409_600, 0xAA0E);
    let mut d = mac_disk(&raw, false, Fill::Sync, varied_ends(6), |_, _, _| Odd::Normal);
    d.write_protected = true;
    let t78 = &d.tracks[156];
    // The end of track 78 sector 0's address field.
    let nib = frame(t78);
    let t0 = (0..nib.len() - 10)
        .find(|&k| nib[k].0 == 0xD5 && nib[k + 1].0 == 0xAA && nib[k + 2].0 == 0x96 && nib[k + 4].0 == gcr::ENCODE[0])
        .map(|k| nib[k + 9].1)
        .expect("sector 0");
    // Track 79: the run before the mark, the mark, then A B … A.
    let mut c = Cells::default();
    c.nib(1, 0xFF);
    c.nib(2, 0xFF);
    c.nib(2, 0xFF);
    let mark = c.len();
    c.nibs(&[0xAA, 0xFF, 0xFF]);
    let a = |c: &mut Cells| {
        c.nib(1, 0xDF);
        c.nibs(&[0xF9, 0xE7, 0xFF, 0xE7, 0xFE]);
    };
    for _ in 0..333 {
        a(&mut c);
        for (lead, b) in [(3, 0xFF), (0, 0xCF), (2, 0xFF), (0, 0xFC), (0, 0xFF), (0, 0xDB), (0, 0xFF), (2, 0xF3), (0, 0xFF), (0, 0xF3), (0, 0xFF)] {
            c.nib(lead, b);
        }
    }
    a(&mut c);
    let (n78, n79) = (t78.len() as f64, c.len() as f64);
    let ms = |cells: f64| cells * 2.0425e-3;
    // Published halfway through the protection's wait; the angle carries over as a fraction.
    let f = t0 as f64 / n78 + 20.0 / ms(n78) + (54.1 - 20.0) / ms(n79);
    let at = (f.fract() * n79) as usize;
    let n = c.len();
    d.tracks[158] = (0..n).map(|i| c.0[(i + n - at + mark) % n]).collect();
    d.synchronized = true;
    d
}

// ---- MFM -----------------------------------------------------------------------------------

pub fn crc16(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { crc << 1 ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

struct Mfm {
    c: Cells,
    last: bool,
}

impl Mfm {
    fn byte(&mut self, b: u8) {
        for k in (0..8).rev() {
            let d = b >> k & 1 != 0;
            self.c.0.push(!self.last && !d);
            self.c.0.push(d);
            self.last = d;
        }
    }

    fn bytes(&mut self, b: u8, n: usize) {
        for _ in 0..n {
            self.byte(b);
        }
    }

    /// A mark byte with a missing clock: `A1` as 4489, `C2` as 5224.
    fn raw(&mut self, w: u16) {
        self.c.0.extend((0..16).rev().map(|k| w >> k & 1 != 0));
        self.last = w & 1 != 0;
    }
}

/// An MFM track side of `sectors` sectors (1-based), `cells` half-cells long: gap 4a, index mark,
/// gap 1, the sectors 1:1 (ID, gap 2, data, gap 3), gap 4b to the end.
pub fn mfm_track<'a>(track: u8, side: u8, sectors: u8, gap3: usize, cells: usize, data: impl Fn(u8) -> &'a [u8]) -> Vec<bool> {
    let mut m = Mfm { c: Cells::default(), last: false };
    m.bytes(0x4E, 80);
    m.bytes(0, 12);
    for _ in 0..3 {
        m.raw(0x5224);
    }
    m.byte(0xFC);
    m.bytes(0x4E, 50);
    for s in 1..=sectors {
        m.bytes(0, 12);
        for _ in 0..3 {
            m.raw(0x4489);
        }
        let id = [0xA1, 0xA1, 0xA1, 0xFE, track, side, s, 2];
        let crc = crc16(0xFFFF, &id);
        for &b in &id[3..] {
            m.byte(b);
        }
        m.byte((crc >> 8) as u8);
        m.byte(crc as u8);
        m.bytes(0x4E, 22);
        m.bytes(0, 12);
        for _ in 0..3 {
            m.raw(0x4489);
        }
        m.byte(0xFB);
        let d = data(s);
        let mut all = vec![0xA1, 0xA1, 0xA1, 0xFB];
        all.extend_from_slice(d);
        let crc = crc16(0xFFFF, &all);
        for &b in d {
            m.byte(b);
        }
        m.byte((crc >> 8) as u8);
        m.byte(crc as u8);
        m.bytes(0x4E, gap3);
    }
    assert!(m.c.len() <= cells && (cells - m.c.len()) % 16 == 0, "{} half-cells don't fit {cells}", m.c.len());
    while m.c.len() < cells {
        m.byte(0x4E);
    }
    m.c.0
}

/// A 1.44 MB disk of `raw`, every track side 192,992 half-cells (as `Install.moof`).
pub fn mfm_disk(raw: &[u8]) -> MacDisk {
    assert_eq!(raw.len(), 1_474_560);
    let mut tracks = vec![];
    for t in 0..80u8 {
        for side in 0..2u8 {
            let first = (t as usize * 2 + side as usize) * 18;
            tracks.push(mfm_track(t, side, 18, 84, 192_992, |s| &raw[(first + s as usize - 1) * BLOCK..][..BLOCK]));
        }
    }
    MacDisk { sides: 2, format: 0x22, raw: raw.to_vec(), tracks, mfm: true, write_protected: false, synchronized: false }
}

// ---- Containers ----------------------------------------------------------------------------

/// A track's data: cells, or flux bytes (125 ns ticks).
pub enum Trk {
    Bits(Vec<bool>),
    Flux(Vec<u8>),
}

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    /// INFO's disk type: 1 400K, 2 800K, 3 1.44 MB.
    Moof(u8),
    Woz35,
    /// The optimal bit timing in 125 ns units.
    Woz525(u8),
}

/// A MOOF or WOZ 2: header, INFO, TMAP, TRKS (one record per entry of `trks`), the tracks from
/// block 3, and, if any track is flux, a FLUX chunk at the end, in a partial last block.
/// `map[i]` is the record played at TMAP entry `i` (track × 2 + side, or a WOZ's quarter
/// track), 0xFF for none. The CRC covers everything after it.
pub fn container(kind: Kind, wp: bool, synchronized: bool, map: &[u8; 160], trks: &[Trk]) -> Vec<u8> {
    let mut f = vec![0u8; 3 * BLOCK];
    f[..8].copy_from_slice(if let Kind::Moof(_) = kind { b"MOOF\xFF\x0A\x0D\x0A" } else { b"WOZ2\xFF\x0A\x0D\x0A" });
    f[12..16].copy_from_slice(b"INFO");
    f[16..20].copy_from_slice(&60u32.to_le_bytes());
    let any_flux = trks.iter().any(|t| matches!(t, Trk::Flux(_)));
    f[22] = wp as u8;
    f[23] = synchronized as u8;
    let creator = b"rp2350-floppy host tests";
    f[25..57].fill(b' ');
    f[25..25 + creator.len()].copy_from_slice(creator);
    match kind {
        Kind::Moof(t) => {
            f[20] = 1;
            f[21] = t;
            f[24] = if t == 3 { 8 } else { 16 };
        }
        Kind::Woz35 | Kind::Woz525(_) => {
            f[20] = if any_flux { 3 } else { 2 };
            f[21] = if kind == Kind::Woz35 { 2 } else { 1 };
            f[57] = if kind == Kind::Woz35 { 2 } else { 1 };
            f[59] = if let Kind::Woz525(b) = kind { b } else { 16 };
        }
    }
    f[80..84].copy_from_slice(b"TMAP");
    f[84..88].copy_from_slice(&160u32.to_le_bytes());
    f[88..248].fill(0xFF);
    f[248..252].copy_from_slice(b"TRKS");
    let mut flux_map = [0xFFu8; 160];
    for (i, &k) in map.iter().enumerate() {
        match trks.get(k as usize) {
            Some(Trk::Bits(_)) => f[88 + i] = k,
            Some(Trk::Flux(_)) => flux_map[i] = k,
            None => {}
        }
    }
    let (mut largest, mut largest_flux) = (0u16, 0u16);
    for (k, t) in trks.iter().enumerate() {
        let (bytes, count) = match t {
            Trk::Bits(c) => (pack(c), c.len() as u32),
            Trk::Flux(b) => (b.clone(), b.len() as u32),
        };
        let start = f.len() / BLOCK;
        let blocks = bytes.len().div_ceil(BLOCK) as u16;
        f.extend(&bytes);
        f.resize((start + blocks as usize) * BLOCK, 0);
        let r = 256 + 8 * k;
        f[r..r + 2].copy_from_slice(&(start as u16).to_le_bytes());
        f[r + 2..r + 4].copy_from_slice(&blocks.to_le_bytes());
        f[r + 4..r + 8].copy_from_slice(&count.to_le_bytes());
        if let Trk::Flux(_) = t {
            largest_flux = largest_flux.max(blocks);
        } else {
            largest = largest.max(blocks);
        }
    }
    let trks_size = (f.len() - 256) as u32;
    f[252..256].copy_from_slice(&trks_size.to_le_bytes());
    let (largest_at, flux_at) = if let Kind::Moof(_) = kind { (58, 60) } else { (64, 66) };
    f[largest_at..largest_at + 2].copy_from_slice(&largest.to_le_bytes());
    if any_flux {
        let block = (f.len() / BLOCK) as u16;
        f[flux_at..flux_at + 2].copy_from_slice(&block.to_le_bytes());
        f[flux_at + 2..flux_at + 4].copy_from_slice(&largest_flux.to_le_bytes());
        f.extend_from_slice(b"FLUX");
        f.extend_from_slice(&160u32.to_le_bytes());
        f.extend_from_slice(&flux_map);
    }
    let crc = crc32(&f[12..]);
    f[8..12].copy_from_slice(&crc.to_le_bytes());
    f
}

pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 { crc >> 1 ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

/// A Mac disk as a MOOF (or a 3.5" WOZ), every side bitstream; side 1 of a 400K disk blank.
pub fn moof(d: &MacDisk) -> Vec<u8> {
    let kind = if d.mfm { Kind::Moof(3) } else { Kind::Moof(d.sides) };
    moof_as(kind, d)
}

pub fn woz35(d: &MacDisk) -> Vec<u8> {
    moof_as(Kind::Woz35, d)
}

fn moof_as(kind: Kind, d: &MacDisk) -> Vec<u8> {
    let mut map = [0xFFu8; 160];
    let mut trks = vec![];
    for (i, t) in d.tracks.iter().enumerate() {
        if !t.is_empty() {
            map[i] = trks.len() as u8;
            trks.push(Trk::Bits(t.clone()));
        }
    }
    container(kind, d.write_protected, d.synchronized, &map, &trks)
}

/// DiskCopy's checksum: add each big-endian word, rotate right one.
pub fn dc42_sum(data: &[u8]) -> u32 {
    data.chunks(2).fold(0u32, |s, w| s.wrapping_add(u32::from(u16::from_be_bytes([w[0], w[1]]))).rotate_right(1))
}

/// A DiskCopy 4.2 image of `raw`, with 12 tag bytes a sector if `tags` (the tag checksum leaves
/// out the first sector's).
pub fn diskcopy(raw: &[u8], tags: Option<&[u8]>, name: &str) -> Vec<u8> {
    let format = match raw.len() {
        409_600 => 0,
        819_200 => 1,
        737_280 => 2,
        1_474_560 => 3,
        n => panic!("{n} bytes isn't a floppy size"),
    };
    let mut h = vec![0u8; 84];
    h[0] = name.len() as u8;
    h[1..1 + name.len()].copy_from_slice(name.as_bytes());
    h[0x40..0x44].copy_from_slice(&(raw.len() as u32).to_be_bytes());
    h[0x44..0x48].copy_from_slice(&(tags.map_or(0, |t| t.len()) as u32).to_be_bytes());
    h[0x48..0x4C].copy_from_slice(&dc42_sum(raw).to_be_bytes());
    if let Some(t) = tags {
        assert_eq!(t.len(), raw.len() / BLOCK * 12);
        h[0x4C..0x50].copy_from_slice(&dc42_sum(&t[12..]).to_be_bytes());
    }
    h[0x50] = format;
    h[0x51] = match format {
        0 => 0x02,
        1 => 0x22,
        _ => 0x22,
    };
    h[0x52..0x54].copy_from_slice(&0x0100u16.to_be_bytes());
    h.extend_from_slice(raw);
    h.extend_from_slice(tags.unwrap_or(&[]));
    h
}

/// A `.2mg` of ProDOS-order blocks.
pub fn two_mg(blocks: &[u8]) -> Vec<u8> {
    let mut h = vec![0u8; 64];
    h[..4].copy_from_slice(b"2IMG");
    h[4..8].copy_from_slice(b"RP23");
    h[8..10].copy_from_slice(&64u16.to_le_bytes());
    h[10..12].copy_from_slice(&1u16.to_le_bytes());
    h[12..16].copy_from_slice(&1u32.to_le_bytes()); // ProDOS order
    h[20..24].copy_from_slice(&((blocks.len() / BLOCK) as u32).to_le_bytes());
    h[24..28].copy_from_slice(&64u32.to_le_bytes());
    h[28..32].copy_from_slice(&(blocks.len() as u32).to_le_bytes());
    h.extend_from_slice(blocks);
    h
}

// ---- Flux ----------------------------------------------------------------------------------

pub fn push_ticks(out: &mut Vec<u8>, mut t: u64) {
    while t >= 255 {
        out.push(255);
        t -= 255;
    }
    out.push(t as u8);
}

/// Where a reversal goes.
pub enum At {
    /// At its cell, timed from the start, `shift` cells late and ±`jitter` ticks.
    Grid { shift: f64, jitter: u64 },
    /// This many cells after the last reversal.
    After(f64),
}

/// Flux bytes of `count` cells of the loop `cells` from cell `from`, `cell_ticks` ticks a cell,
/// each reversal placed as `at(cell)` says, to the tick. With `looped`, the first interval runs
/// from the last reversal before `from`, as a MOOF FLUX track's does; otherwise from the
/// capture's start.
pub fn flux(cells: &[bool], from: usize, count: usize, cell_ticks: f64, looped: bool, seed: &mut u64, at: impl Fn(usize) -> At) -> Vec<u8> {
    let n = cells.len();
    let mut out = vec![];
    let mut last = if looped {
        let back = (1..=n).find(|&k| cells[(from + n - k) % n]).unwrap();
        -((back as f64 - 1.0) * cell_ticks).round() as i64
    } else {
        0
    };
    for k in 0..count {
        let i = (from + k) % n;
        if cells[i] {
            let at = match at(i) {
                At::After(after) => last + (after * cell_ticks).round() as i64,
                At::Grid { shift, jitter } => {
                    let noise = (rng(seed) % (2 * jitter + 1)) as f64 - jitter as f64;
                    (((k + 1) as f64 + shift) * cell_ticks + noise).round() as i64
                }
            };
            let at = at.max(last + 1);
            push_ticks(&mut out, (at - last) as u64);
            last = at;
        }
    }
    out
}

/// Oids: an 800K MOOF, track 1 side 0 flux (125 ns ticks, ±1 tick of jitter) with 62 fuzzy
/// reversals in sector 5's data field: each where 1 then 3 cells go by, moved to 1.52 cells
/// after the one before (25 ticks), on the IWM's boundary, so it reads either way (a fixed
/// rounding breaks the sector). The FLUX chunk is the file's partial last block. Returns the disk (its bitstream tracks) and the file.
pub fn oids() -> (MacDisk, Vec<u8>) {
    let raw = random_bytes(819_200, 0x01D5);
    let mut d = mac_disk(&raw, false, Fill::Sync, varied_ends(7), |_, _, _| Odd::Normal);
    d.write_protected = true;
    let cells = &d.tracks[2];
    // Sector 5's data field: from its data mark to the next address mark.
    let nib = frame(cells);
    let k = (0..nib.len() - 10)
        .find(|&k| nib[k].0 == 0xD5 && nib[k + 1].0 == 0xAA && nib[k + 2].0 == 0x96 && nib[k + 4].0 == gcr::ENCODE[5])
        .expect("sector 5");
    let from = nib[k + 15].1;
    let to = from + 690 * 8;
    let mut fuzzy = vec![];
    let mut c = from;
    while c + 5 < to && fuzzy.len() < 62 {
        if cells[c] && cells[c + 1] && !cells[c + 2] && !cells[c + 3] && cells[c + 4] {
            fuzzy.push(c + 1);
            c += 12;
        } else {
            c += 1;
        }
    }
    assert_eq!(fuzzy.len(), 62);
    let mut s = 0x0D5u64;
    let near = |i: usize| fuzzy.iter().any(|&f| i + 2 >= f && i <= f + 4);
    let at = |i| if fuzzy.contains(&i) { At::After(1.52) } else { At::Grid { shift: 0.0, jitter: if near(i) { 0 } else { 1 } } };
    let bytes = flux(cells, 0, cells.len(), 16.34 * 1.002, true, &mut s, at);
    let mut map = [0xFFu8; 160];
    let mut trks = vec![];
    for (i, t) in d.tracks.iter().enumerate() {
        map[i] = trks.len() as u8;
        trks.push(if i == 2 { Trk::Flux(bytes.clone()) } else { Trk::Bits(t.clone()) });
    }
    let file = container(Kind::Moof(2), true, true, &map, &trks);
    (d, file)
}

/// Peak shift, as a real drive's read head gives it: each reversal pushed away from the nearer
/// of its neighbours, `PEAK_SHIFT` × (1/before − 1/after) cells, so 1-cell gaps come out wide
/// and 3-cell ones narrow. With ±3 ticks of jitter, 65 reversals of `labelwriter`'s capture come
/// out within `FUZZY` (0.44 cell) of the IWM's boundary, as 51 of the real one did; at 0.40,
/// ~17,000.
pub fn peak_shift(cells: &[bool]) -> Vec<f64> {
    let n = cells.len();
    let ones: Vec<usize> = (0..n).filter(|&i| cells[i]).collect();
    let mut shift = vec![0.0; n];
    for (k, &i) in ones.iter().enumerate() {
        let before = (i + n - ones[(k + ones.len() - 1) % ones.len()]) % n;
        let after = (ones[(k + 1) % ones.len()] + n - i) % n;
        shift[i] = PEAK_SHIFT * (1.0 / before as f64 - 1.0 / after as f64);
    }
    shift
}

pub const PEAK_SHIFT: f64 = 0.17;

/// LabelWriter II: an 800K disk as Fast Imager made its MOOF (every track of a zone the same
/// standard length) and as an A2R 3 capture (62.5 ns ticks) of the disk as its duplicator wrote
/// it: the same sectors on tracks 1–4% longer, each captured from a random angle for 1.25
/// revolutions on a drive up to 1% off speed, with peak shift and ±3 ticks of jitter. Returns
/// (disk, MOOF, A2R).
pub fn labelwriter() -> (MacDisk, Vec<u8>, Vec<u8>) {
    let raw = random_bytes(819_200, 0x1AB3);
    let d = mac_disk(&raw, false, Fill::Sync, |_, _| 300, |_, _, _| Odd::Normal);
    let written = mac_disk(&raw, false, Fill::Sync, |t, side| 300 + varied_ends(8)(t, side) * 3, |_, _, _| Odd::Normal);
    let mut s = 0x1AB3u64;
    let mut a2r = b"A2R3\xFF\x0A\x0D\x0A".to_vec();
    let mut rwcp = vec![1u8];
    rwcp.extend_from_slice(&62_500u32.to_le_bytes());
    rwcp.extend_from_slice(&[0; 11]);
    for (loc, cells) in written.tracks.iter().enumerate() {
        let speed = 1.0 + ((rng(&mut s) % 201) as f64 - 100.0) / 10_000.0;
        let cell_ticks = 2.0425 / 0.0625 * speed;
        let from = rng(&mut s) as usize % cells.len();
        let shift = peak_shift(cells);
        let bytes = flux(cells, from, cells.len() * 5 / 4, cell_ticks, false, &mut s, |i| At::Grid { shift: shift[i], jitter: 3 });
        rwcp.push(b'C');
        rwcp.push(1); // timing
        rwcp.extend_from_slice(&(loc as u16).to_le_bytes());
        rwcp.push(1);
        rwcp.extend_from_slice(&((cells.len() as f64 * cell_ticks) as u32).to_le_bytes());
        rwcp.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
        rwcp.extend_from_slice(&bytes);
    }
    rwcp.push(b'X');
    a2r.extend_from_slice(b"RWCP");
    a2r.extend_from_slice(&(rwcp.len() as u32).to_le_bytes());
    a2r.extend_from_slice(&rwcp);
    let m = moof(&d);
    (d, m, a2r)
}

/// The Oregon Trail as a IIgs disk: 800K at 4:1 interleave with the IIgs's format byte, track 0
/// side 0 sector 8's data checksum bad (1,599 of 1,600). Returns the disk, its 3.5" WOZ and a
/// `.2mg` of the same blocks.
pub fn oregon() -> (MacDisk, Vec<u8>, Vec<u8>) {
    let raw = random_bytes(819_200, 0x0E7A11);
    let mut d = mac_disk(&raw, true, Fill::Sync, varied_ends(9), |t, side, s| if (t, side, s) == (0, 0, 8) { Odd::BadSum } else { Odd::Normal });
    d.synchronized = false;
    let w = woz35(&d);
    (d, w, two_mg(&raw))
}

// ---- Apple II 5.25" ------------------------------------------------------------------------

/// 6-and-2: 343 disk nibbles, checksum last.
pub fn encode62(src: &[u8; 256]) -> [u8; 343] {
    let rev = [0u8, 2, 1, 3];
    let mut d = [0u8; 343];
    for c in 0..84 {
        d[c] = rev[(src[c] & 3) as usize] | rev[(src[c + 86] & 3) as usize] << 2 | rev[(src[c + 172] & 3) as usize] << 4;
    }
    d[84] = rev[(src[84] & 3) as usize] | rev[(src[170] & 3) as usize] << 2;
    d[85] = rev[(src[85] & 3) as usize] | rev[(src[171] & 3) as usize] << 2;
    for c in 0..256 {
        d[86 + c] = src[c] >> 2;
    }
    d[342] = d[341];
    let mut at = 342;
    while at > 1 {
        at -= 1;
        d[at] ^= d[at - 1];
    }
    d.map(|v| gcr::ENCODE[v as usize])
}

fn four_and_four(b: u8) -> [u8; 2] {
    [b >> 1 | 0xAA, b | 0xAA]
}

/// A 16-sector track, `cells` long: 40 syncs, then each physical sector's address field, 6
/// syncs, data field and 16 syncs; the rest `FF`s.
pub fn a2_track(track: u8, volume: u8, cells: usize, sector: impl Fn(u8) -> [u8; 256]) -> Vec<bool> {
    let mut c = Cells::default();
    c.sync(40);
    for p in 0..16u8 {
        c.nibs(&[0xD5, 0xAA, 0x96]);
        for b in [volume, track, p, volume ^ track ^ p] {
            c.nibs(&four_and_four(b));
        }
        c.nibs(&[0xDE, 0xAA, 0xEB]);
        c.sync(6);
        c.nibs(&[0xD5, 0xAA, 0xAD]);
        c.nibs(&encode62(&sector(p)));
        c.nibs(&[0xDE, 0xAA, 0xEB]);
        c.sync(16);
    }
    let left = cells - c.len();
    c.fill(0xFF, left);
    c.0
}

/// DOS 3.3's logical sector at each physical one is the inverse of this: the physical sector of
/// logical sector l.
pub const DOS_PHYSICAL: [u8; 16] = [0, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 15];
/// The same for ProDOS order.
pub const PRODOS_PHYSICAL: [u8; 16] = [0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15];

/// A DOS 3.3 disk (DOS order, 35 tracks): random sectors, a VTOC at track 17 sector 0 and the
/// catalog down track 17 from sector 15, each linking to the one below.
pub fn dos33_image() -> Vec<u8> {
    let mut d = random_bytes(143_360, 0xD053);
    let at = |t: usize, s: usize| (t * 16 + s) * 256;
    let v = at(17, 0);
    d[v..v + 256].fill(0);
    d[v + 1] = 17;
    d[v + 2] = 15;
    d[v + 3] = 3;
    d[v + 6] = 254;
    d[v + 0x27] = 122;
    d[v + 0x30] = 17;
    d[v + 0x31] = 1;
    d[v + 0x34] = 35;
    d[v + 0x35] = 16;
    d[v + 0x37] = 1;
    for s in 1..16 {
        let c = at(17, s);
        d[c..c + 256].fill(0);
        if s > 1 {
            d[c + 1] = 17;
            d[c + 2] = s as u8 - 1;
        }
    }
    d
}

/// A ProDOS disk (ProDOS order): random blocks but the volume directory, blocks 2–5, linked.
pub fn prodos_image() -> Vec<u8> {
    let mut d = random_bytes(143_360, 0x9D05);
    for b in 2..6usize {
        let k = &mut d[b * BLOCK..(b + 1) * BLOCK];
        k.fill(0);
        k[..2].copy_from_slice(&(if b == 2 { 0 } else { b as u16 - 1 }).to_le_bytes());
        k[2..4].copy_from_slice(&(if b == 5 { 0 } else { b as u16 + 1 }).to_le_bytes());
    }
    let k = &mut d[2 * BLOCK..3 * BLOCK];
    k[4] = 0xF0 | 6;
    k[5..11].copy_from_slice(b"FLOPPY");
    k[0x23] = 0x27;
    k[0x24] = 0x0D;
    d
}

/// A 5.25" WOZ 2 of a sector image (`prodos`: ProDOS order, else DOS order), every track
/// `cells` long, heard at its quarter track and one either side; `bit_timing` in 125 ns units.
pub fn woz525(img: &[u8], prodos: bool, cells: usize, bit_timing: u8) -> Vec<u8> {
    let physical = if prodos { &PRODOS_PHYSICAL } else { &DOS_PHYSICAL };
    let mut map = [0xFFu8; 160];
    let mut trks = vec![];
    for t in 0..35u8 {
        let sector = |p: u8| {
            let l = physical.iter().position(|&x| x == p).unwrap();
            img[(t as usize * 16 + l) * 256..][..256].try_into().unwrap()
        };
        trks.push(Trk::Bits(a2_track(t, 254, cells, sector)));
        for q in [4 * t as i32 - 1, 4 * t as i32, 4 * t as i32 + 1] {
            if q >= 0 {
                map[q as usize] = t;
            }
        }
    }
    container(Kind::Woz525(bit_timing), false, false, &map, &trks)
}
