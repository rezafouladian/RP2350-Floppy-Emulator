//! An independent Disk II track decoder for the host tests (`a2_check`, `a2r_check`): 6-and-2
//! data fields after their address fields, framed as a Disk II does.

/// IWM-style framing of a loop of cells, twice around: every address field and data field,
/// decoded independently of `writes::a2_fields`. Returns (track, sector, volume, data) per
/// sector whose address and data fields are both intact.
pub fn decode_track(cells: &[bool]) -> Vec<(u8, u8, u8, [u8; 256])> {
    let n = cells.len();
    let mut nibbles = vec![];
    let mut sh = 0u8;
    for i in 0..2 * n {
        sh = sh << 1 | cells[i % n] as u8;
        if sh & 0x80 != 0 {
            nibbles.push((sh, i));
            sh = 0;
        }
    }
    let mut out: Vec<(u8, u8, u8, [u8; 256])> = vec![];
    let d44 = |a: u8, b: u8| (a << 1 | 1) & b;
    let mut i = 0;
    while i + 3 < nibbles.len() {
        let w = (nibbles[i].0, nibbles[i + 1].0, nibbles[i + 2].0);
        if w == (0xD5, 0xAA, 0x96) && i + 11 < nibbles.len() {
            let a = |k: usize| d44(nibbles[i + 3 + 2 * k].0, nibbles[i + 4 + 2 * k].0);
            let (vol, trk, sec, chk) = (a(0), a(1), a(2), a(3));
            if vol ^ trk ^ sec == chk {
                // The data mark within 40 nibbles.
                for j in i + 11..(i + 51).min(nibbles.len().saturating_sub(346)) {
                    if (nibbles[j].0, nibbles[j + 1].0, nibbles[j + 2].0) == (0xD5, 0xAA, 0xAD) {
                        let mut v = [0u8; 343];
                        let mut prev = 0u8;
                        let mut ok = true;
                        for k in 0..343 {
                            match crate::gcr::decode6(nibbles[j + 3 + k].0) {
                                Some(x) if k < 342 => {
                                    prev ^= x;
                                    v[k] = prev;
                                }
                                Some(x) => ok &= x == prev,
                                None => ok = false,
                            }
                        }
                        if ok && nibbles[i].1 < n {
                            let mut data = [0u8; 256];
                            for (b, byte) in data.iter_mut().enumerate() {
                                let low = v[b % 86] >> (2 * (b / 86)) & 3;
                                *byte = v[86 + b] << 2 | [0, 2, 1, 3][low as usize];
                            }
                            out.push((trk, sec, vol, data));
                        }
                        break;
                    }
                }
            }
        }
        i += 1;
    }
    out
}
