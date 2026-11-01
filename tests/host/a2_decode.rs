//! An independent Disk II track decoder for the host tests (`a2_check`, `a2r_check`): 6-and-2
//! data fields after their address fields, framed as a Disk II does; and 13-sector (DOS 3.2)
//! tracks

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

/// Frame a loop of cells IWM-style, three times around: (nibble, last cell).
fn frame3(cells: &[bool]) -> Vec<(u8, usize)> {
    let n = cells.len();
    let mut nibbles = vec![];
    let mut sh = 0u8;
    for i in 0..3 * n {
        sh = sh << 1 | cells[i % n] as u8;
        if sh & 0x80 != 0 {
            nibbles.push((sh, i));
            sh = 0;
        }
    }
    nibbles
}

const DISK_BYTES_53: [u8; 32] = [
    0xab, 0xad, 0xae, 0xaf, 0xb5, 0xb6, 0xb7, 0xba, 0xbb, 0xbd, 0xbe, 0xbf, 0xd6, 0xd7, 0xda, 0xdb,
    0xdd, 0xde, 0xdf, 0xea, 0xeb, 0xed, 0xee, 0xef, 0xf5, 0xf6, 0xf7, 0xfa, 0xfb, 0xfd, 0xfe, 0xff,
];

/// 5-and-3: `None` on a bad nibble or checksum.
pub fn decode53(buf: &[u8]) -> Option<[u8; 256]> {
    const CHUNK: usize = 0x33;
    const THREE: usize = CHUNK * 3 + 1;
    let inv = |b: u8| DISK_BYTES_53.iter().position(|&x| x == b).map(|v| v as u8);
    let mut base = [0u8; 256];
    let mut threes = [0u8; THREE];
    let mut chksum = 0u8;
    let mut idx = 0;
    for i in (0..THREE).rev() {
        chksum ^= inv(buf[idx])?;
        idx += 1;
        threes[i] = chksum;
    }
    for b in base.iter_mut() {
        chksum ^= inv(buf[idx])?;
        idx += 1;
        *b = chksum << 3;
    }
    chksum ^= inv(buf[idx])?;
    if chksum != 0 {
        return None;
    }
    let mut out = [0u8; 256];
    let mut p = 0;
    for i in (0..CHUNK).rev() {
        let (t1, t2, t3) = (threes[i], threes[CHUNK + i], threes[CHUNK * 2 + i]);
        let t4 = (t1 & 0x02) << 1 | (t2 & 0x02) | (t3 & 0x02) >> 1;
        let t5 = (t1 & 0x01) << 2 | (t2 & 0x01) << 1 | (t3 & 0x01);
        out[p] = base[i] | ((t1 >> 2) & 0x07);
        out[p + 1] = base[CHUNK + i] | ((t2 >> 2) & 0x07);
        out[p + 2] = base[CHUNK * 2 + i] | ((t3 >> 2) & 0x07);
        out[p + 3] = base[CHUNK * 3 + i] | (t4 & 0x07);
        out[p + 4] = base[CHUNK * 4 + i] | (t5 & 0x07);
        p += 5;
    }
    out[255] = base[255] | (threes[THREE - 1] & 0x07);
    Some(out)
}

/// [`decode_track`] for a 13-sector track: (track, sector, volume, data) per sector whose
/// `D5 AA B5` address field and 5-and-3 data field are both intact, in the order they start. The
/// marks are taken from the second time around, so a track that starts mid-nibble (the framer
/// out of step at cell 0) still has every sector found once.
pub fn decode_track13(cells: &[bool]) -> Vec<(u8, u8, u8, [u8; 256])> {
    let n = cells.len();
    let nibbles = frame3(cells);
    let d44 = |a: u8, b: u8| (a << 1 | 1) & b;
    let mut out = vec![];
    for i in 0..nibbles.len().saturating_sub(12) {
        if (nibbles[i].0, nibbles[i + 1].0, nibbles[i + 2].0) != (0xD5, 0xAA, 0xB5) || !(n + 7..2 * n + 7).contains(&nibbles[i].1) {
            continue;
        }
        let a = |k: usize| d44(nibbles[i + 3 + 2 * k].0, nibbles[i + 4 + 2 * k].0);
        let (vol, trk, sec, chk) = (a(0), a(1), a(2), a(3));
        if vol ^ trk ^ sec != chk {
            continue;
        }
        for j in i + 11..(i + 51).min(nibbles.len().saturating_sub(414)) {
            if (nibbles[j].0, nibbles[j + 1].0, nibbles[j + 2].0) == (0xD5, 0xAA, 0xAD) {
                let body: Vec<u8> = nibbles[j + 3..j + 3 + 411].iter().map(|x| x.0).collect();
                if let Some(data) = decode53(&body) {
                    out.push((trk, sec, vol, data));
                }
                break;
            }
        }
    }
    out
}
