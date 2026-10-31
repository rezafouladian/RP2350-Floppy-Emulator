//! ProDOS disk images on the host: list, add and extract files in a volume's root directory, or
//! make a new, empty volume. For getting `tools/a2bench`'s program onto a disk the emulator
//! serves, and its results back off it. Images are ProDOS-order block images (`.po`, `.hdv`,
//! 140K `.dsk` in ProDOS order) or `.2mg` (its blocks after the 64-byte header).
//!
//!   rustc --edition 2024 -O tools/prodos.rs -o target/prodos
//!   target/prodos ls <image> [DIR/PATH]
//!   target/prodos put <image> <file> [NAME] [--type FF --aux 2000]
//!       An AppleSingle file (what cc65 makes) gives its own type and aux type; otherwise BIN
//!       ($06) unless --type says. NAME defaults to the file's name, upper-cased. A file of that
//!       name is replaced.
//!   target/prodos get <image> PATH [out] [--text]   (PATH may go into subdirectories)
//!       --text turns ProDOS text (CR line ends, maybe high bits) into a host text file.
//!   target/prodos new <image> <blocks> <VOLUME> [--boot <image>]
//!       An empty volume. With --boot, that image's boot blocks (0-1) and PRODOS file go on it,
//!       so it starts ProDOS and then its first .SYSTEM file; without, it has no boot code (start
//!       programs on it from another disk).
//!
//! Files are added to the root directory only (read from any); seedling, sapling and tree files,
//! sparse blocks read as zeros. `tools/bench_images.rs` uses it as a module.

use std::fs;

const BLOCK: usize = 512;
const ENTRY: usize = 0x27;
const PER_BLOCK: usize = 13;

pub struct Image {
    pub bytes: Vec<u8>,
    /// Where block 0 starts (64 for a `.2mg`).
    pub offset: usize,
}

pub struct Entry {
    /// Directory block and byte offset of the entry.
    pub at: (usize, usize),
    pub storage: u8,
    pub name: String,
    pub file_type: u8,
    pub key: usize,
    pub blocks: usize,
    pub eof: usize,
    pub aux: u16,
}

impl Image {
    pub fn open(path: &str) -> Image {
        Image::from_bytes(fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}")), path)
    }

    pub fn from_bytes(bytes: Vec<u8>, path: &str) -> Image {
        let offset = if bytes.starts_with(b"2IMG") { u32::from_le_bytes(bytes[24..28].try_into().unwrap()) as usize } else { 0 };
        let img = Image { bytes, offset };
        let vh = img.block(2);
        assert!(vh[4] >> 4 == 0xF, "{path}: no ProDOS volume directory at block 2");
        img
    }

    fn block(&self, b: usize) -> &[u8] {
        &self.bytes[self.offset + b * BLOCK..][..BLOCK]
    }

    fn block_mut(&mut self, b: usize) -> &mut [u8] {
        &mut self.bytes[self.offset + b * BLOCK..][..BLOCK]
    }

    fn total_blocks(&self) -> usize {
        u16::from_le_bytes(self.block(2)[0x29..0x2B].try_into().unwrap()) as usize
    }

    fn bitmap_block(&self) -> usize {
        u16::from_le_bytes(self.block(2)[0x27..0x29].try_into().unwrap()) as usize
    }

    fn volume(&self) -> String {
        let h = &self.block(2)[4..];
        String::from_utf8_lossy(&h[1..1 + (h[0] & 0xF) as usize]).into_owned()
    }

    /// The root directory's blocks, in order.
    fn dir_blocks(&self) -> Vec<usize> {
        self.dir_blocks_from(2)
    }

    /// A directory's blocks from its key block.
    fn dir_blocks_from(&self, key: usize) -> Vec<usize> {
        let mut out = vec![];
        let mut b = key;
        while b != 0 && out.len() < 64 {
            out.push(b);
            b = u16::from_le_bytes(self.block(b)[2..4].try_into().unwrap()) as usize;
        }
        out
    }

    fn entries(&self) -> Vec<Entry> {
        self.entries_in(2)
    }

    /// The entry at `path` (names split by `/`, from the root), searching subdirectories.
    pub fn find(&self, path: &str) -> Option<Entry> {
        let mut dir = 2;
        let parts: Vec<&str> = path.split('/').filter(|p| !p.is_empty()).collect();
        for (k, part) in parts.iter().enumerate() {
            let e = self.entries_in(dir).into_iter().find(|e| e.name.eq_ignore_ascii_case(part))?;
            if k + 1 == parts.len() {
                return Some(e);
            }
            if e.storage != 0xD {
                return None;
            }
            dir = e.key;
        }
        None
    }

    /// The entries of the directory whose key block is `key` (the root's is 2).
    fn entries_in(&self, key: usize) -> Vec<Entry> {
        let mut out = vec![];
        for (k, &b) in self.dir_blocks_from(key).iter().enumerate() {
            for i in 0..PER_BLOCK {
                if k == 0 && i == 0 {
                    continue; // the volume or subdirectory header
                }
                let off = 4 + i * ENTRY;
                let e = &self.block(b)[off..off + ENTRY];
                if e[0] >> 4 == 0 {
                    continue;
                }
                out.push(Entry {
                    at: (b, off),
                    storage: e[0] >> 4,
                    name: String::from_utf8_lossy(&e[1..1 + (e[0] & 0xF) as usize]).into_owned(),
                    file_type: e[0x10],
                    key: u16::from_le_bytes([e[0x11], e[0x12]]) as usize,
                    blocks: u16::from_le_bytes([e[0x13], e[0x14]]) as usize,
                    eof: e[0x15] as usize | (e[0x16] as usize) << 8 | (e[0x17] as usize) << 16,
                    aux: u16::from_le_bytes([e[0x1F], e[0x20]]),
                });
            }
        }
        out
    }

    fn is_free(&self, b: usize) -> bool {
        let bm = self.bitmap_block() + b / 4096;
        self.block(bm)[b % 4096 / 8] & 0x80 >> (b % 8) != 0
    }

    fn set_free(&mut self, b: usize, free: bool) {
        let bm = self.bitmap_block() + b / 4096;
        let byte = &mut self.block_mut(bm)[b % 4096 / 8];
        if free { *byte |= 0x80 >> (b % 8) } else { *byte &= !(0x80 >> (b % 8)) }
    }

    fn alloc(&mut self) -> usize {
        let b = (0..self.total_blocks()).find(|&b| self.is_free(b)).expect("disk full");
        self.set_free(b, false);
        self.block_mut(b).fill(0);
        b
    }

    /// A file's data blocks in order (0 for a sparse one).
    fn data_blocks(&self, e: &Entry) -> Vec<usize> {
        let index = |b: usize| -> Vec<usize> {
            let blk = self.block(b);
            (0..256).map(|i| blk[i] as usize | (blk[256 + i] as usize) << 8).collect()
        };
        let n = e.eof.div_ceil(BLOCK).max(1);
        let all = match e.storage {
            1 => vec![e.key],
            2 => index(e.key),
            3 => index(e.key).into_iter().flat_map(|m| if m == 0 { vec![0; 256] } else { index(m) }).collect(),
            s => panic!("{}: storage type {s} isn't a file", e.name),
        };
        all.into_iter().take(n).collect()
    }

    pub fn read(&self, e: &Entry) -> Vec<u8> {
        let mut out = vec![];
        for b in self.data_blocks(e) {
            if b == 0 { out.extend_from_slice(&[0; BLOCK]) } else { out.extend_from_slice(self.block(b)) }
        }
        out.truncate(e.eof);
        out
    }

    /// Free a file's blocks (data and index) and its entry.
    fn delete(&mut self, e: &Entry) {
        let mut blocks = self.data_blocks(e);
        if e.storage >= 2 {
            blocks.push(e.key);
        }
        if e.storage == 3 {
            let m = self.block(e.key).to_vec();
            blocks.extend((0..256).map(|i| m[i] as usize | (m[256 + i] as usize) << 8));
        }
        for b in blocks.into_iter().filter(|&b| b != 0) {
            self.set_free(b, true);
        }
        let (b, off) = e.at;
        self.block_mut(b)[off..off + ENTRY].fill(0);
        let count = &mut self.block_mut(2)[0x25..0x27];
        let n = u16::from_le_bytes([count[0], count[1]]) - 1;
        count.copy_from_slice(&n.to_le_bytes());
    }

    pub fn put(&mut self, name: &str, file_type: u8, aux: u16, data: &[u8]) {
        assert!(!name.is_empty() && name.len() <= 15 && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '.'), "bad ProDOS name {name:?}");
        assert!(name.as_bytes()[0].is_ascii_alphabetic(), "a ProDOS name starts with a letter");
        assert!(data.len() <= 256 * 256 * BLOCK, "too big");
        if let Some(old) = self.entries().into_iter().find(|e| e.name.eq_ignore_ascii_case(name)) {
            self.delete(&old);
        }
        // A free entry first, so a full directory fails before anything is allocated.
        let slot = self
            .dir_blocks()
            .iter()
            .enumerate()
            .flat_map(|(k, &b)| (0..PER_BLOCK).filter(move |&i| !(k == 0 && i == 0)).map(move |i| (b, 4 + i * ENTRY)))
            .find(|&(b, off)| self.block(b)[off] >> 4 == 0)
            .expect("the root directory is full");
        let n = data.len().div_ceil(BLOCK).max(1);
        let mut blocks = vec![];
        for k in 0..n {
            let b = self.alloc();
            let chunk = &data[(k * BLOCK).min(data.len())..((k + 1) * BLOCK).min(data.len())];
            self.block_mut(b)[..chunk.len()].copy_from_slice(chunk);
            blocks.push(b);
        }
        let index = |img: &mut Image, list: &[usize]| {
            let b = img.alloc();
            for (i, &d) in list.iter().enumerate() {
                img.block_mut(b)[i] = d as u8;
                img.block_mut(b)[256 + i] = (d >> 8) as u8;
            }
            b
        };
        let (storage, key, used) = if n == 1 {
            (1u8, blocks[0], 1)
        } else if n <= 256 {
            (2, index(self, &blocks), n + 1)
        } else {
            let subs: Vec<usize> = blocks.chunks(256).map(|c| index(self, c)).collect();
            (3, index(self, &subs), n + subs.len() + 1)
        };
        let (b, off) = slot;
        let e = &mut self.block_mut(b)[off..off + ENTRY];
        e.fill(0);
        e[0] = storage << 4 | name.len() as u8;
        e[1..1 + name.len()].copy_from_slice(name.to_ascii_uppercase().as_bytes());
        e[0x10] = file_type;
        e[0x11..0x13].copy_from_slice(&(key as u16).to_le_bytes());
        e[0x13..0x15].copy_from_slice(&(used as u16).to_le_bytes());
        e[0x15..0x18].copy_from_slice(&(data.len() as u32).to_le_bytes()[..3]);
        e[0x1E] = 0xE3; // access: read, write, rename, destroy
        e[0x1F..0x21].copy_from_slice(&aux.to_le_bytes());
        e[0x25..0x27].copy_from_slice(&2u16.to_le_bytes()); // header pointer: the key block
        let count = &mut self.block_mut(2)[0x25..0x27];
        let n = u16::from_le_bytes([count[0], count[1]]) + 1;
        count.copy_from_slice(&n.to_le_bytes());
    }
}

/// A new volume: blocks 0-1 empty (no boot code), a 4-block root directory, the bitmap.
pub fn new_volume(blocks: usize, name: &str) -> Vec<u8> {
    assert!((16..=65535).contains(&blocks));
    let mut v = vec![0u8; blocks * BLOCK];
    for (k, b) in (2..6).enumerate() {
        let d = &mut v[b * BLOCK..(b + 1) * BLOCK];
        d[..2].copy_from_slice(&(if k == 0 { 0u16 } else { b as u16 - 1 }).to_le_bytes());
        d[2..4].copy_from_slice(&(if b == 5 { 0u16 } else { b as u16 + 1 }).to_le_bytes());
    }
    let h = &mut v[2 * BLOCK + 4..2 * BLOCK + 4 + ENTRY];
    h[0] = 0xF0 | name.len() as u8;
    h[1..1 + name.len()].copy_from_slice(name.to_ascii_uppercase().as_bytes());
    h[0x1E] = 0xC3;
    h[0x1F] = ENTRY as u8;
    h[0x20] = PER_BLOCK as u8;
    h[0x23..0x25].copy_from_slice(&6u16.to_le_bytes());
    h[0x25..0x27].copy_from_slice(&(blocks as u16).to_le_bytes());
    let bitmap_blocks = blocks.div_ceil(4096);
    for b in 6 + bitmap_blocks..blocks {
        v[6 * BLOCK + b / 8] |= 0x80 >> (b % 8);
    }
    v
}

/// The data fork, type and aux type of an AppleSingle file, if it is one.
pub fn apple_single(f: &[u8]) -> Option<(Vec<u8>, u8, u16)> {
    if f.len() < 26 || f[..4] != [0, 5, 0x16, 0] {
        return None;
    }
    let be32 = |at: usize| u32::from_be_bytes(f[at..at + 4].try_into().unwrap()) as usize;
    let count = u16::from_be_bytes([f[24], f[25]]) as usize;
    let (mut data, mut ty, mut aux) = (None, 0x06u8, 0u16);
    for k in 0..count {
        let e = 26 + 12 * k;
        let (id, off, len) = (be32(e), be32(e + 4), be32(e + 8));
        match id {
            1 => data = Some(f[off..off + len].to_vec()),
            11 => {
                ty = f[off + 3];
                aux = u16::from_be_bytes([f[off + 6], f[off + 7]]);
            }
            _ => {}
        }
    }
    data.map(|d| (d, ty, aux))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let flag = |name: &str| args.iter().position(|a| a == name).map(|i| args[i + 1].clone());
    let plain: Vec<&String> = {
        let mut v = vec![];
        let mut i = 0;
        while i < args.len() {
            if args[i] == "--type" || args[i] == "--aux" || args[i] == "--boot" {
                i += 2;
                continue;
            }
            if !args[i].starts_with("--") {
                v.push(&args[i]);
            }
            i += 1;
        }
        v
    };
    let usage = || -> ! {
        eprintln!("usage: prodos ls|put|get|new ... (see the header of tools/prodos.rs)");
        std::process::exit(2)
    };
    match plain.first().map(|s| s.as_str()) {
        Some("ls") if plain.len() >= 2 => {
            let img = Image::open(plain[1]);
            println!("/{}: {} blocks", img.volume(), img.total_blocks());
            let dir = match plain.get(2) {
                Some(p) => img.find(p).filter(|e| e.storage == 0xD).unwrap_or_else(|| panic!("no directory {p}")).key,
                None => 2,
            };
            for e in img.entries_in(dir) {
                println!("  {:15} ${:02X} aux ${:04X} {:7} bytes {:5} blocks", e.name, e.file_type, e.aux, e.eof, e.blocks);
            }
            let free = (0..img.total_blocks()).filter(|&b| img.is_free(b)).count();
            println!("{free} blocks free");
        }
        Some("put") if plain.len() >= 3 => {
            let mut img = Image::open(plain[1]);
            let raw = fs::read(plain[2]).unwrap_or_else(|e| panic!("{}: {e}", plain[2]));
            let (data, mut ty, mut aux) = apple_single(&raw).unwrap_or((raw, 0x06, 0));
            if let Some(t) = flag("--type") {
                ty = u8::from_str_radix(t.trim_start_matches('$'), 16).expect("--type hex");
            }
            if let Some(a) = flag("--aux") {
                aux = u16::from_str_radix(a.trim_start_matches('$'), 16).expect("--aux hex");
            }
            let name = plain.get(3).map(|s| s.to_string()).unwrap_or_else(|| {
                std::path::Path::new(plain[2]).file_name().unwrap().to_string_lossy().to_ascii_uppercase()
            });
            img.put(&name, ty, aux, &data);
            fs::write(plain[1], &img.bytes).unwrap();
            println!("{name}: {} bytes, type ${ty:02X}, aux ${aux:04X}, on /{}", data.len(), img.volume());
        }
        Some("get") if plain.len() >= 3 => {
            let img = Image::open(plain[1]);
            let e = img.find(plain[2]).unwrap_or_else(|| panic!("no {} on /{}", plain[2], img.volume()));
            let mut data = img.read(&e);
            if args.iter().any(|a| a == "--text") {
                data = data.iter().map(|&b| b & 0x7F).map(|b| if b == b'\r' { b'\n' } else { b }).collect();
            }
            match plain.get(3) {
                Some(out) => fs::write(out, &data).unwrap(),
                None => {
                    use std::io::Write;
                    std::io::stdout().write_all(&data).unwrap();
                }
            }
        }
        Some("new") if plain.len() >= 4 => {
            let blocks: usize = plain[2].parse().expect("blocks");
            let mut img = Image { bytes: new_volume(blocks, plain[3]), offset: 0 };
            if let Some(from) = flag("--boot") {
                let src = Image::open(&from);
                let prodos = src.entries().into_iter().find(|e| e.name == "PRODOS").unwrap_or_else(|| panic!("no PRODOS on {from}"));
                img.bytes[..2 * BLOCK].copy_from_slice(&src.bytes[src.offset..src.offset + 2 * BLOCK]);
                img.put("PRODOS", prodos.file_type, prodos.aux, &src.read(&prodos));
            }
            fs::write(plain[1], &img.bytes).unwrap();
            println!("/{}: {} blocks{}", img.volume(), img.total_blocks(), if flag("--boot").is_some() { ", boot blocks and PRODOS" } else { ", empty" });
        }
        _ => usage(),
    }
}
