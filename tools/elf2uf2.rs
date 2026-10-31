//! Host tool: the firmware's ELF to a UF2 for the RP2350's boot ROM (drag and drop onto the
//! RP2350 drive), as `picotool uf2 convert -t elf` makes it: the flash segments' bytes at their
//! load addresses, 256 bytes a block, whole 256-byte pages (gaps 0), family RP2350 Arm Secure.
//! Used by `.forgejo/workflows/build.yml`, so CI needn't build picotool and the Pico SDK.
//!
//! `rustc --edition 2024 -O tools/elf2uf2.rs -o target/elf2uf2 && target/elf2uf2 <elf> <uf2>`

use std::collections::BTreeMap;

/// UF2 block magic numbers and the flag saying the family ID field is set.
const MAGIC_START0: u32 = 0x0A32_4655;
const MAGIC_START1: u32 = 0x9E5D_5157;
const MAGIC_END: u32 = 0x0AB1_6F30;
const FLAG_FAMILY: u32 = 0x0000_2000;
/// RP2350, Arm, Secure (what the RP2350 boot ROM takes for an Arm image).
const FAMILY_RP2350_ARM_S: u32 = 0xE48B_FF59;
const PAGE: u32 = 256;
const FLASH: std::ops::Range<u32> = 0x1000_0000..0x1200_0000;

fn u16_at(d: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([d[at], d[at + 1]])
}

fn u32_at(d: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(d[at..at + 4].try_into().unwrap())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let [_, elf, out] = args.as_slice() else {
        eprintln!("usage: elf2uf2 <elf> <uf2>");
        std::process::exit(2);
    };
    let d = std::fs::read(elf).unwrap_or_else(|e| panic!("{elf}: {e}"));
    assert!(d.len() > 52 && &d[..4] == b"\x7fELF", "{elf}: not an ELF file");
    assert!(d[4] == 1 && d[5] == 1, "{elf}: not a 32-bit little-endian ELF");
    let (phoff, phentsize, phnum) = (u32_at(&d, 28) as usize, u16_at(&d, 42) as usize, u16_at(&d, 44) as usize);

    // The bytes to program, by page: every loadable segment with file contents, at its load
    // (physical) address. Segments only in RAM (.bss, the stack) have none.
    let mut pages: BTreeMap<u32, [u8; PAGE as usize]> = BTreeMap::new();
    for k in 0..phnum {
        let h = phoff + k * phentsize;
        let (kind, offset, paddr, filesz) = (u32_at(&d, h), u32_at(&d, h + 4) as usize, u32_at(&d, h + 12), u32_at(&d, h + 16));
        if kind != 1 || filesz == 0 {
            continue;
        }
        assert!(FLASH.contains(&paddr) && FLASH.contains(&(paddr + filesz - 1)), "segment at {paddr:#x} isn't in flash");
        for (i, &b) in d[offset..offset + filesz as usize].iter().enumerate() {
            let at = paddr + i as u32;
            pages.entry(at & !(PAGE - 1)).or_insert([0; PAGE as usize])[(at % PAGE) as usize] = b;
        }
    }
    assert!(!pages.is_empty(), "{elf}: nothing to program");

    let count = pages.len() as u32;
    let mut uf2 = Vec::with_capacity(pages.len() * 512);
    for (n, (addr, data)) in pages.iter().enumerate() {
        for word in [MAGIC_START0, MAGIC_START1, FLAG_FAMILY, *addr, PAGE, n as u32, count, FAMILY_RP2350_ARM_S] {
            uf2.extend_from_slice(&word.to_le_bytes());
        }
        uf2.extend_from_slice(data);
        uf2.resize(uf2.len() + 476 - PAGE as usize, 0);
        uf2.extend_from_slice(&MAGIC_END.to_le_bytes());
    }
    std::fs::write(out, &uf2).unwrap_or_else(|e| panic!("{out}: {e}"));
    let (first, last) = (pages.keys().next().unwrap(), pages.keys().last().unwrap());
    println!("{out}: {count} blocks, {first:#010x}..{:#010x}", last + PAGE);
}
