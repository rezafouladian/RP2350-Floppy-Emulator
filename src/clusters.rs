//! Where a file's clusters are on the card, from the file allocation table itself, so a hard
//! disk image (up to gigabytes) can be mapped without reading it through: `image::load` maps
//! floppy images by reading them once, which at SD speeds would take minutes here.
//!
//! FAT16/FAT32 layouts come from the boot sector of the card's first partition (the one
//! `embedded-sdmmc` opens); exFAT's from `exfat-embedded`'s geometry. A file is given by its
//! first cluster; exFAT can also mark it contiguous (no FAT chain).
//!
//! Hardware-free, so the host tests compile it directly.

pub const BLOCK: usize = 512;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Fat {
    Fat16,
    Fat32,
    Exfat,
}

/// Where a volume's FAT and clusters are, in card sectors.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Layout {
    /// First sector of the (first) FAT.
    pub fat: u32,
    /// Sector of cluster 2, the first one.
    pub heap: u32,
    pub per_cluster: u32,
    /// Clusters on the volume (2 to `clusters + 1`).
    pub clusters: u32,
    pub kind: Fat,
}

fn u16_at(b: &[u8], at: usize) -> u32 {
    u32::from(u16::from_le_bytes([b[at], b[at + 1]]))
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

/// Where a FAT16/FAT32 volume's root directory is.
#[derive(Clone, Copy)]
enum Root {
    /// FAT16: a fixed area of `sectors` sectors from `first`.
    Fixed { first: u32, sectors: u32 },
    /// FAT32: a cluster chain from this cluster.
    Chain(u32),
}

/// The FAT16/FAT32 volume in the first partition of the card's MBR, and its root directory.
fn fat_volume(mut read: impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>) -> Result<(Layout, Root), &'static str> {
    let mut b = [0u8; BLOCK];
    read(0, &mut b)?;
    if b[510..] != [0x55, 0xAA] || b[446 + 4] == 0 {
        return Err("FAT: no partition table");
    }
    let start = u32_at(&b, 446 + 8);
    read(start, &mut b)?;
    if u16_at(&b, 11) != BLOCK as u32 || b[13] == 0 || b[16] == 0 {
        return Err("FAT: unsupported boot sector");
    }
    let per_cluster = u32::from(b[13]);
    let reserved = u16_at(&b, 14);
    let fats = u32::from(b[16]);
    let root_sectors = (u16_at(&b, 17) * 32).div_ceil(BLOCK as u32);
    let total = match u16_at(&b, 19) {
        0 => u32_at(&b, 32),
        n => n,
    };
    let fat_sectors = match u16_at(&b, 22) {
        0 => u32_at(&b, 36),
        n => n,
    };
    let data = reserved + fats * fat_sectors + root_sectors;
    let clusters = total.checked_sub(data).ok_or("FAT: bad boot sector")? / per_cluster;
    let (kind, root) = match clusters {
        0..4_085 => return Err("FAT12 card"),
        4_085..65_525 => (Fat::Fat16, Root::Fixed { first: start + data - root_sectors, sectors: root_sectors }),
        _ => (Fat::Fat32, Root::Chain(u32_at(&b, 44))),
    };
    Ok((Layout { fat: start + reserved, heap: start + data, per_cluster, clusters, kind }, root))
}

impl Layout {
    /// The FAT16/FAT32 volume in the first partition of the card's MBR.
    pub fn fat(read: impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>) -> Result<Self, &'static str> {
        fat_volume(read).map(|(layout, _)| layout)
    }

    /// An exFAT volume, from its geometry (the offsets are from the partition's start).
    pub fn exfat(first_lba: u32, fat_offset: u32, heap_offset: u32, per_cluster: u32, clusters: u32) -> Self {
        Self { fat: first_lba + fat_offset, heap: first_lba + heap_offset, per_cluster, clusters, kind: Fat::Exfat }
    }

    /// First sector of `cluster`.
    pub fn sector(&self, cluster: u32) -> u32 {
        self.heap + (cluster - 2) * self.per_cluster
    }

    /// The cluster that starts at sector `lba`, if one does.
    pub fn cluster_at(&self, lba: u32) -> Option<u32> {
        let off = lba.checked_sub(self.heap)?;
        let cluster = off / self.per_cluster + 2;
        (off % self.per_cluster == 0 && self.valid(cluster)).then_some(cluster)
    }

    fn valid(&self, cluster: u32) -> bool {
        (2..self.clusters + 2).contains(&cluster)
    }
}

/// Add an empty file called `long` to the root directory of the card's FAT16/FAT32 volume, and
/// return its short name (8.3, as stored: `STEM~N  EXT`). `embedded-sdmmc` can only create files
/// with 8.3 names, but it can open this one by its short name and write it.
///
/// The file is a long-name entry and the short entry after it, in the first two free entries in a
/// row. `long` must be ASCII, at most 13 characters (what one long-name entry holds), and not
/// already in the root.
pub fn add_root_file(
    mut read: impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>,
    mut write: impl FnMut(u32, &[u8; BLOCK]) -> Result<(), &'static str>,
    long: &str,
) -> Result<[u8; 11], &'static str> {
    if long.is_empty() || long.len() > 13 || !long.is_ascii() {
        return Err("FAT: bad long name");
    }
    let (layout, root) = fat_volume(&mut read)?;
    // The short name: up to 6 characters of the stem, `~N`, and up to 3 of the extension.
    let keep = |c: &u8| c.is_ascii_alphanumeric() || b"!#$%&'()-@^_`{}".contains(c);
    let (stem, ext) = long.rsplit_once('.').unwrap_or((long, ""));
    let mut short = [b' '; 11];
    let mut tilde = 0;
    for (d, c) in short[..6].iter_mut().zip(stem.bytes().filter(keep)) {
        *d = c.to_ascii_uppercase();
        tilde += 1;
    }
    short[tilde] = b'~';
    for (d, c) in short[8..].iter_mut().zip(ext.bytes().filter(keep)) {
        *d = c.to_ascii_uppercase();
    }
    let is_alias = |e: &[u8]| {
        e[..=tilde] == short[..=tilde]
            && (b'1'..=b'9').contains(&e[tilde + 1])
            && e[tilde + 2..8].iter().all(|&c| c == b' ')
            && e[8..11] == short[8..11]
    };

    // Look through the root: which `~N` are taken, and where two free entries in a row are. After
    // an entry starting with 0 (the end) every entry is free.
    let mut taken = 0u16;
    let mut last_free: Option<(u32, usize)> = None;
    let mut pair = None;
    let mut ended = false;
    let mut b = [0u8; BLOCK];
    let mut cluster = match root {
        Root::Chain(c) => c,
        Root::Fixed { .. } => 0,
    };
    for index in 0u32.. {
        let lba = match root {
            Root::Fixed { first, sectors } if index < sectors => first + index,
            Root::Fixed { .. } => break,
            Root::Chain(_) => {
                if index > 0 && index % layout.per_cluster == 0 {
                    if index / layout.per_cluster > layout.clusters {
                        return Err("FAT: root folder's chain loops");
                    }
                    cluster = next_cluster(&layout, cluster, &mut read, &mut b)?;
                    if cluster >= 0x0FFF_FFF8 {
                        break;
                    }
                }
                if !layout.valid(cluster) {
                    return Err("FAT: bad root folder chain");
                }
                layout.sector(cluster) + index % layout.per_cluster
            }
        };
        read(lba, &mut b)?;
        for (slot, e) in b.chunks_exact(32).enumerate() {
            ended |= e[0] == 0;
            if ended || e[0] == 0xE5 {
                if pair.is_none()
                    && let Some(prev) = last_free
                {
                    pair = Some((prev, (lba, slot)));
                }
                last_free = Some((lba, slot));
            } else {
                last_free = None;
                if e[11] & 0x3F != 0x0F && is_alias(e) {
                    taken |= 1 << (e[tilde + 1] - b'0');
                }
            }
        }
        if ended && pair.is_some() {
            break;
        }
    }
    let Some((at_long, at_short)) = pair else { return Err("FAT: root folder is full") };
    let n = (1..=9).find(|n| taken & 1 << n == 0).ok_or("FAT: no short name left")?;
    short[tilde + 1] = b'0' + n as u8;

    // The long-name entry: the last (and only) one, its 13 UCS-2 characters, ended by 0 and
    // padded with FFFF if shorter.
    let mut entry = [0u8; 32];
    entry[0] = 0x41;
    entry[11] = 0x0F;
    entry[13] = short.iter().fold(0u8, |sum, &c| sum.rotate_right(1).wrapping_add(c));
    const UNITS: [usize; 13] = [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30];
    for (i, at) in UNITS.into_iter().enumerate() {
        let unit = match long.as_bytes().get(i) {
            Some(&c) => u16::from(c),
            None if i == long.len() => 0,
            None => 0xFFFF,
        };
        entry[at..at + 2].copy_from_slice(&unit.to_le_bytes());
    }
    let mut put = |(lba, slot): (u32, usize), entry: &[u8; 32]| {
        read(lba, &mut b)?;
        b[slot * 32..][..32].copy_from_slice(entry);
        write(lba, &b)
    };
    put(at_long, &entry)?;
    // The short entry: an empty archive file, dated 1980-01-01 (there's no clock).
    let mut entry = [0u8; 32];
    entry[..11].copy_from_slice(&short);
    entry[11] = 0x20;
    for at in [16, 18, 24] {
        entry[at..at + 2].copy_from_slice(&0x0021u16.to_le_bytes());
    }
    put(at_short, &entry)?;
    Ok(short)
}

/// The cluster after `cluster` in a FAT32 chain (`b` is scratch).
fn next_cluster(
    layout: &Layout,
    cluster: u32,
    read: &mut impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>,
    b: &mut [u8; BLOCK],
) -> Result<u32, &'static str> {
    let at = cluster * 4;
    read(layout.fat + at / BLOCK as u32, b)?;
    Ok(u32_at(b, (at % BLOCK as u32) as usize) & 0x0FFF_FFFF)
}

/// Call `run(sector, count)` for each run of consecutive card sectors holding the first `blocks`
/// blocks of the file that starts at cluster `first`, in file order. `contiguous`: exFAT's
/// NoFatChain, the clusters follow each other without a chain in the FAT.
pub fn walk(
    layout: &Layout,
    first: u32,
    blocks: u32,
    contiguous: bool,
    mut read: impl FnMut(u32, &mut [u8; BLOCK]) -> Result<(), &'static str>,
    mut run: impl FnMut(u32, u32) -> Result<(), &'static str>,
) -> Result<(), &'static str> {
    if blocks == 0 {
        return Ok(());
    }
    let needed = blocks.div_ceil(layout.per_cluster);
    if !layout.valid(first) {
        return Err("file's first cluster is out of range");
    }
    if contiguous {
        if u64::from(first) + u64::from(needed) > u64::from(layout.clusters) + 2 {
            return Err("file runs past the volume");
        }
        return run(layout.sector(first), blocks);
    }
    // One FAT sector at a time; a run of clusters goes out once it ends.
    let (bytes, mask, end) = match layout.kind {
        Fat::Fat16 => (2, 0xFFFF, 0xFFF8),
        Fat::Fat32 => (4, 0x0FFF_FFFF, 0x0FFF_FFF8),
        Fat::Exfat => (4, 0xFFFF_FFFF, 0xFFFF_FFF8),
    };
    let mut fat = [0u8; BLOCK];
    let mut fat_sector = u32::MAX;
    let mut next_of = |cluster: u32| -> Result<u32, &'static str> {
        let at = cluster * bytes;
        let sector = layout.fat + at / BLOCK as u32;
        if sector != fat_sector {
            read(sector, &mut fat)?;
            fat_sector = sector;
        }
        let at = (at % BLOCK as u32) as usize;
        Ok(if bytes == 2 { u16_at(&fat, at) } else { u32_at(&fat, at) } & mask)
    };
    let mut run_start = first;
    let mut run_len = 1u32;
    let mut cluster = first;
    let mut done = 0u32;
    loop {
        let next = next_of(cluster)?;
        if done + run_len * layout.per_cluster >= blocks {
            // The file's last cluster (maybe only partly used) must end the chain: anything else
            // is a size that doesn't match the chain, or a chain that loops back on itself.
            if next < end {
                return Err("cluster chain is longer than the file");
            }
            return run(layout.sector(run_start), blocks - done);
        }
        if next >= end {
            return Err("file is shorter than its size");
        }
        if !layout.valid(next) {
            return Err("bad cluster chain");
        }
        if next == run_start + run_len {
            run_len += 1;
        } else {
            run(layout.sector(run_start), run_len * layout.per_cluster)?;
            done += run_len * layout.per_cluster;
            run_start = next;
            run_len = 1;
        }
        cluster = next;
    }
}
