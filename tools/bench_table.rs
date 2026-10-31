//! Makes a comparison table of benchmark runs: the "FloppyBench Summary" file FloppyBench writes
//! on the Mac, and the `BENCH.SUMMARY` file of the Apple II benchmark (`tools/a2bench`). Each line
//! is one run: its label, then tab-separated `key=value` pairs (CR or LF line ends). Rows are
//! runs, columns the keys in the order first seen (or `--keys`), as a Markdown table, or
//! tab-separated with `--tsv` for a spreadsheet.
//!
//!   rustc --edition 2024 -O tools/bench_table.rs -o target/bench_table
//!   target/bench_table "FloppyBench Summary" [more files] [--keys seq8k,block_ms,rand_ms] [--tsv]
//!     [--match TEXT]
//!
//! `--match` keeps the runs whose label contains TEXT (ignoring case). The identity columns
//! (date, machine, system) are left out unless named in `--keys`.

fn main() {
    let mut files = vec![];
    let (mut keys, mut tsv, mut filter): (Option<Vec<String>>, bool, Option<String>) = (None, false, None);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--keys" => keys = Some(args.next().expect("--keys a,b,c").split(',').map(str::to_string).collect()),
            "--tsv" => tsv = true,
            "--match" => filter = Some(args.next().expect("--match TEXT").to_lowercase()),
            _ => files.push(a),
        }
    }
    if files.is_empty() {
        eprintln!("usage: bench_table <summary file>... [--keys k1,k2] [--tsv] [--match TEXT]");
        std::process::exit(2);
    }
    let mut runs: Vec<(String, Vec<(String, String)>)> = vec![];
    for f in &files {
        let bytes = std::fs::read(f).unwrap_or_else(|e| panic!("{f}: {e}"));
        // Mac Roman and ProDOS text are ASCII for what's written here; anything else shows as '?'.
        let text: String = bytes.iter().map(|&b| if b < 0x80 { b as char } else { '?' }).collect();
        for line in text.split(['\r', '\n']).filter(|l| !l.trim().is_empty()) {
            let mut fields = line.split('\t');
            let label = fields.next().unwrap_or("").trim().to_string();
            let pairs = fields.filter_map(|p| p.split_once('=')).map(|(k, v)| (k.trim().to_string(), v.trim().to_string())).collect();
            if filter.as_ref().is_none_or(|m| label.to_lowercase().contains(m)) {
                runs.push((label, pairs));
            }
        }
    }
    let hidden = ["date", "machine", "system"];
    let columns: Vec<String> = keys.unwrap_or_else(|| {
        let mut c: Vec<String> = vec![];
        for (_, pairs) in &runs {
            for (k, _) in pairs {
                if !hidden.contains(&k.as_str()) && !c.contains(k) {
                    c.push(k.clone());
                }
            }
        }
        c
    });
    let value = |pairs: &[(String, String)], k: &str| pairs.iter().rev().find(|(key, _)| key == k).map_or(String::new(), |(_, v)| v.clone());
    if tsv {
        println!("label\t{}", columns.join("\t"));
        for (label, pairs) in &runs {
            let row: Vec<String> = columns.iter().map(|c| value(pairs, c)).collect();
            println!("{label}\t{}", row.join("\t"));
        }
        return;
    }
    println!("| run | {} |", columns.join(" | "));
    println!("|---|{}", "---:|".repeat(columns.len()));
    for (label, pairs) in &runs {
        let row: Vec<String> = columns.iter().map(|c| value(pairs, c)).collect();
        println!("| {} | {} |", label.replace('|', "/"), row.join(" | "));
    }
}
