//! Host-side check of the settings file parser in `src/config.rs`.
//!
//! Run from the repo root:
//!   rustc --edition 2024 -O -A dead_code tests/host/config_check.rs -o target/config_check && target/config_check

#[path = "../../src/config.rs"]
mod config;

use config::{Autoload, Config, Mode, parse};

fn check(text: &str, autoload: Autoload, bad_line: Option<usize>) {
    let got = parse(text.as_bytes());
    let want = Config { autoload, mode: Mode::Floppy, keep_rotation: true, remember: false, last: [None; 4], bad_line };
    assert_eq!(got, want, "for {:?}", text);
}

fn main() {
    // No settings: the defaults.
    check("", Autoload::Off, None);
    check("\n\n# just a comment\n; another\n", Autoload::Off, None);

    // The values, any case, any spacing, any line ending.
    for v in ["first", "FIRST", "on", "Yes", "true"] {
        check(&format!("autoload = {}\n", v), Autoload::First, None);
    }
    for v in ["off", "No", "false", "none", ""] {
        check(&format!("autoload={}", v), Autoload::Off, None);
    }
    check("  AutoLoad\t=  first  \r\n", Autoload::First, None);
    check("autoload = MacTerminal.dsk\r\n", Autoload::Named("MacTerminal.dsk"), None);
    // Names keep their spaces and `=`s; only the ends are trimmed.
    check("autoload = My Disk = 2.img ", Autoload::Named("My Disk = 2.img"), None);

    // A byte order mark (TextEdit, Notepad).
    check("\u{FEFF}autoload = first", Autoload::First, None);

    // The last setting wins.
    check("autoload = first\nautoload = off\n", Autoload::Off, None);

    // Lines not understood are reported (the first one), and the rest still apply.
    check("autoload = first\nspeed = 11\nnonsense\n", Autoload::First, Some(2));
    check("hello\nautoload = on", Autoload::First, Some(1));
    let mut bytes = b"autoload = on\n".to_vec();
    bytes.extend_from_slice(b"x = \xFF\xFE\n");
    assert_eq!(parse(&bytes), Config { autoload: Autoload::First, mode: Mode::Floppy, keep_rotation: true, remember: false, last: [None; 4], bad_line: Some(2) });

    // The mode.
    for (v, mode) in [("hd20", Mode::Hd20), ("HD", Mode::Hd20), ("dcd", Mode::Hd20), ("hd20+floppy", Mode::Hd20Floppy), ("HD20 Floppy", Mode::Hd20Floppy), ("chain", Mode::Hd20Floppy), ("Floppy", Mode::Floppy), ("sony", Mode::Floppy), ("apple2", Mode::Apple2), ("Apple II", Mode::Apple2), ("5.25", Mode::Apple2), ("apple35", Mode::Apple35), ("Apple 3.5", Mode::Apple35), ("IIgs", Mode::Apple35), ("3.5", Mode::Apple35), ("apple2dual", Mode::Dual), ("Dual", Mode::Dual), ("5.25+3.5", Mode::Dual), ("smartport", Mode::SmartPort), ("SP", Mode::SmartPort), ("UniDisk", Mode::SmartPort)] {
        let text = format!("autoload = Disk.hda\r\nmode = {}\r\n", v);
        let c = parse(text.as_bytes());
        assert_eq!(c, Config { autoload: Autoload::Named("Disk.hda"), mode, keep_rotation: true, remember: false, last: [None; 4], bad_line: None }, "mode = {}", v);
    }
    assert_eq!(parse(b"mode = hd20\nmode = floppy").mode, Mode::Floppy);
    // Keeping the rotation: on unless the file says off.
    for (v, keep) in [("on", true), ("Yes", true), ("true", true), ("keep", true), ("OFF", false), ("no", false), ("false", false)] {
        let text = format!("keep_rotation = {}\r\nautoload = first", v);
        let c = parse(text.as_bytes());
        assert_eq!(c, Config { autoload: Autoload::First, mode: Mode::Floppy, keep_rotation: keep, remember: false, last: [None; 4], bad_line: None }, "keep_rotation = {}", v);
    }
    assert!(parse(b"autoload = first").keep_rotation);
    let c = parse(b"keep_rotation = sometimes\nKeep_Rotation=off");
    assert_eq!((c.keep_rotation, c.bad_line), (false, Some(1)));

    // Remembering: off unless the file says on; the images it keeps, by drive or unit.
    assert!(!parse(b"autoload = first").remember);
    for (v, on) in [("on", true), ("YES", true), ("true", true), ("off", false), ("no", false), ("False", false)] {
        assert_eq!(parse(format!("remember = {}", v).as_bytes()).remember, on, "remember = {}", v);
    }
    let c = parse(b"remember = maybe\nremember = on");
    assert_eq!((c.remember, c.bad_line), (true, Some(1)));
    let c = parse(b"remember = on\nlast = /Games/My Disk = 2.dsk \nLAST3 =\nlast2 = /a.woz\nlast2 = /b.woz\nlast5 = /c.dsk\n");
    assert_eq!(c.last, [Some("/Games/My Disk = 2.dsk"), Some("/b.woz"), Some(""), None]);
    assert_eq!(c.bad_line, Some(6));
    // A mode it doesn't know is reported and leaves the default.
    assert_eq!(parse(b"mode = scsi\nautoload = first"), Config { autoload: Autoload::First, mode: Mode::Floppy, keep_rotation: true, remember: false, last: [None; 4], bad_line: Some(1) });

    // The file written when there is none: every setting at its default, nothing misunderstood,
    // and lines a text editor shows whole.
    assert_eq!(parse(config::DEFAULT_FILE.as_bytes()), Config::default());
    assert!(config::DEFAULT_FILE.lines().all(|l| l.len() <= 76), "a long line in DEFAULT_FILE");
    for (mode, want) in [(Mode::Floppy, "floppy"), (Mode::Hd20, "hd20"), (Mode::Hd20Floppy, "hd20+floppy"), (Mode::Apple2, "apple2"), (Mode::Apple35, "apple35"), (Mode::Dual, "apple2dual"), (Mode::SmartPort, "smartport")] {
        assert_eq!(mode.value(), want);
        assert_eq!(parse(format!("mode = {}", want).as_bytes()).mode, mode);
    }

    // Changing a value keeps the rest of the file.
    let mut t = config::Text::new();
    t.defaults();
    assert!(t.set(config::MODE, "hd20"));
    assert!(t.set(config::AUTOLOAD, "My Disk = 2.hda"));
    assert!(t.set(config::KEEP_ROTATION, "off"));
    assert!(t.set(config::REMEMBER, "on"));
    assert!(t.set(config::LAST[0], "/Games/Disk.dsk"));
    assert!(t.set(config::LAST[1], ""));
    assert_eq!(t.config(), Config { autoload: Autoload::Named("My Disk = 2.hda"), mode: Mode::Hd20, keep_rotation: false, remember: true, last: [Some("/Games/Disk.dsk"), Some(""), None, None], bad_line: None });
    let want = config::DEFAULT_FILE
        .replace("\nmode = floppy\n", "\nmode = hd20\n")
        .replace("\nautoload = off\n", "\nautoload = My Disk = 2.hda\n")
        .replace("\nkeep_rotation = on\n", "\nkeep_rotation = off\n")
        .replace("\nremember = off\n", "\nremember = on\nlast = /Games/Disk.dsk\nlast2 = \n");
    assert_eq!(text(&t), want);
    // And back, through every value.
    for v in [Autoload::Off, Autoload::First, Autoload::Named("Disk.dsk")] {
        assert!(t.set(config::AUTOLOAD, v.value()));
        assert_eq!(t.config().autoload, v);
    }
    assert!(t.set(config::AUTOLOAD, "off"));
    assert!(t.set(config::MODE, "floppy"));
    assert!(t.set(config::KEEP_ROTATION, "on"));
    assert!(t.set(config::REMEMBER, "off"));
    // The remembered images' lines stay (unused while it's off).
    assert_eq!(text(&t), format!("{}last = /Games/Disk.dsk\nlast2 = \n", config::DEFAULT_FILE));

    // The line that counts is the last one for the key (also one that wasn't understood); a key
    // only in a comment, or not there at all, gets a line at the end.
    edit("mode = hd20\nmode = scsi\nautoload = first\n", "mode", "floppy", "mode = hd20\nmode = floppy\nautoload = first\n");
    edit("  Mode\t=  hd20  \n", "mode", "apple2", "mode = apple2\n");
    edit("# mode = hd20\n", "mode", "hd20", "# mode = hd20\nmode = hd20\n");
    edit("autoload = first", "mode", "hd20", "autoload = first\nmode = hd20\n");
    edit("", "mode", "hd20", "mode = hd20\n");
    // Line endings and a byte order mark stay as they were.
    edit("autoload = first\r\nmode = floppy\r\n", "mode", "hd20", "autoload = first\r\nmode = hd20\r\n");
    edit("autoload = first\r\n", "mode", "hd20", "autoload = first\r\nmode = hd20\r\n");
    edit("\u{FEFF}mode = floppy\n", "mode", "apple2", "\u{FEFF}mode = apple2\n");
    edit("\u{FEFF}", "mode", "apple2", "\u{FEFF}mode = apple2\n");

    // Nothing changes if the result wouldn't fit.
    let mut t = config::Text::new();
    let long = format!("# {}\nmode = hd20\n", "x".repeat(config::MAX_BYTES - 20));
    load(&mut t, &long);
    assert!(t.set("mode", "apple2"));
    assert!(!t.set("mode", "floppy and then some more"));
    assert!(!t.set("autoload", "first"));
    assert_eq!(t.config().mode, Mode::Apple2);
    // A file longer than what's read is marked partial.
    t.read(config::MAX_BYTES, config::MAX_BYTES as u64 + 1);
    assert!(t.partial);

    println!("config_check: all passed");
}

fn text(t: &config::Text) -> &str {
    std::str::from_utf8(t.as_bytes()).unwrap()
}

fn load(t: &mut config::Text, s: &str) {
    t.buf_mut()[..s.len()].copy_from_slice(s.as_bytes());
    t.read(s.len(), s.len() as u64);
    assert!(!t.partial);
}

fn edit(before: &str, key: &str, value: &str, after: &str) {
    let mut t = config::Text::new();
    load(&mut t, before);
    assert!(t.set(key, value));
    assert_eq!(text(&t), after, "setting {} = {} in {:?}", key, value, before);
}
