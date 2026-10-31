/*
 * A2Bench: times an Apple II block device (a 5.25" disk, a 3.5" disk, a SmartPort unit or hard
 * disk) from the computer's side, through ProDOS 8's block calls, so drives, emulators and image
 * types can be compared. A ProDOS 8 system program (cc65); it runs from memory, so the disk it
 * was loaded from can be swapped for the one to test.
 *
 * Tests, on one device:
 *   1. Sequential reads, one block per call (the whole disk, or its first 1,600 blocks).
 *   2. Random single-block reads (the same sequence every run).
 *   3. Seeks: alternating reads between tracks 1, half the disk and all of it apart.
 *   4. Side switches (800K disks).
 *   5. (optional) Changed-data writes: blocks written with their complement, read back and
 *      compared, then written back as they were and compared again.
 *   6. (optional) File test: a file written, closed, read back and deleted.
 *
 * Timing needs a IIgs: its battery clock, read with ReadTimeHex, keeps counting while the disk
 * firmware has interrupts off (which loses VBL interrupts). Each timed run starts on a clock
 * second; when it ends, video frames are counted (by polling RDVBLBAR) to the next second, so
 * the fraction is known to a frame. Elsewhere the tests run untimed.
 *
 * Results are shown and appended to BENCH.RESULTS (text) and BENCH.SUMMARY (one line of
 * tab-separated key=value pairs per run, for tools/bench_table.rs) in the prefix it was started
 * from. If that disk isn't in, it asks for it.
 */

#ifdef __APPLE2ENH__
#include <apple2enh.h>
#else
#include <apple2.h> /* the 6502 build, for tools/a2emu (40 columns) */
#endif
#include <conio.h>
#include <ctype.h>
#include <device.h>
#include <dio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define VERSION "1.0"
#define BLOCK 512
#define SEQ_MAX 1600u
#define RANDOM_READS 48
#define SLICE 24 /* blocks kept in memory at a time by the write test */
#define SWITCHED 0x2E /* ProDOS: disk switched */

unsigned char iigs(void);
void __fastcall__ rtc_read(unsigned char *hms);
unsigned char vbl(void);
unsigned char fast(void);
void __fastcall__ cout(char c);
void home(void);
void col80(void);

static unsigned char buf[BLOCK], buf2[BLOCK];
static unsigned char orig[SLICE * BLOCK];

/* ---- Timing ---- */

static unsigned char fps; /* frames per second; 0: no clock */
static unsigned long t_start;

static unsigned long secs(void)
{
    unsigned char t[3];
    rtc_read(t);
    return t[2] * 3600ul + t[1] * 60u + t[0];
}

/* One full video frame, whatever RDVBLBAR's polarity. */
static void frame(void)
{
    unsigned char s = vbl();
    while (vbl() == s)
        ;
    while (vbl() != s)
        ;
}

/* Wait for the clock's next second; returns the frames waited. */
static unsigned next_second(unsigned long *at)
{
    unsigned long s = secs(), n;
    unsigned frames = 0;
    do {
        frame();
        frames++;
        n = secs();
    } while (n == s);
    *at = n;
    return frames;
}

static void timer_start(void)
{
    if (fps)
        next_second(&t_start);
}

/* Milliseconds since timer_start, 0 without a clock. */
static unsigned long timer_ms(void)
{
    unsigned long s, n;
    unsigned frames;
    if (!fps)
        return 0;
    s = secs();
    frames = next_second(&n);
    return ((s + 86400ul - t_start) % 86400ul) * 1000ul + 1000ul - frames * 1000ul / fps;
}

static void measure_fps(void)
{
    unsigned long a;
    unsigned f;
    fps = 1;
    next_second(&a);
    f = next_second(&a);
    f += next_second(&a);
    fps = (f + 1) / 2;
}

/* ---- Output: the screen, the results and the summary ---- */

/* Through the monitor's COUT, which scrolls (cc65's conio wraps to the top). */
static void puts_(const char *s)
{
    for (; *s; s++)
        cout(*s == '\n' ? '\r' : *s);
}

static void say(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    puts_(line);
}

static char results[3000];
static char summary[600];

static void out(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    puts_(line);
    if (strlen(results) + strlen(line) < sizeof results)
        strcat(results, line);
}

static void sum(const char *key, const char *fmt, ...)
{
    char v[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(v, sizeof v, fmt, ap);
    va_end(ap);
    if (strlen(summary) + strlen(key) + strlen(v) + 3 < sizeof summary) {
        strcat(summary, "\t");
        strcat(summary, key);
        strcat(summary, "=");
        strcat(summary, v);
    }
}

/* "12.3" from tenths. */
static const char *tenths(unsigned long t)
{
    static char s[4][16];
    static unsigned char k;
    k = (k + 1) & 3;
    sprintf(s[k], "%lu.%lu", t / 10, t % 10);
    return s[k];
}

/* KB/s in tenths for `blocks` blocks in `ms`. */
static unsigned long kbps10(unsigned long blocks, unsigned long ms)
{
    return ms ? blocks * 5000ul / ms : 0;
}

static void line_in(char *s, unsigned char size)
{
    unsigned char n = 0;
    char c;
    for (;;) {
        c = cgetc();
        if (c == '\r') {
            puts_("\n");
            s[n] = 0;
            return;
        }
        if ((c == 8 || c == 0x7F) && n) {
            n--;
            puts_("\x08 \x08");
        } else if (c >= ' ' && c < 0x7F && n < size - 1) {
            s[n++] = c;
            cout(c);
        }
    }
}

static int yes(const char *q)
{
    char c;
    say("%s (y/N) ", q);
    c = cgetc();
    puts_("\n");
    return c == 'y' || c == 'Y';
}

/* Append text to a file in `prefix` (created as TXT if need be). */
static int append(const char *prefix, const char *name, const char *text)
{
    char path[80];
    int fd;
    unsigned n = strlen(text);
    char *p;
    strcpy(path, prefix);
    strcat(path, name);
    _filetype = PRODOS_T_TXT;
    _auxtype = 0;
    fd = open(path, O_WRONLY | O_CREAT);
    if (fd < 0)
        return 0;
    lseek(fd, 0, SEEK_END);
    /* ProDOS text has CR line ends. */
    for (p = (char *)text; *p; p++)
        if (*p == '\n')
            *p = '\r';
    n = write(fd, text, n) == (int)n;
    close(fd);
    return n;
}

static char prefix[64];

/* Save the run's results in `prefix`; if that disk isn't there, wait for it, or take another
   prefix (another volume, say the disk just tested). */
static void save(void)
{
    char c;
    char other[64];
    char *at = prefix;
    strcat(summary, "\n");
    for (;;) {
        if (append(at, "BENCH.RESULTS", results) && append(at, "BENCH.SUMMARY", summary)) {
            say("Saved to %sBENCH.RESULTS and BENCH.SUMMARY.\n\n", at);
            if (at != prefix)
                strcpy(prefix, at); /* and there from now on */
            return;
        }
        say("Can't write to %s. Put that disk back (and writable) and press Return,\n"
            "or P to save elsewhere, or Esc not to save. ", at);
        c = cgetc();
        puts_("\n");
        if (c == 0x1B)
            return;
        if (c == 'p' || c == 'P') {
            puts_("Prefix (e.g. /BLANK800K/): ");
            line_in(other, sizeof other - 1);
            if (other[0] && other[strlen(other) - 1] != '/')
                strcat(other, "/");
            if (other[0])
                at = other;
        }
    }
}

/* ---- Devices ---- */

typedef struct {
    unsigned char dev; /* cc65's device number: the ProDOS unit number >> 4 (DSSS) */
    dhandle_t h;
    unsigned blocks;
    char volume[17];
    const char *kind;
} Disk;

static unsigned char rd(Disk *d, unsigned block, void *p)
{
    unsigned char e = dio_read(d->h, block, p);
    if (e == SWITCHED)
        e = dio_read(d->h, block, p);
    return e;
}

static unsigned char wr(Disk *d, unsigned block, const void *p)
{
    unsigned char e = dio_write(d->h, block, p);
    if (e == SWITCHED)
        e = dio_write(d->h, block, p);
    return e;
}

static const char *kind_of(unsigned blocks)
{
    return blocks == 280 || blocks == 320 ? "5.25\"" : blocks == 800 || blocks == 1600 ? "3.5\"" : "hard disk";
}

/* First block of cylinder `c` on a 3.5" disk (12 sectors a side in zone 0, down to 8). */
static unsigned cyl35(const Disk *d, unsigned char c)
{
    unsigned b = 0;
    unsigned char k, sides = d->blocks == 800 ? 1 : 2;
    for (k = 0; k < c; k++)
        b += (12 - k / 16) * sides;
    return b;
}

/* ---- Tests ---- */

static void test_sequential(Disk *d)
{
    unsigned n = d->blocks < SEQ_MAX ? d->blocks : SEQ_MAX;
    unsigned b;
    unsigned long ms;
    out("1. Sequential reads, one block per call: %u blocks\n", n);
    timer_start();
    for (b = 0; b < n; b++)
        if (rd(d, b, buf)) {
            out("   read error at block %u\n", b);
            return;
        }
    ms = timer_ms();
    if (ms) {
        out("   %s s = %s KB/s, %s ms per block\n", tenths(ms / 100), tenths(kbps10(n, ms)), tenths(ms * 10 / n));
        sum("seq", "%s", tenths(kbps10(n, ms)));
        sum("block_ms", "%s", tenths(ms * 10 / n));
    }
}

static unsigned long rand_state;
static unsigned next_random(unsigned blocks)
{
    rand_state = rand_state * 1103515245ul + 12345;
    return (unsigned)((rand_state >> 8) % blocks);
}

static void test_random(Disk *d)
{
    unsigned char i;
    unsigned long ms;
    rand_state = 0x5EED;
    out("2. Random single-block reads: %u\n", RANDOM_READS);
    rd(d, 0, buf);
    timer_start();
    for (i = 0; i < RANDOM_READS; i++)
        if (rd(d, next_random(d->blocks), buf)) {
            out("   read error\n");
            return;
        }
    ms = timer_ms();
    if (ms) {
        out("   %s ms per read\n", tenths(ms * 10 / RANDOM_READS));
        sum("rand_ms", "%s", tenths(ms * 10 / RANDOM_READS));
    }
}

/* Alternate single-block reads; ms per read in tenths. */
static unsigned long alternate(Disk *d, unsigned a, unsigned b, unsigned char rounds)
{
    unsigned char i;
    rd(d, a, buf);
    timer_start();
    for (i = 0; i < rounds; i++)
        if (rd(d, b, buf) || rd(d, a, buf))
            return 0;
    return timer_ms() * 10 / (2 * rounds);
}

static void seek(Disk *d, const char *what, const char *key, unsigned a, unsigned b)
{
    unsigned long t = alternate(d, a, b, 12);
    out("   %s (blocks %u <-> %u): %s ms\n", what, a, b, tenths(t));
    if (t)
        sum(key, "%s", tenths(t));
}

static void test_seeks(Disk *d)
{
    out("3. Seeks: alternating single-block reads, ms per read including the step\n");
    if (d->blocks == 280 || d->blocks == 320) {
        seek(d, " 1 track ", "seek1", 8 * 20, 8 * 21);
        seek(d, "17 tracks", "seek17", 0, 8 * 17);
        seek(d, "34 tracks", "seek34", 0, 8 * 34);
    } else if (d->blocks == 800 || d->blocks == 1600) {
        seek(d, " 1 track ", "seek1", cyl35(d, 20), cyl35(d, 21));
        seek(d, "40 tracks", "seek40", cyl35(d, 20), cyl35(d, 60));
        seek(d, "79 tracks", "seek79", 0, cyl35(d, 79));
    } else {
        seek(d, "full stroke", "stroke", 0, d->blocks - 1);
    }
    if (d->blocks == 1600) {
        unsigned a = cyl35(d, 20);
        unsigned long t = alternate(d, a, a + 11, 12);
        out("4. Side switches (cylinder 20): %s ms per read\n", tenths(t));
        if (t)
            sum("side", "%s", tenths(t));
    }
}

static void test_writes(Disk *d)
{
    /* The second half of the disk, away from the volume's directory and bitmap. */
    unsigned count = d->blocks <= 320 ? 96 : d->blocks <= 1600 ? 400 : 1024;
    unsigned first = d->blocks / 2, at, k, i;
    unsigned long wms = 0, rms = 0, w2ms = 0;
    unsigned bad = 0, bad_after = 0, errors = 0;
    if (first + count > d->blocks)
        count = d->blocks - first;
    out("5. Changed-data writes: %u blocks from %u written with their complement, read back\n"
        "   and compared, then written back as they were (%u at a time)\n", count, first, SLICE);
    for (at = 0; at < count; at += SLICE) {
        unsigned n = count - at < SLICE ? count - at : SLICE;
        for (k = 0; k < n; k++)
            errors += rd(d, first + at + k, orig + k * BLOCK) != 0;
        timer_start();
        for (k = 0; k < n; k++) {
            for (i = 0; i < BLOCK; i++)
                buf[i] = ~orig[k * BLOCK + i];
            errors += wr(d, first + at + k, buf) != 0;
        }
        wms += timer_ms();
        timer_start();
        for (k = 0; k < n; k++) {
            errors += rd(d, first + at + k, buf) != 0;
            for (i = 0; i < BLOCK; i++)
                if ((unsigned char)(buf[i] ^ orig[k * BLOCK + i]) != 0xFF) { /* cc65 2.19 gets (unsigned char)~x wrong */
                    bad++;
                    break;
                }
        }
        rms += timer_ms();
        timer_start();
        for (k = 0; k < n; k++)
            errors += wr(d, first + at + k, orig + k * BLOCK) != 0;
        w2ms += timer_ms();
        for (k = 0; k < n; k++) {
            errors += rd(d, first + at + k, buf2) != 0;
            if (memcmp(buf2, orig + k * BLOCK, BLOCK))
                bad_after++;
        }
        if (errors) {
            out("   I/O ERRORS (write-protected?): stopped\n");
            break;
        }
    }
    if (wms)
        out("   complement write %s KB/s, read back %s KB/s, restore write %s KB/s\n", tenths(kbps10(count, wms)),
            tenths(kbps10(count, rms)), tenths(kbps10(count, w2ms)));
    out("   %u bad blocks, %u after restoring, %u I/O errors\n", bad, bad_after, errors);
    if (wms)
        sum("chg_w", "%s", tenths(kbps10(count, wms)));
    sum("chg_bad", "%u", bad + bad_after + errors);
}

static void test_file(Disk *d)
{
    char path[40];
    unsigned kb = d->blocks <= 320 ? 16 : 64, i;
    unsigned long wms, rms;
    int fd;
    if (!d->volume[0]) {
        out("6. File test: no ProDOS volume on this disk, skipped\n");
        return;
    }
    sprintf(path, "/%s/BENCH.TMP", d->volume);
    out("6. File test: %u KB written to %s and closed, read back, deleted\n", kb, path);
    for (i = 0; i < BLOCK; i++)
        buf[i] = i * 7;
    _filetype = PRODOS_T_BIN;
    remove(path);
    timer_start();
    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd < 0) {
        out("   can't create it (error %d)\n", _oserror);
        return;
    }
    for (i = 0; i < kb * 2; i++)
        if (write(fd, buf, BLOCK) != BLOCK) {
            out("   write error %d (disk full?)\n", _oserror);
            close(fd);
            remove(path);
            return;
        }
    close(fd);
    wms = timer_ms();
    timer_start();
    fd = open(path, O_RDONLY);
    for (i = 0; fd >= 0 && i < kb * 2; i++)
        if (read(fd, buf, BLOCK) != BLOCK)
            break;
    close(fd);
    rms = timer_ms();
    remove(path);
    if (i < kb * 2) {
        out("   read error %d after %u blocks\n", _oserror, i);
        return;
    }
    if (wms) {
        out("   write %s KB/s, read %s KB/s\n", tenths(kbps10(kb * 2, wms)), tenths(kbps10(kb * 2, rms)));
        sum("file_w", "%s", tenths(kbps10(kb * 2, wms)));
        sum("file_r", "%s", tenths(kbps10(kb * 2, rms)));
    }
}

/* ---- Main ---- */

static unsigned char rom;

static void header(Disk *d, char *label)
{
    unsigned char t[3];
    t[0] = t[1] = t[2] = 0;
    if (rom)
        rtc_read(t);
    out("\n==== %s ====\n", label[0] ? label : "(no label)");
    out("A2Bench %s, %s", VERSION, rom ? "IIgs" : "not a IIgs (no timing)");
    if (rom)
        out(" ROM %u, %s, %u frames/s, clock %02u:%02u", rom - 1, fast() ? "fast" : "normal speed", fps, t[2], t[1]);
    out("\nUnit S%uD%u: %s, %u blocks, volume /%s\n", d->dev & 7, (d->dev >> 3) + 1, d->kind, d->blocks,
        d->volume[0] ? d->volume : "(none)");
    strcpy(summary, label[0] ? label : "(no label)");
    sum("machine", rom ? "IIgs ROM %u" : "Apple II", rom - 1);
    sum("speed", "%s", rom && fast() ? "fast" : "normal");
    sum("drive", "%s", d->kind);
    sum("blocks", "%u", d->blocks);
}

int main(void)
{
    unsigned char devs[14], count, i, dev;
    static Disk disks[14];
    char line[64];
    Disk *d;
    int writes;

#ifdef __APPLE2ENH__
    col80();
#endif
    home();
    getcwd(prefix, sizeof prefix);
    if (prefix[0] && prefix[strlen(prefix) - 1] != '/')
        strcat(prefix, "/");
    rom = iigs();
    say("A2Bench %s: ProDOS block device timings for comparing drives and emulators.\n", VERSION);
    if (rom) {
        puts_("Timing with the IIgs clock...");
        measure_fps();
        say(" %u frames a second.\n", fps);
    } else {
        puts_("Not a IIgs: the tests run, but nothing can be timed.\n");
    }
    say("Results go to %s.\n\n", prefix);

    for (;;) {
        count = 0;
        for (dev = getfirstdevice(); dev != INVALID_DEVICE && count < 14; dev = getnextdevice(dev)) {
            Disk *e = &disks[count];
            memset(e, 0, sizeof *e);
            e->dev = dev;
            if (!getdevicedir(dev, line, sizeof line))
                line[0] = 0;
            strncpy(e->volume, line[0] == '/' ? line + 1 : line, 16);
            devs[count++] = dev;
        }
        puts_("Devices:\n");
        for (i = 0; i < count; i++)
            say("  %c: S%uD%u  %s\n", 'A' + i, devs[i] & 7, (devs[i] >> 3) + 1,
                    disks[i].volume[0] ? disks[i].volume : "(no ProDOS volume, or no disk)");
        puts_("Test which (letter), R to rescan, Q to quit? ");
        dev = toupper(cgetc());
        puts_("\n");
        if (dev == 'Q')
            break;
        if (dev < 'A' || dev >= 'A' + count)
            continue;
        d = &disks[dev - 'A'];
        d->h = dio_open(d->dev);
        if (!d->h) {
            puts_("Can't open it.\n\n");
            continue;
        }
        d->blocks = dio_query_sectcount(d->h);
        if (!d->blocks || rd(d, 0, buf)) {
            puts_("Can't read it (no disk?).\n\n");
            dio_close(d->h);
            continue;
        }
        d->kind = kind_of(d->blocks);
        say("%s, %u blocks. Label for this run (e.g. \"real 3.5 drive\", \"rp2350 800K .po\"):\n", d->kind,
                d->blocks);
        line_in(line, 60);
        writes = yes("Run write tests? They write blocks with other data and then put them back,\n"
                     "and write a temporary file");
        results[0] = 0;
        header(d, line);
        test_sequential(d);
        test_random(d);
        test_seeks(d);
        if (writes) {
            test_writes(d);
            test_file(d);
        } else {
            out("5-6. Write tests skipped\n");
        }
        dio_close(d->h);
        puts_("\n");
        save();
    }
    return 0;
}
