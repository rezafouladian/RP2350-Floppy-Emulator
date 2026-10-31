/*
 * FloppyBench: times a Macintosh floppy drive, or an HD20 (or any other drive in the drive queue),
 * from the Mac's side, in absolute units, so different drives, emulators and disk-image types can
 * be compared.
 *
 * Floppy tests, on one disk (1-6 as in FloppyBench 1.0, so old results still compare):
 *   0. (after an eject) Insert to mounted: the next disk's disk-in to its volume mounted.
 *   1. Raw sequential read of the whole disk through the .Sony driver, 8 KB per request.
 *   2. Raw read of the first 10 cylinders, one 512-byte block per request.
 *   3. Seeks: alternate reads between cylinders 1, 10, 40 and 79 tracks apart.
 *   4. Side switches: alternate reads between the two sides of one cylinder.
 *   5. (optional) Raw rewrite: every block written back with its own contents.
 *   6. (optional) File write, flush and uncached read-back through the File Manager.
 *   7. Random single-block reads (the same sequence every run): min, median, max.
 *   8. Raw sequential read, one whole cylinder (both sides) per request.
 *   9. (optional) Changed-data writes: cylinders 0-9 written with their complement, read back and
 *      compared, then written back as they were and compared again (catches dropped writes,
 *      which test 5 can't).
 *  10. (optional) Format and verify through the driver, the disk's contents kept in memory and
 *      written back afterwards.
 *
 * Hard disk tests (an HD20, or a drive of another driver), on the first part of the disk:
 *   H1. Sequential reads at 512 B, 4 KB and 32 KB per request.
 *   H2. Random single-block reads across the whole disk.
 *   H3. Full stroke: alternating reads of the first and last block.
 *   H4. (optional) Changed-data writes over 512 KB, as test 9.
 *   H5. (optional) File test, 1 MB.
 *
 * Raw reads go to the drive's driver, so the File Manager's cache isn't involved; the .Sony
 * driver's own track cache is left as the system set it, since that's what real use goes through.
 * Results are shown and appended to "FloppyBench Results" (a SimpleText file) next to the app,
 * and each run's numbers as one line of key=value pairs to "FloppyBench Summary"
 * (tools/bench_table.rs makes a table of them).
 *
 * Build with Retro68 (see README.md). Plain C, 68000 code, System 6.0.8 or later. The text window
 * is drawn with plain Toolbox calls: Retro68's console library pulls in most of libstdc++ (an
 * 800 KB app), and this way the app fits on any floppy.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <Devices.h>
#include <Files.h>
#include <Gestalt.h>
#include <OSUtils.h>
#include <Events.h>
#include <Fonts.h>
#include <Memory.h>
#include <Quickdraw.h>
#include <TextEdit.h>
#include <TextUtils.h>
#include <Windows.h>

#define VERSION "1.1"
#define SONY_REFNUM (-5)
#define BLOCK 512
#define CHUNK_BLOCKS 16 /* 8 KB per request in the sequential read */
#define BUF_BLOCKS 64   /* the I/O buffer: 32 KB */
#define FILE_TEST_BYTES (100L * 1024)
#define HD_FILE_TEST_BYTES (1024L * 1024)
#define RANDOM_READS 64
#define NO_CACHE 0x20 /* ioPosMode: don't keep the data in the File Manager's cache */
/* The drive queue header, a low-memory global. (GetDrvQHdr is only glue returning this address,
   and Multiversal doesn't provide it.) */
#define DRVQ_HDR ((QHdrPtr)0x308)
#ifndef fsRtDirID
#define fsRtDirID 2 /* a volume's root directory */
#endif

/* ---- Timing ---- */

static int have_microseconds;

/* _Microseconds (System 7): high word in A0, low word in D0. */
static uint32_t microseconds(void)
{
    if (have_microseconds) {
        register uint32_t lo __asm__("d0");
        register uint32_t hi __asm__("a0");
        __asm__ volatile(".short 0xA193" : "=r"(lo), "=r"(hi) : : "d1", "d2", "a1", "cc", "memory");
        (void)hi;
        return lo;
    }
    /* 1/60 s ticks. */
    return TickCount() * 16667UL;
}

static void init_timing(void)
{
    /* Microseconds is a trap of its own; compare it with the "unimplemented" trap. */
    have_microseconds = GetOSTrapAddress(0xA193) != GetToolTrapAddress(0xA89F);
}

/* ---- A text window: output lines and line input ---- */

#define CON_LINES 300
#define CON_COLS 100

static char con_text[CON_LINES][CON_COLS + 1];
static int con_first, con_count = 1; /* ring of lines; the last is the one being written */
static WindowPtr con_win;
static short con_line_h, con_descent, con_rows, con_cols;

static char *con_line(int i) { return con_text[(con_first + i) % CON_LINES]; }

static void con_draw(void)
{
    SetPort(con_win);
    EraseRect(&con_win->portRect);
    int top = con_count > con_rows ? con_count - con_rows : 0;
    for (int r = 0; top + r < con_count; r++) {
        const char *l = con_line(top + r);
        MoveTo(4, (r + 1) * con_line_h - con_descent);
        DrawText((Ptr)l, 0, strlen(l));
    }
}

static void con_newline(void)
{
    if (con_count == CON_LINES)
        con_first = (con_first + 1) % CON_LINES;
    else
        con_count++;
    con_line(con_count - 1)[0] = 0;
}

static void con_puts(const char *s)
{
    for (; *s; s++) {
        char *l = con_line(con_count - 1);
        size_t n = strlen(l);
        if (*s == '\n') {
            con_newline();
        } else {
            if ((int)n >= con_cols) {
                con_newline();
                l = con_line(con_count - 1);
                n = 0;
            }
            l[n] = *s;
            l[n + 1] = 0;
        }
    }
    con_draw();
}

static void show(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void show(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    con_puts(line);
}

static void con_init(void)
{
    InitGraf(&qd.thePort);
    InitFonts();
    InitWindows();
    InitMenus();
    TEInit();
    InitDialogs(NULL);
    InitCursor();
    FlushEvents(everyEvent, 0);

    Rect r = qd.screenBits.bounds;
    r.top += 40; /* menu bar and title bar */
    InsetRect(&r, 8, 8);
    r.top -= 8 - 4;
    Str255 title;
    title[0] = 11;
    memcpy(title + 1, "FloppyBench", 11);
    con_win = NewWindow(NULL, &r, title, true, documentProc, (WindowPtr)-1, true, 0);
    SetPort(con_win);
    TextFont(4); /* Monaco */
    TextSize(9);
    FontInfo fi;
    GetFontInfo(&fi);
    con_line_h = fi.ascent + fi.descent + fi.leading;
    con_descent = fi.descent + fi.leading;
    con_rows = (con_win->portRect.bottom - con_win->portRect.top) / con_line_h;
    con_cols = (con_win->portRect.right - con_win->portRect.left - 8) / CharWidth('0');
    if (con_cols > CON_COLS)
        con_cols = CON_COLS;
}

/* Handle one event while waiting for input: window updates, dragging, closing, Cmd-Q. Returns
   the typed character, or 0. (GetNextEvent also mounts a newly inserted disk's volume.) */
static char con_event(void)
{
    EventRecord ev;
    SystemTask();
    if (!GetNextEvent(everyEvent, &ev))
        return 0;
    WindowPtr w;
    switch (ev.what) {
    case keyDown:
    case autoKey: {
        char c = ev.message & charCodeMask;
        if ((ev.modifiers & cmdKey) && (c == 'q' || c == 'Q'))
            ExitToShell();
        return c;
    }
    case updateEvt:
        BeginUpdate((WindowPtr)ev.message);
        con_draw();
        EndUpdate((WindowPtr)ev.message);
        break;
    case mouseDown:
        switch (FindWindow(ev.where, &w)) {
        case inDrag:
            DragWindow(w, ev.where, &qd.screenBits.bounds);
            break;
        case inGoAway:
            if (TrackGoAway(w, ev.where))
                ExitToShell();
            break;
        case inSysWindow:
            SystemClick(&ev, w);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    return 0;
}

static void read_line(char *buf, int size)
{
    int n = 0;
    buf[0] = 0;
    for (;;) {
        char c = con_event();
        if (!c)
            continue;
        if (c == '\r' || c == 3) { /* Return or Enter */
            con_puts("\n");
            return;
        }
        if (c == 8) { /* Delete */
            if (n > 0) {
                buf[--n] = 0;
                char *l = con_line(con_count - 1);
                size_t len = strlen(l);
                if (len)
                    l[len - 1] = 0;
                con_draw();
            }
            continue;
        }
        if ((unsigned char)c >= ' ' && n < size - 1) {
            buf[n++] = c;
            buf[n] = 0;
            char one[2] = {c, 0};
            con_puts(one);
        }
    }
}

/* ---- Output: the window, the results file and the summary line ---- */

static short results_ref; /* 0 if the file couldn't be opened */

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void out(const char *fmt, ...)
{
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    con_puts(line);
    if (results_ref) {
        /* SimpleText wants CR line ends. */
        long n = strlen(line);
        for (long i = 0; i < n; i++)
            if (line[i] == '\n')
                line[i] = '\r';
        FSWrite(results_ref, &n, line);
    }
}

static void pstr(Str255 dst, const char *src)
{
    size_t n = strlen(src);
    if (n > 255)
        n = 255;
    dst[0] = (unsigned char)n;
    memcpy(dst + 1, src, n);
}

static void cstr(char *dst, const unsigned char *src)
{
    memcpy(dst, src + 1, src[0]);
    dst[src[0]] = 0;
}

/* Open (creating if need be) a text file in the app's folder, positioned at its end. */
static short open_append(const char *file)
{
    Str255 name;
    short ref;
    pstr(name, file);
    /* vRefNum 0, dirID 0: the default directory, the app's folder at launch. */
    OSErr err = HCreate(0, 0, name, 'ttxt', 'TEXT');
    if (err != noErr && err != dupFNErr)
        return 0;
    if (HOpen(0, 0, name, fsRdWrPerm, &ref) != noErr)
        return 0;
    SetFPos(ref, fsFromLEOF, 0);
    return ref;
}

static void open_results(void) { results_ref = open_append("FloppyBench Results"); }

static void close_results(void)
{
    if (results_ref) {
        FSClose(results_ref);
        FlushVol(NULL, 0);
        results_ref = 0;
    }
}

/* The run's summary: tab-separated key=value pairs, written as one line at the end. */
static char summary[1024];

static void sum(const char *key, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sum(const char *key, const char *fmt, ...)
{
    char v[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(v, sizeof v, fmt, ap);
    va_end(ap);
    size_t n = strlen(summary);
    snprintf(summary + n, sizeof summary - n, "\t%s=%s", key, v);
}

static void write_summary(void)
{
    short ref = open_append("FloppyBench Summary");
    if (!ref)
        return;
    size_t n = strlen(summary);
    if (n < sizeof summary - 1)
        summary[n++] = '\r';
    long len = n;
    FSWrite(ref, &len, summary);
    FSClose(ref);
    FlushVol(NULL, 0);
}

/* ---- Input ---- */

static int ask_yes(const char *question)
{
    char a[16];
    show("%s (y/N) ", question);
    read_line(a, sizeof a);
    return a[0] == 'y' || a[0] == 'Y';
}

/* ---- Drives and disks ---- */

typedef struct {
    short drive;
    short refnum;      /* the drive's driver */
    int disk_in;
    int locked;
    int hard;          /* not a floppy: an HD20 or another driver's disk */
    long queue_blocks; /* size from the drive queue, 0 if none */
    long blocks;       /* disk size, 0 if unknown */
    const char *format;
    char kind[32];
    short vref;        /* mounted volume, 0 if none */
    char volume[64];
    long free_bytes;
} Disk;

/* Flag bytes just before a drive queue element. */
static int dq_disk_in(DrvQEl *q) { return ((unsigned char *)q)[-3] == 1 || ((unsigned char *)q)[-3] == 2; }
static int dq_locked(DrvQEl *q) { return (((unsigned char *)q)[-4] & 0x80) != 0; }

static OSErr raw_io(const Disk *d, int write, long block, long count, void *buf)
{
    ParamBlockRec pb;
    memset(&pb, 0, sizeof pb);
    pb.ioParam.ioRefNum = d->refnum;
    pb.ioParam.ioVRefNum = d->drive;
    pb.ioParam.ioBuffer = buf;
    pb.ioParam.ioReqCount = count * BLOCK;
    pb.ioParam.ioPosMode = fsFromStart;
    pb.ioParam.ioPosOffset = block * BLOCK;
    return write ? PBWriteSync(&pb) : PBReadSync(&pb);
}

/* Reads and writes of a run of blocks in requests of `chunk` blocks; nonzero if all worked. */
static int raw_run(const Disk *d, int write, long first, long count, long chunk, char *buf)
{
    for (long b = 0; b < count; b += chunk) {
        long n = count - b < chunk ? count - b : chunk;
        if (raw_io(d, write, first + b, n, buf + b * BLOCK) != noErr)
            return 0;
    }
    return 1;
}

static int readable(const Disk *d, long block, char *buf) { return raw_io(d, 0, block, 1, buf) == noErr; }

static OSErr control(const Disk *d, short code, short param)
{
    ParamBlockRec pb;
    memset(&pb, 0, sizeof pb);
    pb.cntrlParam.ioCRefNum = d->refnum;
    pb.cntrlParam.ioVRefNum = d->drive;
    pb.cntrlParam.csCode = code;
    pb.cntrlParam.csParam[0] = param;
    return PBControlSync(&pb);
}

/* The .Sony driver's drive kind (control 23, drive info): 2 400K, 3 800K, 4 SuperDrive, 7 HD20;
   0 if the call isn't there. */
static int drive_kind(const Disk *d)
{
    ParamBlockRec pb;
    memset(&pb, 0, sizeof pb);
    pb.cntrlParam.ioCRefNum = d->refnum;
    pb.cntrlParam.ioVRefNum = d->drive;
    pb.cntrlParam.csCode = 23;
    if (PBControlSync(&pb) != noErr)
        return 0;
    return (unsigned char)pb.cntrlParam.csParam[1];
}

/* A floppy's size: the last block that reads, of the four floppy sizes. */
static void probe_floppy(Disk *d, char *buf)
{
    static const struct { long blocks; const char *name; } sizes[] = {
        {2880, "1.44 MB MFM"}, {1600, "800K GCR"}, {1440, "720K MFM"}, {800, "400K GCR"},
    };
    d->blocks = 0;
    d->format = "unknown";
    for (unsigned i = 0; i < sizeof sizes / sizeof sizes[0]; i++) {
        if (readable(d, sizes[i].blocks - 1, buf)) {
            d->blocks = sizes[i].blocks;
            d->format = sizes[i].name;
            return;
        }
    }
}

/* A hard disk's size: the drive queue's, if its last block reads and the next doesn't; else the
   first block that doesn't read, found by doubling and halving. */
static void probe_hard(Disk *d, char *buf)
{
    static char name[40];
    long q = d->queue_blocks;
    long size;
    if (q > 0 && readable(d, q - 1, buf) && !readable(d, q, buf)) {
        size = q;
    } else if (!readable(d, 0, buf)) {
        size = 0;
    } else {
        long lo = 0, hi = 1024; /* lo reads; is hi past the end? */
        while (hi < (1L << 25) && readable(d, hi, buf)) {
            lo = hi;
            hi *= 2;
        }
        while (hi - lo > 1) {
            long mid = lo + (hi - lo) / 2;
            if (readable(d, mid, buf))
                lo = mid;
            else
                hi = mid;
        }
        size = lo + 1;
    }
    d->blocks = size;
    snprintf(name, sizeof name, "%s, %.1f MB", d->kind, size / 2048.0);
    d->format = name;
}

static void find_volume(Disk *d)
{
    HParamBlockRec pb;
    Str255 name;
    d->vref = 0;
    d->volume[0] = 0;
    for (short i = 1;; i++) {
        memset(&pb, 0, sizeof pb);
        pb.volumeParam.ioNamePtr = name;
        pb.volumeParam.ioVolIndex = i;
        if (PBHGetVInfoSync(&pb) != noErr)
            return;
        if (pb.volumeParam.ioVDrvInfo == d->drive && pb.volumeParam.ioVDRefNum == d->refnum) {
            d->vref = pb.volumeParam.ioVRefNum;
            cstr(d->volume, name);
            d->free_bytes = (long)pb.volumeParam.ioVFrBlk * pb.volumeParam.ioVAlBlkSiz;
            return;
        }
    }
}

/* All drives in the drive queue. */
static int scan_drives(Disk *disks, int max)
{
    int count = 0;
    QHdrPtr qh = DRVQ_HDR;
    for (DrvQEl *q = (DrvQEl *)qh->qHead; q && count < max; q = (DrvQEl *)q->qLink) {
        Disk *d = &disks[count++];
        memset(d, 0, sizeof *d);
        d->drive = q->dQDrive;
        d->refnum = q->dQRefNum;
        d->disk_in = dq_disk_in(q);
        d->locked = dq_locked(q);
        d->queue_blocks = q->qType == 1 ? (long)q->dQDrvSz2 << 16 | q->dQDrvSz : q->dQDrvSz;
        d->format = "";
        if (d->refnum == SONY_REFNUM)
            strcpy(d->kind, "floppy");
        else
            snprintf(d->kind, sizeof d->kind, "driver %d", d->refnum);
        if (d->disk_in)
            find_volume(d);
    }
    return count;
}

/* What kind of drive it is, and the disk's size. A .Sony drive is an HD20 if the driver says
   so, if it isn't the .Sony's own refnum (the ROM installs DCDs under -2), or if a block past
   the biggest floppy reads. */
static void probe(Disk *d, char *buf)
{
    int kind = d->refnum == SONY_REFNUM ? drive_kind(d) : 0;
    if (kind == 7 || d->refnum == -2) {
        d->hard = 1;
        strcpy(d->kind, "HD20");
    } else if (d->refnum != SONY_REFNUM) {
        d->hard = 1;
    } else if (readable(d, 2880, buf)) {
        d->hard = 1;
        strcpy(d->kind, "HD20");
    } else {
        d->hard = 0;
        strcpy(d->kind, kind == 4 ? "SuperDrive" : kind == 3 ? "800K drive" : kind == 2 ? "400K drive" : "floppy");
    }
    if (d->hard)
        probe_hard(d, buf);
    else
        probe_floppy(d, buf);
}

/* Unmount the disk's volume, if it has one, and eject it, so another disk can go in without
   quitting. The traps are called directly: Multiversal declares the UnmountVol and DiskEject glue
   but doesn't provide it. Returns nonzero if the disk was ejected. */
static int eject_disk(Disk *d)
{
    ParamBlockRec pb;
    OSErr err;
    if (d->vref) {
        memset(&pb, 0, sizeof pb);
        pb.volumeParam.ioVRefNum = d->vref;
        err = PBUnmountVol(&pb);
        if (err != noErr) {
            show("Can't unmount \"%s\" (error %d%s).\n\n", d->volume, err,
                 err == fBsyErr ? ": a file on it is open, or the app runs from it" : "");
            return 0;
        }
    }
    err = control(d, 7, 0); /* ejectCode */
    if (err != noErr) {
        show("Eject failed (error %d).\n\n", err);
        return 0;
    }
    return 1;
}

/* After an eject: wait for a disk in the same drive, mounted by GetNextEvent. Returns the time
   from disk-in to mounted (0 if it had no volume), or -1 if a key was pressed. */
static long wait_for_disk(const Disk *old)
{
    show("Ejected drive %d. Insert the next disk (any key goes back to the list)...\n", old->drive);
    Disk disks[8];
    uint32_t in_at = 0;
    for (;;) {
        if (con_event())
            return -1;
        int n = scan_drives(disks, 8);
        for (int i = 0; i < n; i++) {
            Disk *d = &disks[i];
            if (d->drive != old->drive || d->refnum != old->refnum || !d->disk_in)
                continue;
            if (!in_at)
                in_at = microseconds();
            if (d->vref)
                return microseconds() - in_at;
            if (microseconds() - in_at > 15000000UL)
                return 0; /* no volume (an unformatted or foreign disk) */
        }
    }
}

/* First block of a cylinder (side 0). GCR has 12..8 sectors per side by zone of 16 tracks. */
static long cylinder_block(const Disk *d, int cyl)
{
    if (d->blocks == 2880)
        return cyl * 36L;
    if (d->blocks == 1440)
        return cyl * 18L;
    int sides = d->blocks == 800 ? 1 : 2;
    long b = 0;
    for (int c = 0; c < cyl; c++)
        b += (12 - c / 16) * sides;
    return b;
}

static int sides_of(const Disk *d) { return d->blocks == 800 ? 1 : 2; }
static int side_blocks(const Disk *d, int cyl)
{
    if (d->blocks == 2880)
        return 18;
    if (d->blocks == 1440)
        return 9;
    return 12 - cyl / 16;
}

/* ---- Measuring ---- */

static double kbps(long bytes, uint32_t us) { return us ? bytes / 1024.0 / (us / 1e6) : 0; }

/* The same pseudo-random blocks every run. */
static uint32_t rand_state;
static long next_random(long blocks)
{
    rand_state = rand_state * 1103515245UL + 12345;
    return (long)((rand_state >> 8) % (uint32_t)blocks);
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : x > y;
}

/* Min, median, max and mean of n times (sorted in place), in ms. */
typedef struct { double min, median, max, mean; } Stats;
static Stats stats(uint32_t *t, int n)
{
    Stats s;
    double total = 0;
    qsort(t, n, sizeof *t, cmp_u32);
    for (int i = 0; i < n; i++)
        total += t[i];
    s.min = t[0] / 1e3;
    s.median = t[n / 2] / 1e3;
    s.max = t[n - 1] / 1e3;
    s.mean = total / n / 1e3;
    return s;
}

/* Sequential raw read of `count` blocks from `first`, `chunk` blocks per request. Returns the
   time, 0 on an error. */
static uint32_t timed_read(const Disk *d, long first, long count, long chunk, char *buf, uint32_t *slowest, long *slow_at)
{
    uint32_t start = microseconds();
    for (long b = first; b < first + count; b += chunk) {
        long n = first + count - b < chunk ? first + count - b : chunk;
        uint32_t t0 = microseconds();
        OSErr err = raw_io(d, 0, b, n, buf);
        uint32_t t = microseconds() - t0;
        if (err != noErr) {
            out("   read error %d at block %ld\n", err, b);
            return 0;
        }
        if (slowest && t > *slowest) {
            *slowest = t;
            *slow_at = b;
        }
    }
    uint32_t total = microseconds() - start;
    return total ? total : 1;
}

/* Alternate single-block reads between two places; time per read. */
static uint32_t alternate(const Disk *d, long a, long b, int rounds, char *buf)
{
    raw_io(d, 0, a, 1, buf); /* start at a */
    uint32_t start = microseconds();
    for (int i = 0; i < rounds; i++) {
        if (raw_io(d, 0, b, 1, buf) != noErr || raw_io(d, 0, a, 1, buf) != noErr)
            return 0;
    }
    return (microseconds() - start) / (2 * rounds);
}

/* Random single-block reads over the whole disk. */
static void random_reads(const Disk *d, const char *title, char *buf)
{
    static uint32_t t[RANDOM_READS];
    rand_state = 0x5EED;
    raw_io(d, 0, 0, 1, buf);
    for (int i = 0; i < RANDOM_READS; i++) {
        long b = next_random(d->blocks);
        uint32_t t0 = microseconds();
        if (raw_io(d, 0, b, 1, buf) != noErr) {
            out("%s: read error at block %ld\n", title, b);
            return;
        }
        t[i] = microseconds() - t0;
    }
    Stats s = stats(t, RANDOM_READS);
    out("%s: %d reads, ms per read: mean %.1f, median %.1f, min %.1f, max %.1f\n", title, RANDOM_READS, s.mean,
        s.median, s.min, s.max);
    sum("rand_ms", "%.1f", s.mean);
    sum("rand_max", "%.0f", s.max);
}

/* Changed-data writes over a run of blocks, `chunk` per request: the originals kept in memory,
   the complement written and read back, then the originals written back and read back. */
static void changed_writes(const Disk *d, const char *title, long first, long count, long chunk, char *buf)
{
    char *orig = NewPtr(count * BLOCK);
    while (!orig && count > 4 * chunk) {
        count /= 2;
        orig = NewPtr(count * BLOCK);
    }
    if (!orig) {
        out("%s: not enough memory, skipped\n", title);
        return;
    }
    out("%s: %ld blocks from %ld written with their complement, %ld KB requests, read back and\n"
        "   compared, then written back as they were\n", title, count, first, chunk * BLOCK / 1024);
    if (!raw_run(d, 0, first, count, chunk, orig)) {
        out("   read error; skipped\n");
        DisposePtr(orig);
        return;
    }
    long bad = 0, bad_after = 0;
    uint32_t w = 0, r = 0, w2 = 0;
    int ok = 1;
    for (long b = 0; b < count && ok; b += chunk) {
        long n = count - b < chunk ? count - b : chunk;
        for (long i = 0; i < n * BLOCK; i++)
            buf[i] = ~orig[b * BLOCK + i];
        uint32_t t0 = microseconds();
        ok = raw_io(d, 1, first + b, n, buf) == noErr;
        w += microseconds() - t0;
    }
    for (long b = 0; b < count && ok; b += chunk) {
        long n = count - b < chunk ? count - b : chunk;
        uint32_t t0 = microseconds();
        ok = raw_io(d, 0, first + b, n, buf) == noErr;
        r += microseconds() - t0;
        for (long i = 0; ok && i < n * BLOCK; i += BLOCK)
            for (long k = 0; k < BLOCK; k++)
                if ((char)~buf[i + k] != orig[b * BLOCK + i + k]) {
                    bad++;
                    break;
                }
    }
    /* Always try to put the originals back. */
    uint32_t t0 = microseconds();
    int restored = raw_run(d, 1, first, count, chunk, orig);
    w2 = microseconds() - t0;
    for (long b = 0; b < count && restored; b += chunk) {
        long n = count - b < chunk ? count - b : chunk;
        if (raw_io(d, 0, first + b, n, buf) != noErr) {
            restored = 0;
            break;
        }
        for (long i = 0; i < n * BLOCK; i += BLOCK)
            if (memcmp(buf + i, orig + b * BLOCK + i, BLOCK) != 0)
                bad_after++;
    }
    long bytes = count * BLOCK;
    if (!ok)
        out("   I/O ERROR during the complement pass\n");
    out("   complement write %.1f KB/s, read back %.1f KB/s: %ld bad blocks; restore write %.1f KB/s: %s\n",
        kbps(bytes, w), kbps(bytes, r), bad, kbps(bytes, w2),
        !restored ? "RESTORE FAILED" : bad_after ? "BAD BLOCKS AFTER RESTORE" : "all as before");
    sum("chg_w", "%.1f", kbps(bytes, w));
    sum("chg_bad", "%ld", bad + bad_after + (!ok || !restored));
    DisposePtr(orig);
}

/* ---- Floppy tests ---- */

static void test_sequential(const Disk *d, char *buf)
{
    out("1. Sequential raw read, %d KB requests\n", CHUNK_BLOCKS * BLOCK / 1024);
    uint32_t slowest = 0;
    long slow_at = 0;
    uint32_t total = timed_read(d, 0, d->blocks, CHUNK_BLOCKS, buf, &slowest, &slow_at);
    if (!total)
        return;
    out("   %ld KB in %.2f s = %.1f KB/s; slowest request %.0f ms (block %ld)\n", d->blocks / 2,
        total / 1e6, kbps(d->blocks * BLOCK, total), slowest / 1e3, slow_at);
    sum("seq8k", "%.1f", kbps(d->blocks * BLOCK, total));
}

static void test_blockwise(const Disk *d, char *buf)
{
    long end = cylinder_block(d, 10);
    out("2. Raw read of cylinders 0-9, one block per request (%ld blocks)\n", end);
    uint32_t total = timed_read(d, 0, end, 1, buf, NULL, NULL);
    if (!total)
        return;
    out("   %.2f s = %.1f KB/s, %.1f ms per block\n", total / 1e6, kbps(end * BLOCK, total), total / 1e3 / end);
    sum("block_ms", "%.1f", total / 1e3 / end);
}

static void test_seeks(const Disk *d, char *buf)
{
    out("3. Seeks: alternating single-block reads, ms per read including the step\n");
    static const int dist[] = {1, 10, 40, 79};
    for (unsigned i = 0; i < sizeof dist / sizeof dist[0]; i++) {
        int from = dist[i] == 79 ? 0 : 20, to = from + dist[i];
        if (to > 79)
            to = 79;
        uint32_t t = alternate(d, cylinder_block(d, from), cylinder_block(d, to), 6, buf);
        out("   %2d tracks (cyl %2d <-> %2d): %.0f ms\n", dist[i], from, to, t / 1e3);
        char key[16];
        snprintf(key, sizeof key, "seek%d", dist[i]);
        sum(key, "%.0f", t / 1e3);
    }
}

static void test_sides(const Disk *d, char *buf)
{
    if (sides_of(d) < 2) {
        out("4. Side switches: single-sided disk, skipped\n");
        return;
    }
    long a = cylinder_block(d, 20);
    uint32_t t = alternate(d, a, a + side_blocks(d, 20), 8, buf);
    out("4. Side switches (cylinder 20, side 0 <-> 1): %.0f ms per read\n", t / 1e3);
    sum("side", "%.0f", t / 1e3);
}

static void test_rewrite(const Disk *d, char *buf)
{
    out("5. Raw rewrite of every block with its own contents, %d KB requests\n", CHUNK_BLOCKS * BLOCK / 1024);
    uint32_t writing = 0, slowest = 0;
    for (long b = 0; b < d->blocks; b += CHUNK_BLOCKS) {
        long n = d->blocks - b < CHUNK_BLOCKS ? d->blocks - b : CHUNK_BLOCKS;
        OSErr err = raw_io(d, 0, b, n, buf);
        if (err != noErr) {
            out("   read error %d at block %ld; stopped\n", err, b);
            return;
        }
        uint32_t t0 = microseconds();
        err = raw_io(d, 1, b, n, buf);
        uint32_t t = microseconds() - t0;
        if (err != noErr) {
            out("   write error %d at block %ld; stopped\n", err, b);
            return;
        }
        writing += t;
        if (t > slowest)
            slowest = t;
    }
    out("   writing took %.2f s = %.1f KB/s; slowest request %.0f ms\n", writing / 1e6,
        kbps(d->blocks * BLOCK, writing), slowest / 1e3);
    sum("rewrite", "%.1f", kbps(d->blocks * BLOCK, writing));
    /* Read it all back: any error means a write didn't stick. */
    for (long b = 0; b < d->blocks; b += CHUNK_BLOCKS) {
        long n = d->blocks - b < CHUNK_BLOCKS ? d->blocks - b : CHUNK_BLOCKS;
        if (raw_io(d, 0, b, n, buf) != noErr) {
            out("   VERIFY FAILED at block %ld\n", b);
            return;
        }
    }
    out("   read back without errors\n");
}

static void test_file(const Disk *d, const char *title, long size, char *buf)
{
    if (!d->vref) {
        out("%s: no volume mounted on this disk, skipped\n", title);
        return;
    }
    if (d->free_bytes < size + 8192) {
        out("%s: only %ld KB free, skipped\n", title, d->free_bytes / 1024);
        return;
    }
    out("%s: %ld KB written, flushed, read back uncached (the disk cache may still\n"
        "   serve part of the read; a small cache in the Memory control panel helps)\n", title, size / 1024);
    Str255 name;
    pstr(name, "FloppyBench temp");
    HDelete(d->vref, fsRtDirID, name);
    OSErr err = HCreate(d->vref, fsRtDirID, name, '????', 'BINA');
    short ref;
    if (err == noErr)
        err = HOpen(d->vref, fsRtDirID, name, fsRdWrPerm, &ref);
    if (err != noErr) {
        out("   can't create the file (error %d)\n", err);
        return;
    }
    for (long i = 0; i < 8192; i++)
        buf[i] = (char)(i * 7);
    uint32_t t0 = microseconds();
    for (long done = 0; done < size && err == noErr; done += 8192) {
        long n = 8192;
        err = FSWrite(ref, &n, buf);
    }
    if (err == noErr)
        err = FlushVol(NULL, d->vref);
    uint32_t w = microseconds() - t0;
    if (err != noErr) {
        out("   write error %d\n", err);
    } else {
        ParamBlockRec pb;
        uint32_t t1 = microseconds();
        for (long done = 0; done < size && err == noErr; done += 8192) {
            memset(&pb, 0, sizeof pb);
            pb.ioParam.ioRefNum = ref;
            pb.ioParam.ioBuffer = buf;
            pb.ioParam.ioReqCount = 8192;
            pb.ioParam.ioPosMode = fsFromStart | NO_CACHE;
            pb.ioParam.ioPosOffset = done;
            err = PBReadSync(&pb);
        }
        uint32_t r = microseconds() - t1;
        if (err != noErr) {
            out("   read error %d\n", err);
        } else {
            out("   write+flush %.2f s = %.1f KB/s; read %.2f s = %.1f KB/s\n", w / 1e6, kbps(size, w),
                r / 1e6, kbps(size, r));
            sum("file_w", "%.1f", kbps(size, w));
            sum("file_r", "%.1f", kbps(size, r));
        }
    }
    FSClose(ref);
    HDelete(d->vref, fsRtDirID, name);
    FlushVol(NULL, d->vref);
}

static void test_random(const Disk *d, char *buf) { random_reads(d, "7. Random single-block reads", buf); }

static void test_cylinders(const Disk *d, char *buf)
{
    out("8. Sequential raw read, one cylinder (both sides) per request\n");
    uint32_t start = microseconds();
    for (int c = 0; c < 80; c++) {
        long first = cylinder_block(d, c), next = c == 79 ? d->blocks : cylinder_block(d, c + 1);
        if (raw_io(d, 0, first, next - first, buf) != noErr) {
            out("   read error at cylinder %d\n", c);
            return;
        }
    }
    uint32_t total = microseconds() - start;
    out("   %ld KB in %.2f s = %.1f KB/s\n", d->blocks / 2, total / 1e6, kbps(d->blocks * BLOCK, total));
    sum("cyl", "%.1f", kbps(d->blocks * BLOCK, total));
}

static void test_changed(const Disk *d, char *buf)
{
    changed_writes(d, "9. Changed-data writes", 0, cylinder_block(d, 10), CHUNK_BLOCKS, buf);
}

/* The format list's entry for the disk's own format (status 6), 1-based; 0 if there's no list,
   which the driver takes as its default (the old ROMs want 1 for a 400K disk). */
static short current_format(const Disk *d)
{
    struct { long blocks; long flags; } list[8];
    ParamBlockRec pb;
    memset(&pb, 0, sizeof pb);
    pb.cntrlParam.ioCRefNum = d->refnum;
    pb.cntrlParam.ioVRefNum = d->drive;
    pb.cntrlParam.csCode = 6;
    pb.cntrlParam.csParam[0] = 8;
    void *at = list;
    memcpy(&pb.cntrlParam.csParam[1], &at, sizeof at);
    if (PBStatusSync(&pb) != noErr)
        return d->blocks == 800 ? 1 : 0;
    for (short i = 0; i < pb.cntrlParam.csParam[0] && i < 8; i++)
        if (list[i].flags & 0x40000000L)
            return i + 1;
    for (short i = 0; i < pb.cntrlParam.csParam[0] && i < 8; i++)
        if (list[i].blocks == d->blocks)
            return i + 1;
    return 0;
}

static void test_format(const Disk *d, char *backup, char *buf)
{
    out("10. Format and verify through the driver, then the disk written back from memory\n");
    if (d->vref)
        FlushVol(NULL, d->vref);
    if (!raw_run(d, 0, 0, d->blocks, BUF_BLOCKS, backup)) {
        out("   read error making the copy; skipped\n");
        return;
    }
    short index = current_format(d);
    uint32_t t0 = microseconds();
    OSErr err = control(d, 6, index);
    uint32_t tf = microseconds() - t0;
    if (err != noErr) {
        out("   format (list entry %d) failed: error %d\n", index, err);
    } else {
        t0 = microseconds();
        err = control(d, 5, 0);
        uint32_t tv = microseconds() - t0;
        out("   format %.1f s, verify %.1f s%s\n", tf / 1e6, tv / 1e6, err ? "" : ", verify passed");
        if (err)
            out("   VERIFY FAILED: error %d\n", err);
        sum("format_s", "%.1f", tf / 1e6);
        sum("verify_s", "%.1f", tv / 1e6);
        sum("verify_ok", "%d", err == noErr);
    }
    t0 = microseconds();
    int ok = raw_run(d, 1, 0, d->blocks, CHUNK_BLOCKS, backup);
    uint32_t tw = microseconds() - t0;
    long bad = 0;
    for (long b = 0; ok && b < d->blocks; b += BUF_BLOCKS) {
        long n = d->blocks - b < BUF_BLOCKS ? d->blocks - b : BUF_BLOCKS;
        if (raw_io(d, 0, b, n, buf) != noErr) {
            ok = 0;
            break;
        }
        for (long i = 0; i < n; i++)
            if (memcmp(buf + i * BLOCK, backup + (b + i) * BLOCK, BLOCK) != 0)
                bad++;
    }
    out("   written back in %.1f s (%.1f KB/s): %s\n", tw / 1e6, kbps(d->blocks * BLOCK, tw),
        !ok ? "I/O ERROR, THE DISK MAY BE LOST" : bad ? "SOME BLOCKS DIFFER" : "every block as before");
}

/* ---- Hard disk tests ---- */

static void test_hd_sequential(const Disk *d, char *buf)
{
    static const struct { long chunk, count; const char *key; } runs[] = {
        {1, 256, "seq512"}, {8, 512, "seq4k"}, {64, 2048, "seq32k"},
    };
    out("H1. Sequential raw reads from block 0\n");
    for (unsigned i = 0; i < sizeof runs / sizeof runs[0]; i++) {
        long count = runs[i].count < d->blocks ? runs[i].count : d->blocks;
        uint32_t total = timed_read(d, 0, count, runs[i].chunk, buf, NULL, NULL);
        if (!total)
            return;
        long requests = (count + runs[i].chunk - 1) / runs[i].chunk;
        out("   %5ld B requests: %ld KB in %.2f s = %.1f KB/s, %.1f ms per request\n", runs[i].chunk * BLOCK,
            count / 2, total / 1e6, kbps(count * BLOCK, total), total / 1e3 / requests);
        sum(runs[i].key, "%.1f", kbps(count * BLOCK, total));
    }
}

static void test_hd_stroke(const Disk *d, char *buf)
{
    uint32_t t = alternate(d, 0, d->blocks - 1, 16, buf);
    out("H3. Full stroke (block 0 <-> %ld): %.1f ms per read\n", d->blocks - 1, t / 1e3);
    sum("stroke", "%.1f", t / 1e3);
}

/* ---- Main ---- */

static void header(const Disk *d, const char *label)
{
    unsigned long secs;
    Str255 date, time;
    char ds[64], ts[64];
    long machine = 0, sysv = 0;
    GetDateTime(&secs);
    IUDateString(secs, shortDate, date);
    IUTimeString(secs, false, time);
    cstr(ds, date);
    cstr(ts, time);
    Gestalt(gestaltMachineType, &machine);
    Gestalt(gestaltSystemVersion, &sysv);
    out("\n==== %s ====\n", label[0] ? label : "(no label)");
    out("FloppyBench %s, %s %s, machine type %ld, System %lx.%lx.%lx, timer %s\n", VERSION, ds, ts, machine,
        (sysv >> 8) & 0xF, (sysv >> 4) & 0xF, sysv & 0xF, have_microseconds ? "Microseconds" : "ticks (1/60 s)");
    out("Drive %d (%s, driver %d): %s, %ld blocks, %s, volume \"%s\"\n", d->drive, d->kind, d->refnum, d->format,
        d->blocks, d->locked ? "locked" : "writable", d->vref ? d->volume : "(none)");
    snprintf(summary, sizeof summary, "%s", label[0] ? label : "(no label)");
    sum("date", "%s %s", ds, ts);
    sum("machine", "%ld", machine);
    sum("system", "%lx.%lx.%lx", (sysv >> 8) & 0xF, (sysv >> 4) & 0xF, sysv & 0xF);
    sum("drive", "%s", d->kind);
    sum("format", "%s", d->format);
    sum("blocks", "%ld", d->blocks);
}

static void run(Disk *d, long mount_us, char *buf)
{
    show("Label for this run (e.g. \"real drive 800K\", \"rp2350 MOOF\"): ");
    char label[80];
    read_line(label, sizeof label);
    int writes = !d->locked &&
                 ask_yes(d->hard ? "Run write tests? They write a 512 KB stretch of the disk with other data and then\n"
                                   "put it back, and write a temporary file"
                                 : "Run write tests? They rewrite blocks with other data and then put them back, and\n"
                                   "write a temporary file");
    char *backup = NULL;
    int format = 0;
    if (writes && !d->hard &&
        ask_yes("Also format and verify the disk? Its contents are copied to memory first and written back")) {
        backup = NewPtr(d->blocks * BLOCK);
        if (backup)
            format = 1;
        else
            show("Not enough memory for a %ld KB copy of the disk: format test skipped.\n", d->blocks / 2);
    }

    open_results();
    header(d, label);
    if (mount_us > 0) {
        out("0. Insert to mounted: %.2f s\n", mount_us / 1e6);
        sum("mount_s", "%.2f", mount_us / 1e6);
    }
    if (d->hard) {
        test_hd_sequential(d, buf);
        random_reads(d, "H2. Random single-block reads", buf);
        test_hd_stroke(d, buf);
        if (writes) {
            long count = 1024 < d->blocks / 2 ? 1024 : d->blocks / 2;
            changed_writes(d, "H4. Changed-data writes", d->blocks / 2 - count / 2, count, BUF_BLOCKS, buf);
            test_file(d, "H5. File test", HD_FILE_TEST_BYTES, buf);
        } else {
            out("H4-H5. Write tests skipped%s\n", d->locked ? " (disk locked)" : "");
        }
    } else {
        test_sequential(d, buf);
        test_blockwise(d, buf);
        test_seeks(d, buf);
        test_sides(d, buf);
        if (writes) {
            test_rewrite(d, buf);
            test_file(d, "6. File test", FILE_TEST_BYTES, buf);
        } else {
            out("5-6. Write tests skipped%s\n", d->locked ? " (disk locked)" : "");
        }
        test_random(d, buf);
        test_cylinders(d, buf);
        if (writes)
            test_changed(d, buf);
        else
            out("9. Changed-data writes skipped\n");
        if (format)
            test_format(d, backup, buf);
        else
            out("10. Format test skipped\n");
    }
    close_results();
    write_summary();
    if (backup)
        DisposePtr(backup);
    show("Done; results appended to \"FloppyBench Results\" and \"FloppyBench Summary\".\n\n");
}

int main(void)
{
    char line[128];
    Disk disks[8];
    int count = 0;
    con_init();
    char *buf = NewPtr(BUF_BLOCKS * BLOCK);
    if (!buf) {
        show("Not enough memory. Press Return to quit.\n");
        read_line(line, sizeof line);
        return 1;
    }
    init_timing();
    show("FloppyBench %s: absolute floppy and HD20 timings for comparing drives and emulators.\n", VERSION);
    show("Timer: %s. Cmd-Q quits.\n\n", have_microseconds ? "Microseconds" : "ticks (1/60 s)");

    for (;;) {
        count = scan_drives(disks, 8);
        show("Drives:\n");
        for (int i = 0; i < count; i++)
            show("  %d: drive %d (%s), %s%s%s\n", i + 1, disks[i].drive, disks[i].kind,
                 disks[i].disk_in ? "disk in" : "empty", disks[i].vref ? ", volume " : "",
                 disks[i].vref ? disks[i].volume : "");
        show("Test which (number), e and a number to eject (just e if one disk is in),\n"
             "r to rescan, q to quit? ");
        read_line(line, sizeof line);
        if (line[0] == 'q' || line[0] == 'Q')
            break;
        if (line[0] == 'e' || line[0] == 'E') {
            int pick = atoi(line + 1);
            if (!line[1]) { /* plain "e": the only floppy with a disk in */
                for (int i = 0; i < count; i++)
                    if (disks[i].disk_in && disks[i].refnum == SONY_REFNUM && !readable(&disks[i], 2880, buf))
                        pick = pick ? -1 : i + 1;
            }
            if (pick < 1 || pick > count || !disks[pick - 1].disk_in) {
                show("Which drive? Type e and its number from the list.\n\n");
                continue;
            }
            Disk old = disks[pick - 1];
            if (!eject_disk(&old))
                continue;
            long mount = wait_for_disk(&old);
            if (mount < 0) {
                show("\n");
                continue;
            }
            /* Straight on to the new disk in that drive. */
            count = scan_drives(disks, 8);
            for (int i = 0; i < count; i++)
                if (disks[i].drive == old.drive && disks[i].refnum == old.refnum) {
                    probe(&disks[i], buf);
                    if (disks[i].blocks) {
                        show("Drive %d: %s, volume \"%s\"\n", disks[i].drive, disks[i].format, disks[i].volume);
                        run(&disks[i], mount, buf);
                    } else {
                        show("Can't read this disk.\n\n");
                    }
                }
            continue;
        }
        int pick = atoi(line);
        if (pick < 1 || pick > count) {
            show("\n");
            continue;
        }
        Disk *d = &disks[pick - 1];
        if (!d->disk_in) {
            show("No disk in drive %d.\n\n", d->drive);
            continue;
        }
        probe(d, buf);
        if (!d->blocks) {
            show("Can't read this disk.\n\n");
            continue;
        }
        show("Drive %d: %s\n", d->drive, d->format);
        run(d, 0, buf);
    }
    DisposePtr(buf);
    return 0;
}
