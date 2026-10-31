# RP2350 Floppy Emulator

An RP2350-based floppy emulator for Macintosh and Apple II systems.

## Introduction

One day a while back I was unable to find where I put my floppy emu.  
Out of curiosity I wanted to see what Claude would do if I gave it a whole stack of PDFs documenting the floppy interface and have it make an emulator with what I had lying around. For fun I also told it to do it in Rust, since I remember Rust on the RP2350 being limited last time I used it and I wanted to see if it was possible.  
To my surprise it generated working code on the first try. It took me longer to wire up the adapter than generate the code.

I was curious how much further it could go, so I prompted Claude to add more and more functionality to it.

## Components

The emulator is built from what I had lying around:
- Waveshare RP2350-PiZero Development Board
- 3.12" OLED Ver: 2.1 (SSD1322)
- KY-040 Rotary Encoder
- microSD Card
- Pin headers, ribbon cable
- DB-19 to IDC adapter (optional)

## Supported Formats

### Macintosh

All formats support both read and write (except MOOF flux).

Modes:
- floppy: standard Macintosh floppy drive
- hd20: Hard Disk 20
- hd20+floppy: Hard Disk 20 and a floppy drive at the same time

Formats:
- 400K and 800K GCR
- 720K and 1.44MB MFM
- HD20 mode, up to 8GB

File Types:
- Raw image (.img, .dsk, .image)
- DiskCopy 4.2 (.dc42, .diskcopy, .dsk, .img, .image)
- MOOF
- Hard disk images (.hda, .hfs, .img, .dsk, .image)

### Apple II

Modes:
- apple2: Disk II 5.25"
- apple35: Apple 3.5" drive
- apple2dual: Apple 3.5" drive and Disk II 5.25" at the same time
- smartport: up to 4 3.5" floppy drives or hard drives

5.25" Disks:
- DOS 3.3 order (.dsk, .do, read and write)
- ProDOS order (.po, .dsk, read and write)
- 2MG (.2mg, read and write)
- NIB (.nib, read-only)
- WOZ 1 (.woz, read-only)
- WOZ 2 (.woz, read and write)
- A2R 1, 2, 3 (.a2r, read-only)

3.5" Disks (all read and write):
- 400K/800K block image (.po, .dsk. .img, .image, .hdv)
- DiskCopy 4.2 (.dc42, .diskcopy, .image, .img, .dsk)
- 2MG
- MOOF
- WOZ 2

SmartPort Mode:
- Block image (.po, .hdv, .dsk, .img, .image, .hda)
- Hard drive images
- 2MG (ProDOS or DOS order 400K/800K)

## Wiring

> [!WARNING]
> Do not connect anything to GP27 (pin 13) on the Waveshare board. It is used elsewhere.

### Floppy Connector

| Signal | RP2350 GPIO | Header pin | DB-19 pin | Internal 20-pin pin |
|---|---|---|---|---|
| SEL | GP2 | 3 | 16 | 12 |
| CA0 / PH0 | GP3 | 5 | 11 | 2 |
| CA1 / PH1 | GP4 | 7 | 12 | 4 |
| CA2 / PH2 | GP5 | 29 | 13 | 6 |
| LSTRB / PH3 | GP8 | 24 | 14 | 8 |
| /ENBL | GP9 | 21 | 17 | 14 |
| /WRREQ | GP10 | 19 | 15 | 10 |
| WR | GP11 | 23 | 19 | 18 |
| RD | GP12 | 32 | 18 | 16 |
| WRPROT (Apple II modes only) | GP13 | 33 | 10 | 20 |
| /3.5DISK (Apple II 3.5" and dual modes) | GP14 | 8 | 4 | 7 |
| /DRIVE2 (Apple II 3.5" and dual modes, optional, IIgs only) | GP15 | 10 | 9 | — |
| +5V | 5V | | 6 | 11 |
| GND | GND | | 1, 2, 3 | 1, 3, 5 |
| +12V | **N/C** | **N/C** | 7, 8 | 13, 15, 17, 19 |
| -12V | **N/C** | **N/C** | 5 | 9 |

> [!CAUTION]
> Take care not to connect +12V or -12V to anything!


### SSD1322 OLED (256x64)

Make sure the board is in 4SPI mode. This was R5 and R8 on my board.

| OLED signal | RP2350 GPIO | Header pin | OLED pin |
|---|---|---|---|
| CS | GP17 | 11 | 16 |
| SCK | GP18 | 12 | 4 |
| MOSI (SDA/DIN) | GP19 | 35 | 5 |
| DC | GP20 | 38 | 14 |
| RST | GP21 | 40 | 15 |
| VCC | 3V3 | | 2 |
| GND | GND | | 1 |

### KY-040 Rotary Encoder

| KY-040 pin | RP2350 GPIO | Header pin |
|---|---|---|
| CLK | GP22 | 15 |
| DT | GP23 | 16 |
| SW | GP24 | 18 |
| + (VCC) | 3V3 |  |
| GND | GND |  |

### SD (built in to Waveshare board)

| SD signal | RP2350 GPIO |
|---|---|
| SCK | GP30 |
| MOSI (CMD) | GP31 |
| MISO (DAT0) | GP40 |
| CS (DAT3) | GP43 |
