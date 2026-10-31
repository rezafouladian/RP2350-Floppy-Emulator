//! SSD1322 OLED driver (256x64, 4-bit grayscale) over SPI.
//!
//! Port of `ssd1322.py` from micropython-ssd1322. Drawing happens into an in-memory
//! GS4 framebuffer (two pixels per byte, high nibble first); call [`Display::present`]
//! to push it to the panel.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

pub const WIDTH: usize = 256;
pub const HEIGHT: usize = 64;
const BYTE_WIDTH: usize = WIDTH / 2;
const BUF_LEN: usize = BYTE_WIDTH * HEIGHT;

/// Horizontal column offset: pixels start at segment 112 (28 * 4).
const COLUMN_OFFSET: u8 = 28;

mod cmd {
    pub const SET_COLUMN_ADDRESS: u8 = 0x15;
    pub const WRITE_RAM: u8 = 0x5C;
    pub const SET_ROW_ADDRESS: u8 = 0x75;
    pub const SET_REMAP_DUAL_COM_LINE_MODE: u8 = 0xA0;
    pub const SET_DISPLAY_START_LINE: u8 = 0xA1;
    pub const SET_DISPLAY_OFFSET: u8 = 0xA2;
    pub const SET_DISPLAY_MODE_NORMAL: u8 = 0xA6;
    pub const PARTIAL_DISPLAY_DISABLE: u8 = 0xA9;
    pub const SET_FUNCTION_SELECTION: u8 = 0xAB;
    pub const DISPLAY_SLEEP_ON: u8 = 0xAE;
    pub const DISPLAY_SLEEP_OFF: u8 = 0xAF;
    pub const SET_PHASE_LENGTH: u8 = 0xB1;
    pub const SET_FRONT_CLOCK_DIVIDER: u8 = 0xB3;
    pub const DISPLAY_ENHANCEMENT_A: u8 = 0xB4;
    pub const SET_GPIO: u8 = 0xB5;
    pub const SET_SECOND_PRECHARGE_PERIOD: u8 = 0xB6;
    pub const SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE: u8 = 0xB9;
    pub const SET_PRECHARGE_VOLTAGE: u8 = 0xBB;
    pub const SET_VCOMH_VOLTAGE: u8 = 0xBE;
    pub const SET_CONTRAST_CURRENT: u8 = 0xC1;
    pub const MASTER_CURRENT_CONTROL: u8 = 0xC7;
    pub const SET_MULTIPLEX_RATIO: u8 = 0xCA;
    pub const DISPLAY_ENHANCEMENT_B: u8 = 0xD1;
    pub const SET_COMMAND_LOCK: u8 = 0xFD;
}

const ENABLE_EXTERNAL_VSL: u8 = 0x00;
const ENHANCED_LOW_GRAY_SCALE_QUALITY: u8 = 0xF8;
const RESERVED_ENHANCEMENT: u8 = 0x00;
const COMMANDS_UNLOCK: u8 = 0x12;

/// Image rotation for [`Display::draw_bitmap_mono`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Rotate {
    R0,
    R90,
    R180,
    R270,
}

#[derive(Debug)]
pub enum Error<S> {
    Spi(S),
    Pin,
}

pub struct Display<SPI, CS, DC, RST> {
    spi: SPI,
    cs: CS,
    dc: DC,
    rst: RST,
    buf: [u8; BUF_LEN],
}

impl<SPI, CS, DC, RST> Display<SPI, CS, DC, RST>
where
    SPI: SpiBus,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    /// Reset and initialise the panel, then show a blank screen.
    pub fn new(
        spi: SPI,
        cs: CS,
        dc: DC,
        rst: RST,
        delay: &mut impl DelayNs,
    ) -> Result<Self, Error<SPI::Error>> {
        let mut d = Self {
            spi,
            cs,
            dc,
            rst,
            buf: [0; BUF_LEN],
        };
        d.cs.set_high().map_err(|_| Error::Pin)?;
        d.dc.set_low().map_err(|_| Error::Pin)?;
        d.rst.set_high().map_err(|_| Error::Pin)?;
        d.reset(delay)?;

        d.write_cmd(cmd::SET_COMMAND_LOCK, &[COMMANDS_UNLOCK])?;
        d.write_cmd(cmd::DISPLAY_SLEEP_ON, &[])?;
        // 80 frames per second
        d.write_cmd(cmd::SET_FRONT_CLOCK_DIVIDER, &[0x91])?;
        // Multiplex ratio 1/64
        d.write_cmd(cmd::SET_MULTIPLEX_RATIO, &[0x3F])?;
        d.write_cmd(cmd::SET_DISPLAY_OFFSET, &[0x00])?;
        d.write_cmd(cmd::SET_DISPLAY_START_LINE, &[0x00])?;
        // Column 0 -> SEG0, scan COM[N-1]..COM0, dual COM line mode
        d.write_cmd(cmd::SET_REMAP_DUAL_COM_LINE_MODE, &[0x14, 0x11])?;
        // Disable GPIO pin input
        d.write_cmd(cmd::SET_GPIO, &[0x00])?;
        // Enable internal VDD regulator
        d.write_cmd(cmd::SET_FUNCTION_SELECTION, &[0x01])?;
        // External VSL
        d.write_cmd(
            cmd::DISPLAY_ENHANCEMENT_A,
            &[
                ENABLE_EXTERNAL_VSL | 0xA0,
                ENHANCED_LOW_GRAY_SCALE_QUALITY | 0x05,
            ],
        )?;
        d.write_cmd(cmd::SET_CONTRAST_CURRENT, &[0x9F])?;
        d.write_cmd(cmd::MASTER_CURRENT_CONTROL, &[0x0F])?;
        d.write_cmd(cmd::SELECT_DEFAULT_LINEAR_GRAY_SCALE_TABLE, &[])?;
        // Phase 1 = 5 clocks, phase 2 = 14 clocks
        d.write_cmd(cmd::SET_PHASE_LENGTH, &[0xE2])?;
        d.write_cmd(cmd::DISPLAY_ENHANCEMENT_B, &[RESERVED_ENHANCEMENT | 0xA2, 0x20])?;
        // Pre-charge voltage 0.60 * VCC
        d.write_cmd(cmd::SET_PRECHARGE_VOLTAGE, &[0x1F])?;
        // Second pre-charge period 8 clocks
        d.write_cmd(cmd::SET_SECOND_PRECHARGE_PERIOD, &[0x08])?;
        // VCOMH 0.86 * VCC
        d.write_cmd(cmd::SET_VCOMH_VOLTAGE, &[0x07])?;
        d.write_cmd(cmd::SET_DISPLAY_MODE_NORMAL, &[])?;
        d.write_cmd(cmd::PARTIAL_DISPLAY_DISABLE, &[])?;
        d.write_cmd(cmd::DISPLAY_SLEEP_OFF, &[])?;

        d.clear_buffer(0);
        d.present()?;
        Ok(d)
    }

    /// Hardware reset pulse.
    pub fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.rst.set_low().map_err(|_| Error::Pin)?;
        delay.delay_ms(50);
        self.rst.set_high().map_err(|_| Error::Pin)?;
        delay.delay_ms(100);
        Ok(())
    }

    /// Blank the panel and put it to sleep.
    pub fn cleanup(&mut self) -> Result<(), Error<SPI::Error>> {
        self.clear()?;
        self.sleep()
    }

    /// Clear the buffer and the panel.
    pub fn clear(&mut self) -> Result<(), Error<SPI::Error>> {
        self.clear_buffer(0);
        self.present()
    }

    /// Fill the framebuffer with a grayscale level (0 = black, 15 = white).
    pub fn clear_buffer(&mut self, gs: u8) {
        let g = gs & 0x0F;
        self.buf.fill(g << 4 | g);
    }

    /// Send the framebuffer to the panel.
    pub fn present(&mut self) -> Result<(), Error<SPI::Error>> {
        // 2 bytes per address, 2 pixels per byte
        self.set_address(0, 0, (WIDTH / 4 - 1) as u8, (HEIGHT - 1) as u8)?;
        transfer(&mut self.spi, &mut self.cs, &mut self.dc, true, &self.buf)
    }

    /// Send framebuffer rows `from..from + rows` to the panel: [`present`](Self::present) a piece
    /// at a time (8 rows, 1 KB, take under 1 ms at 10 MHz; the whole screen ~7 ms).
    pub fn present_rows(&mut self, from: usize, rows: usize) -> Result<(), Error<SPI::Error>> {
        let to = (from + rows).min(HEIGHT);
        if from >= to {
            return Ok(());
        }
        self.set_address(0, from as u8, (WIDTH / 4 - 1) as u8, (to - 1) as u8)?;
        transfer(&mut self.spi, &mut self.cs, &mut self.dc, true, &self.buf[from * BYTE_WIDTH..to * BYTE_WIDTH])
    }

    pub fn sleep(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_cmd(cmd::DISPLAY_SLEEP_ON, &[])
    }

    pub fn wake(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_cmd(cmd::DISPLAY_SLEEP_OFF, &[])
    }

    /// Set column/row window and start a RAM write. Column addresses get the
    /// 28-unit panel offset applied.
    fn set_address(&mut self, x0: u8, y0: u8, x1: u8, y1: u8) -> Result<(), Error<SPI::Error>> {
        self.write_cmd(
            cmd::SET_COLUMN_ADDRESS,
            &[x0 + COLUMN_OFFSET, x1 + COLUMN_OFFSET],
        )?;
        self.write_cmd(cmd::SET_ROW_ADDRESS, &[y0, y1])?;
        self.write_cmd(cmd::WRITE_RAM, &[])
    }

    fn write_cmd(&mut self, command: u8, args: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.write(false, &[command])?;
        if !args.is_empty() {
            self.write(true, args)?;
        }
        Ok(())
    }

    fn write(&mut self, data: bool, bytes: &[u8]) -> Result<(), Error<SPI::Error>> {
        transfer(&mut self.spi, &mut self.cs, &mut self.dc, data, bytes)
    }

    // ---- Drawing -------------------------------------------------------------------------

    /// Set a pixel; out-of-bounds coordinates are ignored.
    pub fn draw_pixel(&mut self, x: i32, y: i32, gs: u8) {
        if x < 0 || y < 0 || x >= WIDTH as i32 || y >= HEIGHT as i32 {
            return;
        }
        let (x, y) = (x as usize, y as usize);
        let b = &mut self.buf[y * BYTE_WIDTH + x / 2];
        let g = gs & 0x0F;
        *b = if x % 2 == 0 {
            (*b & 0x0F) | g << 4
        } else {
            (*b & 0xF0) | g
        };
    }

    /// Read a pixel; out-of-bounds returns 0.
    pub fn pixel(&self, x: i32, y: i32) -> u8 {
        if x < 0 || y < 0 || x >= WIDTH as i32 || y >= HEIGHT as i32 {
            return 0;
        }
        let (x, y) = (x as usize, y as usize);
        let b = self.buf[y * BYTE_WIDTH + x / 2];
        if x % 2 == 0 { b >> 4 } else { b & 0x0F }
    }

    pub fn draw_hline(&mut self, x: i32, y: i32, w: i32, gs: u8) {
        for i in 0..w {
            self.draw_pixel(x + i, y, gs);
        }
    }

    pub fn draw_vline(&mut self, x: i32, y: i32, h: i32, gs: u8) {
        for i in 0..h {
            self.draw_pixel(x, y + i, gs);
        }
    }

    /// Bresenham line.
    pub fn draw_line(&mut self, x1: i32, y1: i32, x2: i32, y2: i32, gs: u8) {
        let (mut x, mut y) = (x1, y1);
        let dx = (x2 - x1).abs();
        let dy = -(y2 - y1).abs();
        let sx = if x1 < x2 { 1 } else { -1 };
        let sy = if y1 < y2 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.draw_pixel(x, y, gs);
            if x == x2 && y == y2 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    pub fn draw_lines(&mut self, coords: &[(i32, i32)], gs: u8) {
        for w in coords.windows(2) {
            self.draw_line(w[0].0, w[0].1, w[1].0, w[1].1, gs);
        }
    }

    pub fn draw_rectangle(&mut self, x: i32, y: i32, w: i32, h: i32, gs: u8) {
        self.draw_hline(x, y, w, gs);
        self.draw_hline(x, y + h - 1, w, gs);
        self.draw_vline(x, y, h, gs);
        self.draw_vline(x + w - 1, y, h, gs);
    }

    pub fn fill_rectangle(&mut self, x: i32, y: i32, w: i32, h: i32, gs: u8) {
        for j in 0..h {
            self.draw_hline(x, y + j, w, gs);
        }
    }

    /// Draw a MONO_HMSB bitmap (1 bit per pixel, rows padded to whole bytes).
    ///
    /// Despite the name, MicroPython's MONO_HMSB stores the leftmost pixel in the *least*
    /// significant bit of each byte.
    ///
    /// Set pixels are drawn at `gs` and unset pixels black, or the reverse if `invert`.
    /// `x`, `y` is the top-left of the drawn (rotated) image.
    #[allow(clippy::too_many_arguments)]
    pub fn draw_bitmap_mono(
        &mut self,
        data: &[u8],
        x: i32,
        y: i32,
        w: usize,
        h: usize,
        invert: bool,
        gs: u8,
        rotate: Rotate,
    ) {
        let stride = w.div_ceil(8);
        let src = |sx: usize, sy: usize| -> bool {
            data.get(sy * stride + sx / 8)
                .is_some_and(|b| b & (1 << (sx % 8)) != 0)
        };
        let (dw, dh) = match rotate {
            Rotate::R0 | Rotate::R180 => (w, h),
            Rotate::R90 | Rotate::R270 => (h, w),
        };
        for dy in 0..dh {
            for dx in 0..dw {
                let set = match rotate {
                    Rotate::R0 => src(dx, dy),
                    Rotate::R90 => src(dy, h - 1 - dx),
                    Rotate::R180 => src(w - 1 - dx, h - 1 - dy),
                    Rotate::R270 => src(w - 1 - dy, dx),
                };
                let g = if set != invert { gs } else { 0 };
                self.draw_pixel(x + dx as i32, y + dy as i32, g);
            }
        }
    }

    /// Draw a GS4_HMSB bitmap (two pixels per byte, high nibble first).
    pub fn draw_bitmap_gs4(
        &mut self,
        data: &[u8],
        x: i32,
        y: i32,
        w: usize,
        h: usize,
        invert: bool,
    ) {
        let stride = w.div_ceil(2);
        for j in 0..h {
            for i in 0..w {
                let Some(&b) = data.get(j * stride + i / 2) else {
                    return;
                };
                let g = if i % 2 == 0 { b >> 4 } else { b & 0x0F };
                self.draw_pixel(x + i as i32, y + j as i32, if invert { 15 - g } else { g });
            }
        }
    }
}

/// One chip-select-framed SPI transfer.
fn transfer<SPI: SpiBus, CS: OutputPin, DC: OutputPin>(
    spi: &mut SPI,
    cs: &mut CS,
    dc: &mut DC,
    data: bool,
    bytes: &[u8],
) -> Result<(), Error<SPI::Error>> {
    if data {
        dc.set_high()
    } else {
        dc.set_low()
    }
    .map_err(|_| Error::Pin)?;
    cs.set_low().map_err(|_| Error::Pin)?;
    let r = spi.write(bytes).and_then(|_| spi.flush());
    cs.set_high().map_err(|_| Error::Pin)?;
    r.map_err(Error::Spi)
}

impl<SPI, CS, DC, RST> embedded_graphics_core::geometry::OriginDimensions
    for Display<SPI, CS, DC, RST>
{
    fn size(&self) -> embedded_graphics_core::geometry::Size {
        embedded_graphics_core::geometry::Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

/// `embedded-graphics` support: draw into the framebuffer, then call [`Display::present`].
impl<SPI, CS, DC, RST> embedded_graphics_core::draw_target::DrawTarget for Display<SPI, CS, DC, RST>
where
    SPI: SpiBus,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    type Color = embedded_graphics_core::pixelcolor::Gray4;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics_core::Pixel<Self::Color>>,
    {
        use embedded_graphics_core::prelude::*;
        for Pixel(p, c) in pixels {
            self.draw_pixel(p.x, p.y, c.luma());
        }
        Ok(())
    }
}
