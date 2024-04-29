use crate::constants::*;
use crate::SpiDataError;
use crate::FRAME_BUFFER_SIZE;
use crate::HEIGHT;
use crate::LUT;
use crate::WIDTH;
use core::ops::Range;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;

#[cfg(feature = "graphics")]
use embedded_graphics_core::{
    draw_target::DrawTarget,
    geometry::Size,
    geometry::{Dimensions, OriginDimensions},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::Rectangle,
};

/// Uc8151 driver
pub struct Uc8151<SPI, DC, BUSY, RESET, DELAY> {
    framebuffer: [u8; FRAME_BUFFER_SIZE as usize],
    pub spi: SPI,
    pub dc: DC,
    pub busy: BUSY,
    pub reset_pin: RESET,
    pub delay: DELAY,
    pub lut: LUT,
}

impl<SPI, DC, BUSY, RESET, DELAY> Uc8151<SPI, DC, BUSY, RESET, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    BUSY: InputPin,
    RESET: OutputPin,
    DELAY: DelayNs,
{
    /// Create new UC8151 instance from the given SPI and GPIO pins
    pub fn new(spi: SPI, dc: DC, busy: BUSY, reset_pin: RESET, delay: DELAY) -> Self {
        Self {
            framebuffer: [0; FRAME_BUFFER_SIZE as usize],
            spi,
            dc,
            busy,
            reset_pin,
            delay,
            lut: LUT::Internal,
        }
    }

    /// Enable the display controller
    pub fn enable(&mut self) {
        // Ignoring return value for set, RP2040 GPIO is infallible
        let _ = self.reset_pin.set_high();
    }

    /// Disable the display controller
    pub fn disable(&mut self) {
        // Ignoring return value for set, RP2040 GPIO is infallible
        let _ = self.reset_pin.set_low();
    }

    /// Returns true if the display controller is busy
    pub fn is_busy(&mut self) -> bool {
        self.busy.is_low().unwrap_or(true)
    }

    /// Wait until the display controller is not busy before continuing
    pub fn wait_while_busy(&mut self) {
        while self.is_busy() {}
    }

    /// Return the currently selected LUT
    pub fn get_lut(&self) -> &'static Lut {
        match self.lut {
            LUT::Internal => &DEFAULT_LUT,
            LUT::Normal => &DEFAULT_LUT,
            LUT::Medium => &MEDIUM_LUT,
            LUT::Fast => &FAST_LUT,
            LUT::Ultrafast => &ULTRA_LUT,
        }
    }

    /// Return the currently update speed
    pub fn get_update_speed(&self) -> &Lut {
        self.get_lut()
    }

    /// Return the update interval for the current LUT
    pub fn get_update_time(&self) -> u16 {
        self.get_lut().update_time
    }

    fn update_speed(&mut self) -> Result<(), SpiDataError> {
        let lut = self.get_lut();
        self.command(Instruction::LUT_VCOM, &lut.vcom)?;
        self.command(Instruction::LUT_WW, &lut.ww)?;
        self.command(Instruction::LUT_BW, &lut.bw)?;
        self.command(Instruction::LUT_WB, &lut.wb)?;
        self.command(Instruction::LUT_BB, &lut.bb)?;

        self.wait_while_busy();
        Ok(())
    }

    /// Reset the display
    pub fn reset(&mut self) {
        self.disable();
        self.delay.delay_us(10_000);
        self.enable();
        self.delay.delay_us(10_000);

        // Wait until the screen is finished initialising before returning
        self.wait_while_busy();
    }

    /// Send command via SPI to the display
    pub fn command(&mut self, reg: Instruction, data: &[u8]) -> Result<(), SpiDataError> {
        let _ = self.dc.set_low(); // command mode
        self.spi
            .write(&[reg as u8])
            .map_err(|_| SpiDataError::SpiError)?;

        if !data.is_empty() {
            let _ = self.dc.set_high(); // data mode
            self.spi.write(data).map_err(|_| SpiDataError::SpiError)?;
        }
        Ok(())
    }

    /// Send data via SPI to the display
    pub fn data(&mut self, data: &[u8]) -> Result<(), SpiDataError> {
        let _ = self.dc.set_high(); // data mode
        self.spi.write(data).map_err(|_| SpiDataError::SpiError)?;
        Ok(())
    }

    /// Send framebuffer to display via SPI.
    /// This is a low-level function, call update() if you just want to update the display
    pub fn transmit_framebuffer(&mut self) -> Result<(), SpiDataError> {
        let _ = self.dc.set_low(); // command mode
        self.spi
            .write(&[Instruction::DTM2 as u8])
            .map_err(|_| SpiDataError::SpiError)?;

        let _ = self.dc.set_high(); // data mode
        self.spi
            .write(&self.framebuffer)
            .map_err(|_| SpiDataError::SpiError)?;
        Ok(())
    }

    /// Transmits a subset of the framebuffer via SPI.
    /// Call partial_update if you are looking to update a partial area of the display.
    pub fn transmit_framebuffer_range(&mut self, range: Range<usize>) -> Result<(), SpiDataError> {
        let framebuffer = match self.framebuffer.get(range) {
            Some(slice) => slice,
            None => return Err(SpiDataError::SpiError),
        };

        let _ = self.dc.set_high(); // data mode
        self.spi
            .write(framebuffer)
            .map_err(|_| SpiDataError::SpiError)?;
        Ok(())
    }

    /// Configure the display
    pub fn setup(&mut self, speed: LUT) -> Result<(), SpiDataError> {
        self.reset();
        self.lut = speed;

        let lut_type = if speed == LUT::Internal {
            psrflags::LUT_OTP
        } else {
            psrflags::LUT_REG
        };
        let init_cmd = lut_type
            | psrflags::RES_128X296
            | psrflags::FORMAT_BW
            | psrflags::SHIFT_RIGHT
            | psrflags::BOOSTER_ON
            | psrflags::RESET_NONE;

        self.command(Instruction::PSR, &[init_cmd])?;

        // No need to load a LUT if using internal one-time-programmable memory
        if self.lut != LUT::Internal {
            self.update_speed()?;
        }

        self.command(
            Instruction::PWR,
            &[
                pwr_flags_1::VDS_INTERNAL | pwr_flags_1::VDG_INTERNAL,
                pwr_flags_2::VCOM_VD | pwr_flags_2::VGHL_16V,
                0b101011,
                0b101011,
                0b101011,
            ],
        )?;
        self.command(Instruction::PON, &[])?;
        self.wait_while_busy();

        self.command(
            Instruction::BTST,
            &[
                booster_flags::START_10MS | booster_flags::STRENGTH_3 | booster_flags::OFF_6_58US,
                booster_flags::START_10MS | booster_flags::STRENGTH_3 | booster_flags::OFF_6_58US,
                booster_flags::START_10MS | booster_flags::STRENGTH_3 | booster_flags::OFF_6_58US,
            ],
        )?;
        self.command(Instruction::PFS, &[pfs_flags::FRAMES_1])?;
        self.command(
            Instruction::TSE,
            &[tse_flags::TEMP_INTERNAL | tse_flags::OFFSET_0],
        )?;
        self.command(Instruction::TCON, &[0x22])?;

        self.invert_colors(false)?;

        self.command(Instruction::PLL, &[self.get_lut().pll])?;

        self.command(Instruction::POF, &[])?;

        self.wait_while_busy();
        Ok(())
    }

    #[allow(clippy::unusual_byte_groupings)]
    /// Invert the colors of the screen - normal = black on white, inverted = white on black
    pub fn invert_colors(&mut self, inverted: bool) -> Result<(), SpiDataError> {
        self.command(
            Instruction::CDI,
            &[if inverted { 0b01_01_1100 } else { 0b01_00_1100 }],
        )
    }
    // There was a poweroff function that's just off without blocking
    // Won't add it until non-blocking mode added

    /// Ask the display to power itself off
    pub fn off(&mut self) -> Result<(), SpiDataError> {
        self.wait_while_busy();
        self.command(Instruction::POF, &[])?;
        Ok(())
    }

    /// Set or clear the specified pixel
    pub fn pixel(&mut self, x: u32, y: u32, v: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }

        let address = ((y / 8) + (x * (HEIGHT / 8))) as usize;

        let o: u8 = 7 - (y as u8 & 0b111); // bit offset within byte
        let m: u8 = !(1 << o); // bit mask for byte
        let b: u8 = (v as u8) << o; // bit value shifted to position

        self.framebuffer[address] = (self.framebuffer[address] & m) | b;
    }

    /// Refresh the display with what is in the framebuffer
    pub fn update(&mut self) -> Result<(), SpiDataError> {
        self.wait_while_busy();

        self.command(Instruction::PON, &[])?; // turn on
        self.command(Instruction::PTOU, &[])?; // disable partial mode

        self.transmit_framebuffer()?;

        self.command(Instruction::DSP, &[])?; // data stop

        self.command(Instruction::DRF, &[])?; // start display refresh

        self.wait_while_busy();

        self.command(Instruction::POF, &[])?; // turn off
        Ok(())
    }

    /// Peform a partial refresh of the display over the area defined by the `DisplayRegion`.
    pub fn partial_update(&mut self, region: UpdateRegion) -> Result<(), SpiDataError> {
        #![allow(clippy::cast_possible_truncation)]
        self.wait_while_busy();

        let height = region.height;
        let width = region.width;
        let columns = (height / 8) as usize;
        let y = region.y;
        let y1 = y / 8;
        let x = region.x;

        self.command(Instruction::PON, &[])?;
        self.command(Instruction::PTIN, &[])?;
        self.command(
            Instruction::PTL,
            &[
                y as u8,
                (y + height - 1) as u8,
                (x >> 8) as u8,
                (x & 0xff) as u8,
                ((x + width - 1) >> 8) as u8,
                ((x + width - 1) & 0xff) as u8,
                1,
            ],
        )?;

        self.command(Instruction::DTM2, &[])?;

        for dx in 0..width {
            let sx = dx + x;
            let sy = y1;
            let idx = (sy + (sx * (HEIGHT / 8))) as usize;
            self.transmit_framebuffer_range(idx..(idx + columns))?;
        }

        self.command(Instruction::DSP, &[])?;

        self.command(Instruction::DRF, &[])?;

        self.wait_while_busy();

        self.command(Instruction::POF, &[])?; // turn off

        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<SPI, DC, BUSY, RESET, DELAY> DrawTarget for Uc8151<SPI, DC, BUSY, RESET, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    BUSY: InputPin,
    RESET: OutputPin,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let bb = self.bounding_box();

        pixels
            .into_iter()
            .filter(|Pixel(pos, _color)| bb.contains(*pos))
            .for_each(|Pixel(pos, color)| {
                self.pixel(pos.x as u32, pos.y as u32, color == BinaryColor::Off)
            });

        Ok(())
    }
}

#[cfg(feature = "graphics")]
impl<SPI, DC, BUSY, RESET, DELAY> OriginDimensions for Uc8151<SPI, DC, BUSY, RESET, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    BUSY: InputPin,
    RESET: OutputPin,
    DELAY: DelayNs,
{
    fn size(&self) -> Size {
        Size::new(WIDTH, HEIGHT)
    }
}

/// Represents a rectangular display region to be updated.
#[derive(Copy, Clone)]
pub struct UpdateRegion {
    x: u32,
    y: u32,
    width: u32,
    height: u32,
}

impl UpdateRegion {
    /// Create a new `UpdateRegion`. The provided `y` and `height` values must be a multiple of
    /// eight.
    ///
    /// # Errors
    /// Returns an error if the provided y coordinate or height is not a multiple of eight.
    ///
    pub fn new(x: u32, y: u32, width: u32, height: u32) -> Result<Self, &'static str> {
        const LOWER: u32 = 0b111;
        if (y & LOWER) != 0 || (height & LOWER) != 0 {
            return Err("The provide y coordinate and height must be a multiple of eight.");
        }
        Ok(Self {
            x,
            y,
            width,
            height,
        })
    }
}

#[cfg(feature = "graphics")]
impl TryFrom<Rectangle> for UpdateRegion {
    type Error = &'static str;

    fn try_from(value: Rectangle) -> Result<Self, &'static str> {
        #![allow(clippy::cast_sign_loss)]
        let point = value.top_left;
        let size = value.size;
        if point.x < 0 || point.y < 0 {
            return Err("Point must have positive coordinates");
        }
        UpdateRegion::new(point.x as u32, point.y as u32, size.width, size.height)
    }
}
