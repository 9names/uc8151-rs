#![no_std]

#[allow(warnings)]
mod constants;
use constants::*;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::Write;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

#[derive(Clone, Copy, PartialEq)]
pub enum LUT {
    /// DEFAULT_LUT (OTP memory)
    Internal,
    /// DEFAULT_LUT
    Normal,
    /// MEDIUM_LUT
    Medium,
    /// FAST_LUT
    Fast,
    /// ULTRA_LUT
    Ultrafast,
}

pub struct Uc8151<SPI, CS, DC, BUSY, RESET> {
    pub spi: SPI,
    pub cs: CS,
    pub dc: DC,
    pub busy: BUSY,
    pub reset: RESET,
    pub lut: LUT,
}

#[derive(Debug)]
pub enum SpiDataError {
    SpiError,
}

// fakey framebuffer until i get E-G working
// RES_128X296
const WIDTH: u32 = 128;
const HEIGHT: u32 = 296;
const FRAME_BUFFER_SIZE: u32 = (WIDTH * HEIGHT) / 8;
static mut FRAME_BUFFER: [u8; FRAME_BUFFER_SIZE as usize] = [0; FRAME_BUFFER_SIZE as usize];

impl<SPI, CS, DC, BUSY, RESET> Uc8151<SPI, CS, DC, BUSY, RESET>
where
    SPI: Write<u8>,
    CS: OutputPin,
    DC: OutputPin,
    BUSY: InputPin,
    RESET: OutputPin,
{
    pub fn new(spi: SPI, cs: CS, dc: DC, busy: BUSY, reset: RESET) -> Self {
        Self {
            spi,
            cs,
            dc,
            busy,
            reset,
            lut: LUT::Internal,
        }
    }

    pub fn enable(&mut self) {
        // Ignoring return value for set, RP2040 GPIO is infallible
        let _ = self.reset.set_high();
    }

    pub fn disable(&mut self) {
        // Ignoring return value for set, RP2040 GPIO is infallible
        let _ = self.reset.set_low();
    }

    pub fn is_busy(&self) -> bool {
        self.busy.is_low().unwrap_or(true)
    }

    pub fn get_lut(&self) -> &'static Lut {
        match self.lut {
            LUT::Internal => &DEFAULT_LUT,
            LUT::Normal => &DEFAULT_LUT,
            LUT::Medium => &MEDIUM_LUT,
            LUT::Fast => &FAST_LUT,
            LUT::Ultrafast => &ULTRA_LUT,
        }
    }

    pub fn get_update_speed(&self) -> &Lut {
        self.get_lut()
    }

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

        while self.is_busy() {}
        Ok(())
    }

    pub fn reset(&mut self, delay_source: &mut impl DelayUs<u32>) {
        self.disable();
        delay_source.delay_us(10_000);
        self.enable();
        delay_source.delay_us(10_000);

        // Wait until the screen is finished initialising before returning
        while self.is_busy() {}
    }

    // To do read we need to convert the SPI pins back to GPIO and bit-bang
    // since the data comes back over the MOSI pin.
    // Awkward.
    // Might be easiest to bitbang the command as well?
    // Not going there yet.

    // pub fn read(&mut self, reg: Instruction, data: &mut[u8]) -> Result<(), SpiDataError> {
    //     let _ = self.cs.set_low();
    //     let _ = self.dc.set_low(); // command mode
    //     self.spi
    //         .write(&[reg as u8])
    //         .map_err(|_| SpiDataError::SpiError)?;

    //     if !data.is_empty() {
    //         let _ = self.dc.set_high(); // data mode
    //         self.spi.write(data).map_err(|_| SpiDataError::SpiError)?;
    //     }

    //     let _ = self.cs.set_high();
    //     Ok(())
    // }

    pub fn command(&mut self, reg: Instruction, data: &[u8]) -> Result<(), SpiDataError> {
        let _ = self.cs.set_low();
        let _ = self.dc.set_low(); // command mode
        self.spi
            .write(&[reg as u8])
            .map_err(|_| SpiDataError::SpiError)?;

        if !data.is_empty() {
            let _ = self.dc.set_high(); // data mode
            self.spi.write(data).map_err(|_| SpiDataError::SpiError)?;
        }

        let _ = self.cs.set_high();
        Ok(())
    }

    pub fn data(&mut self, data: &[u8]) -> Result<(), SpiDataError> {
        let _ = self.cs.set_low();
        let _ = self.dc.set_high(); // data mode
        self.spi.write(data).map_err(|_| SpiDataError::SpiError)?;

        let _ = self.cs.set_high();
        Ok(())
    }

    pub fn setup(
        &mut self,
        delay_source: &mut impl DelayUs<u32>,
        speed: LUT,
    ) -> Result<(), SpiDataError> {
        self.reset(delay_source);
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
        while self.is_busy() {}

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
        let inverted = false; // TODO: make this a setting
        self.command(
            Instruction::CDI,
            &[if inverted { 0b01_01_1100 } else { 0b01_00_1100 }],
        )?;

        self.command(Instruction::PLL, &[self.get_lut().pll])?;

        self.command(Instruction::POF, &[])?;

        while self.is_busy() {}
        Ok(())
    }

    // There was a poweroff function that's just off without blocking
    // Won't add it until non-blocking mode added

    pub fn off(&mut self) -> Result<(), SpiDataError> {
        while self.is_busy() {}
        self.command(Instruction::POF, &[])?;
        Ok(())
    }

    pub fn pixel(&mut self, x: u32, y: u32, v: bool) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }

        let address = ((y / 8) + (x * (HEIGHT / 8))) as usize;

        let o: u8 = 7 - (y as u8 & 0b111); // bit offset within byte
        let m: u8 = !(1 << o); // bit mask for byte
        let b: u8 = if v { 0 } else { 1 } << o; // bit value shifted to position

        let value = unsafe { FRAME_BUFFER[address] };
        unsafe {
            FRAME_BUFFER[address] = (value & m) | b;
        }
    }

    pub fn update(&mut self) -> Result<(), SpiDataError> {
        // if blocking
        while self.is_busy() {}

        self.command(Instruction::PON, &[])?; // turn on
        self.command(Instruction::PTOU, &[])?; // disable partial mode

        unsafe {
            self.command(Instruction::DTM2, &FRAME_BUFFER)?; // transmit framebuffer
        }
        self.command(Instruction::DSP, &[])?; // data stop

        self.command(Instruction::DRF, &[])?; // start display refresh
                                              // if blocking
        while self.is_busy() {}

        self.command(Instruction::POF, &[])?; // turn off
        Ok(())
    }
}
