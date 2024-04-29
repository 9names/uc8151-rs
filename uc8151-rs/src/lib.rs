//! This is a Rust port of the [Pimoroni UC8151 library](https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/uc8151).  
//! UC8151 is also sometimes referred to as IL0373.
//!
//! The current implementation only supports the particular variant used on the Pimoroni Badger2040, which uses a black and white (1bit per pixel) 128x296 pixel screen.
//! Rust [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) support is enabled with the `graphics` feature, which is enabled by default.
#![no_std]

pub mod asynch;
pub mod blocking;
pub mod constants;

// RES_128X296 1bit per pixel
/// Width of the screen in pixels
pub const WIDTH: u32 = 296;
/// Height of the screen in pixels
pub const HEIGHT: u32 = 128;

const FRAME_BUFFER_SIZE: u32 = (WIDTH * HEIGHT) / 8;

/// Screen refresh-speed configurations for the display
///
/// Preset configuration options for setting the refresh speed for the display.
/// Faster refreshes will leave more of the previous image.
#[derive(Clone, Copy, PartialEq, Eq)]
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

/// An error that with the SPI data bus
#[derive(Debug)]
pub enum SpiDataError {
    SpiError,
}