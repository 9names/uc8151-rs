//! This is a Rust port of the [Pimoroni UC8151 library](https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/uc8151).  
//! UC8151 is also sometimes referred to as IL0373.
//!
//! The current implementation only supports the particular variant used on the Pimoroni Badger2040, which uses a black and white (1bit per pixel) 128x296 pixel screen.
//! Rust [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) support is enabled with the `graphics` feature, which is enabled by default.
#![no_std]

use embedded_graphics_core::primitives::Rectangle;

// #![cfg_attr(not(test), no_std)]

#[allow(warnings)]
mod constants;
mod greyscale;
mod lut;
use constants::*;
use core::ops::Index;
use core::ops::Range;

pub mod asynch;
pub mod blocking;

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
