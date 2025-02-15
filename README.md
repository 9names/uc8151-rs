# uc8151-rs - a no-std Rust library for the UC8151(IL0373) e-ink display
This is a Rust port of the [Pimoroni UC8151 library](https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/uc8151).  
UC8151 is also sometimes referred to as  IL0373.

The current implementation only supports the particular variant used on Pimoroni Badger 2040 and Pimoroni Pico Inky Pack.  
These use a black and white (1bit per pixel) 128x296 pixel e-ink screen.
Rust [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) support is enabled with the `graphics` default feature - you will want to use this!

The examples are written for Badger 2040, but have been tested with Pico Inky Pack.
In order to use the examples with Pico Inky Pack you need to change which pins are assigned to the e-ink signals.
For example, with rp2040-hal:

```rust
let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
let mut dc = pins.gpio20.into_push_pull_output();
let mut cs = pins.gpio17.into_push_pull_output();
let busy = pins.gpio26.into_pull_up_input(); 
let reset = pins.gpio21.into_push_pull_output(); 
```
