//! Print numbers down the screen using partial refresh functionality
//! This example is based on https://github.com/skmcgrail/badger2040-partial-refresh
#![no_std]
#![no_main]

use bsp::entry;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use embedded_graphics::text::Text;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use panic_halt as _;

use bsp::hal;
use fugit::RateExtU32;
use hal::pac;
use hal::{clocks::Clock, Timer};
use pimoroni_badger2040 as bsp;

use embedded_graphics::{
    mono_font::{ascii::FONT_4X6, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::Rectangle,
};
use uc8151::{HEIGHT, WIDTH};

#[entry]
fn main() -> ! {
    // Set up all the basic peripherals, and init clocks/timers
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Enable 3.3V power or you won't see anything
    let mut power = pins.p3v3_en.into_push_pull_output();
    power.set_high().unwrap();

    // Set up the pins for the e-ink display
    let spi_sclk = pins.sclk.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.mosi.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, _>::new(pac.SPI0, (spi_mosi, spi_sclk));
    let mut dc = pins.inky_dc.into_push_pull_output();
    let mut cs = pins.inky_cs_gpio.into_push_pull_output();
    let busy = pins.inky_busy.into_pull_up_input();
    let reset = pins.inky_res.into_push_pull_output();

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        RateExtU32::MHz(1),
        embedded_hal::spi::MODE_0,
    );

    dc.set_high().unwrap();
    cs.set_high().unwrap();

    let mut display = uc8151::Uc8151::new(spi, cs, dc, busy, reset);

    // Initialise display. Using the default LUT speed setting
    let _ = display.setup(&mut timer, uc8151::LUT::Internal);

    let mut current = 0i32;
    let mut channel = 0;

    loop {
        let bounds = Rectangle::new(Point::new(0, current), Size::new(WIDTH, 8));

        bounds
            .into_styled(
                PrimitiveStyleBuilder::default()
                    .stroke_color(BinaryColor::Off)
                    .fill_color(BinaryColor::On)
                    .stroke_width(1)
                    .build(),
            )
            .draw(&mut display)
            .unwrap();

        Text::new(
            value_text(channel),
            bounds.center() + Point::new(0, 2),
            MonoTextStyle::new(&FONT_4X6, BinaryColor::Off),
        )
        .draw(&mut display)
        .unwrap();

        display.partial_update(bounds.try_into().unwrap()).unwrap();

        current = (current + 8) % HEIGHT as i32;
        channel = (channel + 1) % 16;

        timer.delay_ms(50);
    }
}

fn value_text(value: i32) -> &'static str {
    const CHANNEL_NUM: &[&str] = &[
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15",
    ];

    #[allow(clippy::cast_sign_loss)]
    CHANNEL_NUM[(value % 16) as usize]
}

// End of file
