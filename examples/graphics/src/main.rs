//! Display graphics and text on the badger2040 e-ink display
#![no_std]
#![no_main]

use bsp::entry;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use pimoroni_badger2040 as bsp;

use bsp::hal::clocks::Clock;

// Bring in all the rest of our dependencies from the BSP
use bsp::hal;
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use uc8151::WIDTH;

use tinybmp::Bmp;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pimoroni_badger2040::hal::pac;
use pimoroni_badger2040::hal::Timer;

// A few traits required for using the CountDown timer
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use fugit::RateExtU32;

static FERRIS_IMG: &[u8; 2622] = include_bytes!("../ferris_1bpp.bmp");

#[entry]
fn main() -> ! {
    let core = pac::CorePeripherals::take().unwrap();
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pimoroni_badger2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pimoroni_badger2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral to be a CountDown timer for our blinky delay
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Get all the basic peripherals, and init clocks/timers
    // Enable 3.3V power or you won't see anything
    let mut power = pins.p3v3_en.into_push_pull_output();
    power.set_high().unwrap();

    // Set up the pins for the e-ink display
    let spi_sclk = pins.sclk.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.mosi.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, _>::new(pac.SPI0,(spi_mosi, spi_sclk));
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

    let mut count_down = timer.count_down();
    let mut display = uc8151::Uc8151::new(spi, cs, dc, busy, reset);

    // Reset display
    display.disable();
    count_down.start(10u32.millis());
    let _ = nb::block!(count_down.wait());
    display.enable();
    count_down.start(10u32.millis());
    let _ = nb::block!(count_down.wait());
    // Wait for the screen to finish reset
    while display.is_busy() {}

    // Initialise display. Using the default LUT speed setting
    let _ = display.setup(&mut delay, uc8151::LUT::Internal);

    // Note we're setting the Text color to `Off`. The driver is set up to treat Off as Black so that BMPs work as expected.
    let character_style = MonoTextStyle::new(&FONT_9X18_BOLD, BinaryColor::Off);
    let textbox_style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::FitToText)
        .alignment(HorizontalAlignment::Center)
        .paragraph_spacing(6)
        .build();
    // Bounding box for our text. Fill it with the opposite color so we can read the text.
    let bounds = Rectangle::new(Point::new(157, 10), Size::new(WIDTH - 157, 0));
    bounds
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(&mut display)
        .unwrap();
    // Create the text box and apply styling options.
    let text = "Hello\nfellow\nRustacean!";
    let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);
    // Draw the text box.
    text_box.draw(&mut display).unwrap();

    // Draw ferris
    let tga: Bmp<BinaryColor> = Bmp::from_slice(FERRIS_IMG).unwrap();
    let image = Image::new(&tga, Point::zero());
    let _ = image.draw(&mut display);
    let _ = display.update();
    loop {
        cortex_m::asm::nop();
    }
}

// End of file
