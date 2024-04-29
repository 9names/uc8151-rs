#![no_std]
#![no_main]

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::gpio::Input;
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use gpio::{Level, Output, Pull};
use tinybmp::Bmp;
use uc8151::asynch::Uc8151;
use uc8151::LUT;
use uc8151::WIDTH;
use {defmt_rtt as _, panic_probe as _};
static FERRIS_IMG: &[u8; 2622] = include_bytes!("../ferris_1bpp.bmp");
#[cortex_m_rt::pre_init]
unsafe fn before_main() {
    // Soft-reset doesn't clear spinlocks. Clear the one used by critical-section
    // before we hit main to avoid deadlocks when using a debugger
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Program start");
    let p = embassy_rp::init(Default::default());
    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let mut dc = p.PIN_20;
    let mut cs = p.PIN_17;
    let busy = p.PIN_26;
    let reset = p.PIN_21;
    let power = p.PIN_10;

    let btn_up = p.PIN_15;
    let btn_down = p.PIN_11;
    let btn_a = p.PIN_12;
    let btn_b = p.PIN_13;
    let btn_c = p.PIN_14;

    let led = p.PIN_25;

    let reset = Output::new(reset, Level::Low);
    let power = Output::new(power, Level::Low);

    let dc = Output::new(dc, Level::Low);
    let cs = Output::new(cs, Level::High);
    let mut busy = Input::new(busy, Pull::Up);

    let mut led = Output::new(led, Level::Low);
    let mut btn_up = Input::new(btn_up, Pull::Up);
    let mut btn_down = Input::new(btn_down, Pull::Up);
    let mut btn_a = Input::new(btn_a, Pull::Up);
    let mut btn_b = Input::new(btn_b, Pull::Up);
    let mut btn_c = Input::new(btn_c, Pull::Up);

    let spi = Spi::new(
        p.SPI0,
        clk,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        spi::Config::default(),
    );
    let spi_bus: Mutex<NoopRawMutex, _> = Mutex::new(spi);
    let spi_dev = SpiDevice::new(&spi_bus, cs);
    let mut display = Uc8151::new(spi_dev, dc, busy, reset, Delay);
    display.reset().await;

    // Initialise display. Using the default LUT speed setting
    let _ = display.setup(LUT::Internal).await;

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
    let text = "Hello\nasync\nRustacean!";
    let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

    // Draw the text box.
    text_box.draw(&mut display).unwrap();

    // Draw ferris
    let tga: Bmp<BinaryColor> = Bmp::from_slice(FERRIS_IMG).unwrap();
    let image = Image::new(&tga, Point::zero());
    let _ = image.draw(&mut display);
    let _ = display.update().await;

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(Duration::from_secs(1)).await;

        info!("led off!");
        led.set_low();
        Timer::after(Duration::from_secs(1)).await;
    }
}
