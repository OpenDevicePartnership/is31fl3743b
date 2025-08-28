#![allow(missing_docs)]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{BitOrder, Config, Spi, MODE_0};
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use is31fl3743b_driver::{CSy, Is31fl3743b, SWx};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // For this example we use Embassy
    let p = embassy_stm32::init(Default::default());

    // Configure SPI bus
    let mut spi_config = Config::default();
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.mode = MODE_0;
    // IS31FL3743B can handle up to 12 MHz
    spi_config.frequency = Hertz(10_000_000);

    // Instantiate your SPI bus, this example uses embassy_stm32::spi::Spi
    let spi = Spi::new(p.SPI4, p.PB13, p.PA1, p.PA11, p.DMA2_CH1, p.DMA2_CH0, spi_config);
    // Define your chip select pin
    let cs = Output::new(p.PB1, Level::High, Speed::VeryHigh);
    let delay = Delay;

    // One SPI device only on the SPI bus
    let spi_dev = ExclusiveDevice::new(spi, cs, delay).unwrap();

    // Instantiate IS31FL3743B device
    let mut driver = Is31fl3743b::new(spi_dev).await.unwrap();

    // Enable phase delay to help reduce power noise
    let _ = driver.enable_phase_delay().await;
    // Set global current, check method documentation for more info
    let _ = driver.set_global_current(90).await;

    let mut buf = [100_u8; 11 * 18];
    let _ = driver.set_led_peak_current_bulk(SWx::SW1, CSy::CS1, &buf).await;

    // Driver is fully set up, we can now start turning on LEDs!
    // Create a white breathing effect
    loop {
        for brightness in (0..=255_u8).chain((0..=255).rev()) {
            buf.fill(brightness);
            let _ = driver.set_led_brightness_bulk(SWx::SW1, CSy::CS1, &buf).await;
            Timer::after_micros(1).await;
        }
    }
}
