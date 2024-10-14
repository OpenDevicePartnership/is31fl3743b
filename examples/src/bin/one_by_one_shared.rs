#![allow(missing_docs)]
#![no_std]
#![no_main]

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{BitOrder, Config, Spi, MODE_0};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use is31fl3743b_driver::{CSy, Is31fl3743b, SWx};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const COLS: [SWx; 11] = [
    SWx::SW1,
    SWx::SW2,
    SWx::SW3,
    SWx::SW4,
    SWx::SW5,
    SWx::SW6,
    SWx::SW7,
    SWx::SW8,
    SWx::SW9,
    SWx::SW10,
    SWx::SW11,
];
const ROWS: [CSy; 18] = [
    CSy::CS1,
    CSy::CS2,
    CSy::CS3,
    CSy::CS4,
    CSy::CS5,
    CSy::CS6,
    CSy::CS7,
    CSy::CS8,
    CSy::CS9,
    CSy::CS10,
    CSy::CS11,
    CSy::CS12,
    CSy::CS13,
    CSy::CS14,
    CSy::CS15,
    CSy::CS16,
    CSy::CS17,
    CSy::CS18,
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // For this example we use Embassy
    let p = embassy_stm32::init(Default::default());

    // Construct a StaticCell to hold a thread safe instance of the SPI bus
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async>>> = StaticCell::new();

    // Configure SPI bus
    let mut spi_config = Config::default();
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.mode = MODE_0;
    // IS31FL3743B can handle up to 12 MHz
    spi_config.frequency = Hertz(10_000_000);

    // Instantiate your SPI bus, this example uses embassy_stm32::spi::Spi
    let spi = Spi::new(p.SPI4, p.PB13, p.PA1, p.PA11, p.DMA2_CH1, p.DMA2_CH0, spi_config);

    // Create mutex guarding the SPI bus and initialize the StaticCell with it
    let mutex: Mutex<NoopRawMutex, Spi<'_, Async>> = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(mutex);

    // Define your chip select pins for your peripheral devices
    let cs = Output::new(p.PB1, Level::High, Speed::VeryHigh);
    let cs2 = Output::new(p.PB2, Level::High, Speed::VeryHigh);

    // Now we can define multiple SPI devices with each chip select pin
    let spi_dev = SpiDevice::new(spi_bus, cs);
    let spi_dev2 = SpiDevice::new(spi_bus, cs2);

    // Instantiate IS31FL3743B driver with multiple instantiations of IS31FL3743B devices
    let mut driver = Is31fl3743b::new_with_sync([spi_dev, spi_dev2]).await.unwrap();

    // Enable phase delay to help reduce power noise
    let _ = driver[0].enable_phase_delay().await;
    let _ = driver[1].enable_phase_delay().await;
    // Set global current, check method documentation for more info
    let _ = driver[0].set_global_current(90).await;
    let _ = driver[1].set_global_current(90).await;

    loop {
        // Turn on each LED one by one
        for i in ROWS.into_iter() {
            for j in COLS.into_iter() {
                for device in 0..2 {
                    let _ = driver[device].set_led_peak_current(j, i, 80).await;
                    let _ = driver[device].set_led_brightness(j, i, 80).await;
                }
                Timer::after(Duration::from_millis(10)).await;
            }
        }

        // Then turn off each LED one by one
        for i in ROWS.into_iter() {
            for j in COLS.into_iter() {
                for device in 0..2 {
                    let _ = driver[device].set_led_brightness(j, i, 0).await;
                }
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}
