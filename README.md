# IS31F13743B

A `#[no_std]` platform-agnostic driver for the
[IS31F13743B](https://lumissil.com/assets/pdf/core/IS31FL3743B_DS.pdf)
LED matrix controller using the [embedded-hal](https://docs.rs/embedded-hal) traits.

## Usage

*Blocking, single device*
```rust,ignore
// Construct and initialize driver (resets and powers on device)
let mut driver = Is31fl3743b::new(SpiDevice::new(spi_instance, cs_pin))?;

// Perform a test which detects shorted LEDs
let delay = DelayNs;
let led_status = driver.detect_shorts(delay)?;
if !led_status.all_leds_ok() {
    panic!("There is a short among the LEDs")
}

// Perform some configuration
driver.enable_phase_delay()?;
driver.set_global_current(90)?;

// Adjust current/brightness of specific LED (by given SWx and CSy coordinates)
driver.set_led_peak_current(SWx::SW2, CSy::CS10, 50)?;
driver.set_led_brightness(SWx::SW2, CSy::CS10, 50)?;
```

*Async, multiple sync'd devices*
```rust,ignore
// Construct and initialize driver (resets and powers on device, as well performs proper SYNC function config)
// First instance in array is considered "master" device for sync purposes
let spi = [SpiDevice::new(spi_instance, cs_pin1), SpiDevice::new(spi_instance, cs_pin2)];
let mut driver = Is31fl3743b::new_with_sync(spi).await?;

// Perform a test which detects shorted LEDs on first device
let delay = DelayNs;
let led_status = driver[0].detect_shorts(delay).await?;
if !led_status.all_leds_ok() {
    panic!("There is a short among the LEDs")
}

// Perform some configuration on second device
driver[1].enable_phase_delay().await?;
driver[1].set_global_current(90).await?;

// Adjust current/brightness of specific LED (by given SWx and CSy coordinates) on second device
driver[1].set_led_peak_current(SWx::SW2, CSy::CS10, 50).await?;
driver[1].set_led_brightness(SWx::SW2, CSy::CS10, 50).await?;
```

## Features

* Enable `is_blocking` to use blocking mode (async mode used by default)
* Enable `preserve_registers` to restore previous PWM and Scaling registers after open/short test (disabled by default)

## License

Licensed under the terms of the [MIT license](http://opensource.org/licenses/MIT).

## Contribution

Unless you explicitly state otherwise, any contribution submitted for
inclusion in the work by you shall be licensed under the terms of the
MIT license.

License: MIT