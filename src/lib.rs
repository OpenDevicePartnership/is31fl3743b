//! This is a platform-agnostic Rust driver for the Lumissil IS31FL3743B LED 
//! Matrix Driver based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.lumissil.com/assets/pdf/core/IS31FL3743B_DS.pdf

#![doc(html_root_url = "https://docs.rs/is31fl3743b-driver/latest")]
#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

use core::ops::{Deref, DerefMut, Index, IndexMut};

#[cfg(feature = "is_blocking")]
use embedded_hal::{
    delay::DelayNs,
    spi::{Operation, SpiDevice},
};
#[cfg(not(feature = "is_blocking"))]
use embedded_hal_async::{
    delay::DelayNs,
    spi::{Operation, SpiDevice},
};

mod registers;
pub use registers::*;

const SW_MIN: u8 = 1;
const SW_MAX: u8 = 11;
const CS_MIN: u8 = 1;
const CS_MAX: u8 = 18;
const OPEN_REG_MIN: u8 = 0x03;
const OPEN_REG_MAX: u8 = 0x23;
const NUM_OPEN_REG: usize = ((OPEN_REG_MAX - OPEN_REG_MIN) + 1) as usize;
const LED_REG_MIN: u8 = 0x01;
const LED_REG_MAX: u8 = 0xC6;
const NUM_LED_REG: usize = ((LED_REG_MAX - LED_REG_MIN) + 1) as usize;

/// Switch column.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SWx {
    /// Switch column 1
    SW1,
    /// Switch column 2
    SW2,
    /// Switch column 3
    SW3,
    /// Switch column 4
    SW4,
    /// Switch column 5
    SW5,
    /// Switch column 6
    SW6,
    /// Switch column 7
    SW7,
    /// Switch column 8
    SW8,
    /// Switch column 9
    SW9,
    /// Switch column 10
    SW10,
    /// Switch column 11
    SW11,
}

impl From<SWx> for u8 {
    fn from(swx: SWx) -> Self {
        match swx {
            SWx::SW1 => 1,
            SWx::SW2 => 2,
            SWx::SW3 => 3,
            SWx::SW4 => 4,
            SWx::SW5 => 5,
            SWx::SW6 => 6,
            SWx::SW7 => 7,
            SWx::SW8 => 8,
            SWx::SW9 => 9,
            SWx::SW10 => 10,
            SWx::SW11 => 11,
        }
    }
}

impl TryFrom<u8> for SWx {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(SWx::SW1),
            2 => Ok(SWx::SW2),
            3 => Ok(SWx::SW3),
            4 => Ok(SWx::SW4),
            5 => Ok(SWx::SW5),
            6 => Ok(SWx::SW6),
            7 => Ok(SWx::SW7),
            8 => Ok(SWx::SW8),
            9 => Ok(SWx::SW9),
            10 => Ok(SWx::SW10),
            11 => Ok(SWx::SW11),
            _ => Err(()),
        }
    }
}

/// Current sink row.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CSy {
    /// Current sink row 1
    CS1,
    /// Current sink row 2
    CS2,
    /// Current sink row 3
    CS3,
    /// Current sink row 4
    CS4,
    /// Current sink row 5
    CS5,
    /// Current sink row 6
    CS6,
    /// Current sink row 7
    CS7,
    /// Current sink row 8
    CS8,
    /// Current sink row 9
    CS9,
    /// Current sink row 10
    CS10,
    /// Current sink row 11
    CS11,
    /// Current sink row 12
    CS12,
    /// Current sink row 13
    CS13,
    /// Current sink row 14
    CS14,
    /// Current sink row 15
    CS15,
    /// Current sink row 16
    CS16,
    /// Current sink row 17
    CS17,
    /// Current sink row 18
    CS18,
}

impl From<CSy> for u8 {
    fn from(csy: CSy) -> Self {
        match csy {
            CSy::CS1 => 1,
            CSy::CS2 => 2,
            CSy::CS3 => 3,
            CSy::CS4 => 4,
            CSy::CS5 => 5,
            CSy::CS6 => 6,
            CSy::CS7 => 7,
            CSy::CS8 => 8,
            CSy::CS9 => 9,
            CSy::CS10 => 10,
            CSy::CS11 => 11,
            CSy::CS12 => 12,
            CSy::CS13 => 13,
            CSy::CS14 => 14,
            CSy::CS15 => 15,
            CSy::CS16 => 16,
            CSy::CS17 => 17,
            CSy::CS18 => 18,
        }
    }
}

impl TryFrom<u8> for CSy {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(CSy::CS1),
            2 => Ok(CSy::CS2),
            3 => Ok(CSy::CS3),
            4 => Ok(CSy::CS4),
            5 => Ok(CSy::CS5),
            6 => Ok(CSy::CS6),
            7 => Ok(CSy::CS7),
            8 => Ok(CSy::CS8),
            9 => Ok(CSy::CS9),
            10 => Ok(CSy::CS10),
            11 => Ok(CSy::CS11),
            12 => Ok(CSy::CS12),
            13 => Ok(CSy::CS13),
            14 => Ok(CSy::CS14),
            15 => Ok(CSy::CS15),
            16 => Ok(CSy::CS16),
            17 => Ok(CSy::CS17),
            18 => Ok(CSy::CS18),
            _ => Err(()),
        }
    }
}

// Converts SWx and CSy coordinates to an LED register offset.
// This is mainly used as an address offset to the PWM and Scaling register addresses.
// Number is calculated based on diagram from Figure 10 in datasheet.
fn led_reg_offset(swx: SWx, csy: CSy) -> u8 {
    (u8::from(csy) + (u8::from(swx) - 1) * CS_MAX) - LED_REG_MIN
}

// Converts a percentage (from 0-100%) to a raw fraction of u8 max (0xFF).
#[allow(clippy::cast_possible_truncation)]
const fn raw_from_percent(percent: u8) -> u8 {
    debug_assert!(percent <= 100);
    ((0xFF * percent as u16) / 100) as u8
}

#[cfg(feature = "preserve_registers")]
#[derive(Clone, Copy, Debug)]
struct LedRegList([u8; NUM_LED_REG]);
#[cfg(feature = "preserve_registers")]
impl LedRegList {
    fn inner_ref(&self) -> &[u8; NUM_LED_REG] {
        &self.0
    }
}

#[cfg(feature = "preserve_registers")]
impl Index<usize> for LedRegList {
    type Output = u8;
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

#[cfg(feature = "preserve_registers")]
impl IndexMut<usize> for LedRegList {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

#[cfg(feature = "preserve_registers")]
impl Default for LedRegList {
    fn default() -> Self {
        Self([0x00; NUM_LED_REG])
    }
}

// Maintains local, cached copies of device registers
#[derive(Default, Clone, Copy, Debug)]
struct CachedReg {
    config: Configuration,
    gcc: GlobalCurrentControl,
    res_phase: PullDownUpResistorSelection,
    temperature: TemperatureStatus,
    spread_spectrum: SpreadSpectrum,
    #[cfg(feature = "preserve_registers")]
    pwm: LedRegList,
    #[cfg(feature = "preserve_registers")]
    scaling: LedRegList,
}

/// Contains the status of each LED after the last open/short detection test.
///
/// A new test must be performed to receive an updated status.
#[derive(Clone, Copy, Debug)]
pub struct OpenShortTestResult([u8; NUM_OPEN_REG]);

impl OpenShortTestResult {
    /// Returns true if every LED is functioning correctly (no shorts/open), false otherwise.
    #[must_use]
    pub fn all_leds_ok(&self) -> bool {
        self.0.iter().all(|&r| r == 0)
    }

    /// Returns true if specified LED is functioning correctly (not shorted/open), false otherwise.
    #[must_use]
    pub fn led_ok(&self, swx: u8, csy: u8) -> bool {
        debug_assert!((SW_MIN..=SW_MAX).contains(&swx) && (CS_MIN..=CS_MAX).contains(&csy));

        // Calculations based on diagrams from Figure 12 and Table 9 in datasheet.
        let idx = usize::from(((csy - 1) / 6) + ((swx - 1) * 3));
        let bit = (csy - 1) % 6;

        self.0[idx] & (1 << bit) == 0
    }
}

/// IS31FL3743B device driver group
#[derive(Clone, Copy, Debug)]
pub struct Is31fl3743b<const N: usize, SPI>([Is31fl3743bDevice<SPI>; N]);

impl<const N: usize, SPI: SpiDevice> Index<usize> for Is31fl3743b<N, SPI> {
    type Output = Is31fl3743bDevice<SPI>;
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const N: usize, SPI: SpiDevice> IndexMut<usize> for Is31fl3743b<N, SPI> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl<const N: usize, SPI: SpiDevice> Deref for Is31fl3743b<N, SPI> {
    type Target = Is31fl3743bDevice<SPI>;

    fn deref(&self) -> &Self::Target {
        &self.0[0]
    }
}

impl<const N: usize, SPI: SpiDevice> DerefMut for Is31fl3743b<N, SPI> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0[0]
    }
}

#[maybe_async::maybe_async]
impl<const N: usize, SPI: SpiDevice> Is31fl3743b<N, SPI> {
    /// Create new SYNCd group of IS31FL3743B instances.
    ///
    /// This will automatically configure the master (index 0) and
    /// slaves (index 1..N) into the proper SYNC mode (see [Is31fl3743bDevice::enable_sync]).
    ///
    /// The returned group of device instances can be indexed into using
    /// the respective index of SPI instances passed in.
    ///
    /// If using a single device, prefer [Self::new].
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn new_with_sync(spi: [SPI; N]) -> Result<Self, SPI::Error> {
        let mut devices = spi.map(|spi| Is31fl3743bDevice::new(spi));

        // Slaves must be configured before master
        for slave in devices.iter_mut().skip(1) {
            slave.reset().await?;
            slave.enable_sync(Sync::Slave).await?;
            slave.enable().await?;
        }

        // Configure master
        devices[0].reset().await?;
        devices[0].enable_sync(Sync::Master).await?;
        devices[0].enable().await?;

        Ok(Self(devices))
    }

    /// Destroy the driver instances and return SPI instances.
    pub fn destroy_with_sync(self) -> [SPI; N] {
        self.0.map(Is31fl3743bDevice::destroy)
    }
}

#[maybe_async::maybe_async]
impl<SPI: SpiDevice> Is31fl3743b<1, SPI> {
    /// Create new IS31FL3743B instance.
    ///
    /// If using multiple devices that will be tied together via SYNC pin,
    /// prefer [Self::new_with_sync] for automatic configuration.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn new(spi: SPI) -> Result<Self, SPI::Error> {
        let mut devices = [Is31fl3743bDevice::new(spi); 1];
        devices[0].reset().await?;
        devices[0].enable().await?;

        Ok(Self(devices))
    }

    /// Destroy the driver instance and return SPI instance.
    #[allow(clippy::missing_panics_doc)]
    pub fn destroy(self) -> SPI {
        // Simply moving the first element out of the array is not allowed by the compiler.
        // Convert to iterator to allow us to do this.
        self.0
            .into_iter()
            .next()
            .expect("Array is non-empty; this will never panic")
            .destroy()
    }
}

/// IS31FL3743B device driver
#[derive(Clone, Copy, Debug)]
pub struct Is31fl3743bDevice<SPI> {
    /// The concrete SPI instance.
    spi: SPI,

    /// Local cached copy of device registers (since device does not support reads)
    cached_reg: CachedReg,
}

#[maybe_async::maybe_async]
impl<SPI: SpiDevice> Is31fl3743bDevice<SPI> {
    /// Sets the brightness of an LED specified by given `SWx` and `CSy` coordinates as a percentage.
    ///
    /// This effectively modulates the PWM duty cycle by the given percentage
    /// and sets the average current for the specified LED.
    ///
    /// For finer control, set the PWM register directly with [Self::set_pwm_reg].
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_led_brightness(
        &mut self,
        swx: SWx,
        csy: CSy,
        brightness_percentage: u8,
    ) -> Result<(), SPI::Error> {
        let addr_offset = led_reg_offset(swx, csy);
        let raw = raw_from_percent(brightness_percentage);
        self.write_with_addr_offset(Register::Pwm, addr_offset, raw).await
    }

    /// Sets the peak current of an LED specified by given `SWx` and `CSy` coordinates
    /// as a percentage of global current.
    ///
    /// For finer control, set the Scaling register directly with [Self::set_scaling_reg].
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_led_peak_current(&mut self, swx: SWx, csy: CSy, scale_percentage: u8) -> Result<(), SPI::Error> {
        let addr_offset = led_reg_offset(swx, csy);
        let raw = raw_from_percent(scale_percentage);
        self.write_with_addr_offset(Register::Scaling, addr_offset, raw).await
    }

    /// Power on the device.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn enable(&mut self) -> Result<(), SPI::Error> {
        self.set_configuration_reg(self.cached_reg.config.with_ssd(true)).await
    }

    /// Power off the device.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn disable(&mut self) -> Result<(), SPI::Error> {
        self.set_configuration_reg(self.cached_reg.config.with_ssd(false)).await
    }

    /// Enables switch columns from SW1 up to the column specified.
    ///
    /// Also has the effect of changing the duty cycle (which is 1/11 when all columns active).
    ///
    /// e.g. Enabling up to SW3 changes duty cycle to 1/3.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn switch_column_enable_upto(&mut self, switch_column: SwxSetting) -> Result<(), SPI::Error> {
        self.set_configuration_reg(self.cached_reg.config.with_sws(switch_column))
            .await
    }

    /// Sets the global current as a percentage of max current.
    ///
    /// Max current is determined by the external resistor connected to Iset and can be calculated as:
    ///
    /// `current = 343 / Riset`
    ///
    /// For finer control, set the GCC register directly with [Self::set_global_current_control_reg].
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_global_current(&mut self, scale_percentage: u8) -> Result<(), SPI::Error> {
        let gcc = GlobalCurrentControl::from(raw_from_percent(scale_percentage));
        self.set_global_current_control_reg(gcc).await
    }

    /// Sets the internal pull-up resistor to be used by all current sink rows.
    ///
    /// Primarily used for LED "de-ghosting". The default value (2k Ohms) is sufficient in most cases.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_cs_pullup(&mut self, pur: Resistor) -> Result<(), SPI::Error> {
        self.set_pdr_pur_resistor_reg(self.cached_reg.res_phase.with_cspur(pur))
            .await
    }

    /// Sets the internal pull-down resistor to be used by all switch columns.
    ///
    /// Primarily used for "de-ghosting". The default value (2k Ohms) is sufficient in most cases.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_sw_pulldown(&mut self, pdr: Resistor) -> Result<(), SPI::Error> {
        self.set_pdr_pur_resistor_reg(self.cached_reg.res_phase.with_swpdr(pdr))
            .await
    }

    /// Enable 180 degree phase delay.
    ///
    /// Primarily used to help reduce power noise.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn enable_phase_delay(&mut self) -> Result<(), SPI::Error> {
        self.set_pdr_pur_resistor_reg(self.cached_reg.res_phase.with_phc(true))
            .await
    }

    /// Disable 180 degree phase delay.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn disable_phase_delay(&mut self) -> Result<(), SPI::Error> {
        self.set_pdr_pur_resistor_reg(self.cached_reg.res_phase.with_phc(false))
            .await
    }

    /// Performs a short detection test and returns the test result.
    ///
    /// This test is relatively time-consuming and should ideally only
    /// be performed once or periodically, and not in a tight loop.
    ///
    /// Overwrites the `PWM` and `Scaling` registers, so `preserve_registers` must be enabled
    /// for the original values to be restored.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn detect_shorts(&mut self, delay: impl DelayNs) -> Result<OpenShortTestResult, SPI::Error> {
        self.open_short_test(Open::EnableShort, delay).await
    }

    /// Performs an open detection test and returns the test result.
    ///
    /// This test is relatively expensive and should ideally only
    /// be performed once or periodically, and not in a tight loop.
    ///
    /// Overwrites the `PWM` and `Scaling` registers, so `preserve_registers` must be enabled
    /// for the original values to be restored.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn detect_opens(&mut self, delay: impl DelayNs) -> Result<OpenShortTestResult, SPI::Error> {
        self.open_short_test(Open::EnableOpen, delay).await
    }

    /// Sets the thermal roll-off as a percentage of output current.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_thermal_rolloff(&mut self, trof: ThermalRollOff) -> Result<(), SPI::Error> {
        self.set_temperature_status_reg(self.cached_reg.temperature.with_trof(trof))
            .await
    }

    /// Sets the temperature point (in degrees Celsius) of the IC above which thermal roll-off will begin.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_temperature_point(&mut self, ts: TemperaturePoint) -> Result<(), SPI::Error> {
        self.set_temperature_status_reg(self.cached_reg.temperature.with_ts(ts))
            .await
    }

    /// Enables spread spectrum function.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn enable_spread_spectrum(&mut self) -> Result<(), SPI::Error> {
        self.set_spread_spectrum_reg(self.cached_reg.spread_spectrum.with_ssp(true))
            .await
    }

    /// Disables spread spectrum function.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn disable_spread_spectrum(&mut self) -> Result<(), SPI::Error> {
        self.set_spread_spectrum_reg(self.cached_reg.spread_spectrum.with_ssp(false))
            .await
    }

    /// Sets the spread spectrum cycle time in microseconds.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_spread_spectrum_cycle_time(&mut self, clt: CycleTime) -> Result<(), SPI::Error> {
        self.set_spread_spectrum_reg(self.cached_reg.spread_spectrum.with_clt(clt))
            .await
    }

    /// Sets the spread spectrum cycle range.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_spread_spectrum_range(&mut self, rng: Range) -> Result<(), SPI::Error> {
        self.set_spread_spectrum_reg(self.cached_reg.spread_spectrum.with_rng(rng))
            .await
    }

    /// Enables sync function configured in either master or slave mode.
    ///
    /// All slave chips should be set to operate in slave mode before configuring master chip.
    ///
    /// See Note 4 in datasheet.
    ///
    /// Prefer creating a SYNCd group of devices via [Is31fl3743b::new_with_sync]
    /// as opposed to managing this manually for individual devices.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn enable_sync(&mut self, mode: Sync) -> Result<(), SPI::Error> {
        self.set_spread_spectrum_reg(self.cached_reg.spread_spectrum.with_sync(mode))
            .await
    }

    /// Disables sync function which pulls SYNC pin low via internal 30k Ohm resistor.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn disable_sync(&mut self) -> Result<(), SPI::Error> {
        self.enable_sync(Sync::Disabled).await
    }

    /// Resets device, returning all registers to their default state.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn reset(&mut self) -> Result<(), SPI::Error> {
        self.write(Register::Reset, 0xAE).await?;
        self.cached_reg = CachedReg::default();
        Ok(())
    }

    /* See application note AN-107 for reference:
     * https://www.lumissil.com/assets/pdf/support/app%20notes/AN-107%20OPEN%20SHORT%20TEST%20FUNCTION%20OF%20IS31FL3743B.pdf
     */
    async fn open_short_test(
        &mut self,
        test: Open,
        mut delay: impl DelayNs,
    ) -> Result<OpenShortTestResult, SPI::Error> {
        // Backup cached registers to restore later
        let tmp_gcc = self.cached_reg.gcc;
        let tmp_res_phase = self.cached_reg.res_phase;

        #[cfg(feature = "preserve_registers")]
        let tmp_pwm = self.cached_reg.pwm;
        #[cfg(feature = "preserve_registers")]
        let tmp_scaling = self.cached_reg.scaling;

        // Ensure OSDE field is clear before starting test (test only begins on transition from 0x00)
        if self.cached_reg.config.osde() != Open::Disabled {
            self.set_configuration_reg(self.cached_reg.config.with_osde(Open::Disabled))
                .await?;
        }

        // Set PWM and Scaling registers to max value
        self.write_multiple(Register::Pwm, &[0xFF; NUM_LED_REG]).await?;
        #[cfg(feature = "preserve_registers")]
        {
            self.cached_reg.pwm = LedRegList([0xFF; NUM_LED_REG]);
        }
        self.write_multiple(Register::Scaling, &[0xFF; NUM_LED_REG]).await?;
        #[cfg(feature = "preserve_registers")]
        {
            self.cached_reg.scaling = LedRegList([0xFF; NUM_LED_REG]);
        }

        // Write 0x0F (recommended value from application notes) to GCC before starting test
        self.set_global_current_control_reg(GlobalCurrentControl::from(0x0F))
            .await?;

        // Clear pur/pdr register before starting test, as recommended by datasheet when GCC is 0x0F
        self.set_pdr_pur_resistor_reg(PullDownUpResistorSelection::from(0b0000_0000))
            .await?;

        // Begin test by setting the OSDE field to the appropriate test type (Open or Short)
        self.set_configuration_reg(self.cached_reg.config.with_osde(test))
            .await?;

        // Wait 1ms (datasheet only specifies AT LEAST 2 scan cycles; application note recommends 1ms)
        delay.delay_ms(1).await;

        // Restore previous GCC register value
        self.set_global_current_control_reg(tmp_gcc).await?;

        // Restore previous pur/pdr register value
        self.set_pdr_pur_resistor_reg(tmp_res_phase).await?;

        // Restore previous PWM and Scaling register values
        #[cfg(feature = "preserve_registers")]
        {
            self.write_multiple(Register::Pwm, tmp_pwm.inner_ref()).await?;
            self.cached_reg.pwm = tmp_pwm;

            self.write_multiple(Register::Scaling, tmp_scaling.inner_ref()).await?;
            self.cached_reg.scaling = tmp_scaling;
        }

        // Read all `Open` registers and collect into test result struct
        Ok(OpenShortTestResult(
            self.read_multiple_with_addr_offset::<NUM_OPEN_REG>(Register::Open, 0)
                .await?,
        ))
    }

    /// Sets the Nth (where 0x01 <= N <= 0xC6) PWM register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_pwm_reg(&mut self, n: u8, value: u8) -> Result<(), SPI::Error> {
        debug_assert!((LED_REG_MIN..=LED_REG_MAX).contains(&n));

        let offset = n - LED_REG_MIN;
        self.write_with_addr_offset(Register::Pwm, offset, value).await?;
        #[cfg(feature = "preserve_registers")]
        {
            self.cached_reg.pwm[offset as usize] = value;
        }
        Ok(())
    }

    /// Sets the Nth (where 0x01 <= N <= 0xC6) Scaling register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_scaling_reg(&mut self, n: u8, value: u8) -> Result<(), SPI::Error> {
        debug_assert!((LED_REG_MIN..=LED_REG_MAX).contains(&n));

        let offset = n - LED_REG_MIN;
        self.write_with_addr_offset(Register::Scaling, offset, value).await?;
        #[cfg(feature = "preserve_registers")]
        {
            self.cached_reg.scaling[offset as usize] = value;
        }
        Ok(())
    }

    /// Sets the configuration register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_configuration_reg(&mut self, value: Configuration) -> Result<(), SPI::Error> {
        self.write(Register::Configuration, value.into()).await?;
        self.cached_reg.config = value;
        Ok(())
    }

    /// Sets the global current control register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_global_current_control_reg(&mut self, value: GlobalCurrentControl) -> Result<(), SPI::Error> {
        self.write(Register::GlobalCurrentControl, value.into()).await?;
        self.cached_reg.gcc = value;
        Ok(())
    }

    /// Sets the pull down/up resistor selection register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_pdr_pur_resistor_reg(&mut self, value: PullDownUpResistorSelection) -> Result<(), SPI::Error> {
        self.write(Register::PullDownUpResistorSelection, value.into()).await?;
        self.cached_reg.res_phase = value;
        Ok(())
    }

    /// Sets the temperature status register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_temperature_status_reg(&mut self, value: TemperatureStatus) -> Result<(), SPI::Error> {
        self.write(Register::TemperatureStatus, value.into()).await?;
        self.cached_reg.temperature = value;
        Ok(())
    }

    /// Sets the spread spectrum register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn set_spread_spectrum_reg(&mut self, value: SpreadSpectrum) -> Result<(), SPI::Error> {
        self.write(Register::SpreadSpectrum, value.into()).await?;
        self.cached_reg.spread_spectrum = value;
        Ok(())
    }

    /// Gets the Nth (where 0x03 <= N <= 0x23) open register.
    ///
    /// # Errors
    ///
    /// `SPI::Error` when SPI transaction fails
    pub async fn open_reg(&mut self, n: u8) -> Result<u8, SPI::Error> {
        debug_assert!((OPEN_REG_MIN..=OPEN_REG_MAX).contains(&n));
        self.read_with_addr_offset(Register::Open, n - OPEN_REG_MIN).await
    }

    /// Gets the cached Nth (where 0x01 <= N <= 0xC6) PWM register.
    #[cfg(feature = "preserve_registers")]
    pub fn pwm_reg(&mut self, n: u8) -> u8 {
        debug_assert!((LED_REG_MIN..=LED_REG_MAX).contains(&n));
        self.cached_reg.pwm[(n - LED_REG_MIN) as usize]
    }

    /// Gets the cached Nth (where 0x01 <= N <= 0xC6) Scaling register.
    #[cfg(feature = "preserve_registers")]
    pub fn scaling_reg(&mut self, n: u8) -> u8 {
        debug_assert!((LED_REG_MIN..=LED_REG_MAX).contains(&n));
        self.cached_reg.scaling[(n - LED_REG_MIN) as usize]
    }

    /// Gets the cached configuration register.
    pub fn configuration_reg(&mut self) -> Configuration {
        self.cached_reg.config
    }

    /// Gets the global current control register.
    pub fn global_current_control_reg(&mut self) -> GlobalCurrentControl {
        self.cached_reg.gcc
    }

    /// Gets the cached pull down/up resistor selection register.
    pub fn pdr_pur_resistor_reg(&mut self) -> PullDownUpResistorSelection {
        self.cached_reg.res_phase
    }

    /// Gets the cached temperature status register.
    pub fn temperature_status_reg(&mut self) -> TemperatureStatus {
        self.cached_reg.temperature
    }

    /// Gets the cached spread spectrum register.
    pub fn spread_spectrum_reg(&mut self) -> SpreadSpectrum {
        self.cached_reg.spread_spectrum
    }

    async fn read_with_addr_offset(&mut self, reg: Register, offset: u8) -> Result<u8, SPI::Error> {
        let data = self.read_multiple_with_addr_offset::<1>(reg, offset).await?;
        Ok(data[0])
    }

    fn new(spi: SPI) -> Self {
        Self {
            spi,
            cached_reg: CachedReg::default(),
        }
    }

    fn destroy(self) -> SPI {
        self.spi
    }

    async fn read_multiple_with_addr_offset<const N: usize>(
        &mut self,
        reg: Register,
        offset: u8,
    ) -> Result<[u8; N], SPI::Error> {
        let cmd_byte: u8 = Command::default()
            .with_rw(CommandType::Read)
            .with_page(reg.page())
            .into();

        let write_buf = [cmd_byte, u8::from(reg) + offset];
        let write_op = Operation::Write(&write_buf);

        // Device will automatically increment register address for every sequential read
        let mut data: [u8; N] = [0; N];
        let read_op = Operation::Read(&mut data);

        /* Read and write are combined in a single transaction since data is received
         * immediately following the write of the command byte. This does not occur in
         * full-duplex mode (aka simultaneously) hence the use of custom transaction
         * over a `transfer` transaction.
         */
        self.spi.transaction(&mut [write_op, read_op]).await?;
        Ok(data)
    }

    async fn write(&mut self, reg: Register, value: u8) -> Result<(), SPI::Error> {
        self.write_with_addr_offset(reg, 0, value).await
    }

    async fn write_with_addr_offset(&mut self, reg: Register, offset: u8, value: u8) -> Result<(), SPI::Error> {
        let cmd_byte: u8 = Command::default()
            .with_rw(CommandType::Write)
            .with_page(reg.page())
            .into();
        self.spi.write(&[cmd_byte, u8::from(reg) + offset, value]).await
    }

    async fn write_multiple(&mut self, reg: Register, data: &[u8]) -> Result<(), SPI::Error> {
        let cmd_byte: u8 = Command::default()
            .with_rw(CommandType::Write)
            .with_page(reg.page())
            .into();
        let cmd_write_buf = [cmd_byte, reg.into()];

        let cmd_write_op = Operation::Write(&cmd_write_buf);
        let data_write_op = Operation::Write(data);
        self.spi.transaction(&mut [cmd_write_op, data_write_op]).await
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    use super::*;

    #[maybe_async::test(feature = "is_blocking", async(not(feature = "is_blocking"), tokio::test))]
    async fn write_configuration_register() {
        let expectations = vec![
            // Initial reset
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x2F, 0xAE]),
            Transaction::transaction_end(),
            // Initial power on
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x00, 0x09]),
            Transaction::transaction_end(),
            // Write new configuration
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x00, 0x69]),
            Transaction::transaction_end(),
        ];

        let mock = Mock::new(&expectations);

        let mut m = Is31fl3743b::new(mock).await.unwrap();
        m.switch_column_enable_upto(SwxSetting::Sw5).await.unwrap();

        let mut mock = m.destroy();
        mock.done();
    }

    #[maybe_async::test(feature = "is_blocking", async(not(feature = "is_blocking"), tokio::test))]
    async fn new_with_sync() {
        let expectations1 = vec![
            // Initial reset
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x2F, 0xAE]),
            Transaction::transaction_end(),
            // Write master sync config
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x25, 0xC0]),
            Transaction::transaction_end(),
            // Initial power on
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x00, 0x09]),
            Transaction::transaction_end(),
        ];
        let expectations2 = vec![
            // Initial reset
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x2F, 0xAE]),
            Transaction::transaction_end(),
            // Write slave sync config
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x25, 0x80]),
            Transaction::transaction_end(),
            // Initial power on
            Transaction::transaction_start(),
            Transaction::write_vec(vec![0x52, 0x00, 0x09]),
            Transaction::transaction_end(),
        ];

        let mock1 = Mock::new(&expectations1);
        let mock2 = Mock::new(&expectations2);
        let mocks = [mock1, mock2];

        let m = Is31fl3743b::new_with_sync(mocks).await.unwrap();

        let mocks = m.destroy_with_sync();
        mocks.into_iter().for_each(|mut mock| mock.done());
    }

    #[test]
    fn led_register_calculation() {
        assert_eq!(led_reg_offset(SWx::SW1, CSy::CS1), 0x00);
        assert_eq!(led_reg_offset(SWx::SW4, CSy::CS16), 0x45);
        assert_eq!(led_reg_offset(SWx::SW9, CSy::CS2), 0x91);
        assert_eq!(led_reg_offset(SWx::SW11, CSy::CS18), 0xC5);
    }
}
