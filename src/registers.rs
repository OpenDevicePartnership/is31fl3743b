#![allow(missing_docs)]
use bilge::prelude::*;

/// Command byte.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub(crate) struct Command {
    /// Page number of the register this command will operate on.
    pub page: PageNumber,

    /// The chip ID (always 0b101).
    id: u3,

    /// The type of command this is (read or write).
    pub rw: CommandType,
}

impl Default for Command {
    fn default() -> Self {
        Self::from(0b0101_0000)
    }
}

impl Command {
    /// Set page number.
    #[must_use]
    pub fn with_page(mut self, page: PageNumber) -> Self {
        self.set_page(page);
        Self::from(self.value)
    }

    /// Set R/W command type.
    #[must_use]
    pub fn with_rw(mut self, rw: CommandType) -> Self {
        self.set_rw(rw);
        Self::from(self.value)
    }
}

/// Page number.
#[bitsize(4)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub(crate) enum PageNumber {
    /// Page 0.
    #[fallback]
    Pg0,

    /// Page 1.
    Pg1,

    /// Page 2.
    Pg2,
}

/// Command type.
#[bitsize(1)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub(crate) enum CommandType {
    /// Write command.
    Write,

    /// Read command.
    Read,
}

/// Register addresses
#[derive(Debug, PartialEq, PartialOrd)]
pub(crate) enum Register {
    /// PWM modulation for each LED.
    Pwm,

    /// Peak current scaling for each LED (as fraction of global current).
    Scaling,

    /// Operation mode configuration.
    Configuration,

    /// Global current scaling (as fraction of max current).
    GlobalCurrentControl,

    /// Set the pull-down resistor for switching columns
    /// and pull-up resistor for current sink rows.
    ///
    /// Also contains setting for phase delay.
    PullDownUpResistorSelection,

    /// Contains the open information (after performing open test)
    /// or short information (after performing short test).
    Open,

    /// Temperature set point and thermal roll-off configuration.
    TemperatureStatus,

    /// Spread spectrum and sync configuration.
    SpreadSpectrum,

    /// Performs register reset when written with 0xAE.
    Reset,
}

impl Register {
    /// Returns the page number the register belongs to.
    pub fn page(&self) -> PageNumber {
        match *self {
            Self::Pwm => PageNumber::Pg0,
            Self::Scaling => PageNumber::Pg1,
            _ => PageNumber::Pg2,
        }
    }
}

impl From<Register> for u8 {
    fn from(reg: Register) -> Self {
        match reg {
            Register::Configuration => 0x00,
            Register::Pwm | Register::Scaling | Register::GlobalCurrentControl => 0x01,
            Register::PullDownUpResistorSelection => 0x02,
            Register::Open => 0x03,
            Register::TemperatureStatus => 0x24,
            Register::SpreadSpectrum => 0x25,
            Register::Reset => 0x2F,
        }
    }
}

/// Configuration register.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct Configuration {
    /// Software shutdown control.
    pub ssd: bool,

    /// Open/short detection enable.
    pub osde: Open,

    reserved: u1,

    /// `SWx` setting (controls duty cycle).
    pub sws: SwxSetting,
}

impl Default for Configuration {
    fn default() -> Self {
        /* Datasheet inconsistently specifies the default as both:
         *
         * 0b0000_0000 and 0b0000_1000
         *
         * Since 0b0000_1000 is listed twice,
         * and D3 is explicitly mentioned as must be configured as "1",
         * go with this default.
         */
        Self::from(0b0000_1000)
    }
}

impl Configuration {
    /// Configure software shutdown control.
    #[must_use]
    pub fn with_ssd(mut self, ssd: bool) -> Self {
        self.set_ssd(ssd);
        Self::from(self.value)
    }

    /// Configure open/short detection.
    #[must_use]
    pub fn with_osde(mut self, osde: Open) -> Self {
        self.set_osde(osde);
        Self::from(self.value)
    }

    /// Configure `SWx` setting.
    #[must_use]
    pub fn with_sws(mut self, sws: SwxSetting) -> Self {
        self.set_sws(sws);
        Self::from(self.value)
    }
}

/// Open/short detection enable.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Open {
    /// Disable Open/short detection.
    #[fallback]
    Disabled,

    /// Enable open detection.
    EnableOpen,

    /// Enable short detection.
    EnableShort,
}

/// `SWx` setting.
#[bitsize(4)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum SwxSetting {
    /// SW1-SW11 active, 1/11.
    #[fallback]
    Sw11,

    /// SW1-SW10 active, 1/10.
    Sw10,

    /// SW1-SW9 active, 1/9.
    Sw9,

    /// SW1-SW8 active, 1/8.
    Sw8,

    /// SW1-SW7 active, 1/7.
    Sw7,

    /// SW1-SW6 active, 1/6.
    Sw6,

    /// SW1-SW5 active, 1/5.
    Sw5,

    /// SW1-SW4 active, 1/4.
    Sw4,

    /// SW1-SW3 active, 1/3.
    Sw3,

    /// SW1-SW2 active, 1/2.
    Sw2,

    /// No scan, current sink only.
    None,
}

/// Global current control register.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct GlobalCurrentControl(u8);
impl Default for GlobalCurrentControl {
    fn default() -> Self {
        Self::from(0b0000_0000)
    }
}

/// Pull-up/down resistor selection register.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct PullDownUpResistorSelection {
    /// `CSy` pull-up resistor selection.
    pub cspur: Resistor,

    reserved: u1,

    /// `SWx` pull-down resistor selection.
    pub swpdr: Resistor,

    /// Configure 180 degree phase delay enable.
    pub phc: bool,
}

impl Default for PullDownUpResistorSelection {
    fn default() -> Self {
        Self::from(0b0011_0011)
    }
}

impl PullDownUpResistorSelection {
    /// Configure `CSy` pull-up resistor.
    #[must_use]
    pub fn with_cspur(mut self, cspur: Resistor) -> Self {
        self.set_cspur(cspur);
        Self::from(self.value)
    }

    /// Configure `SWx` pull-down resistor.
    #[must_use]
    pub fn with_swpdr(mut self, swpdr: Resistor) -> Self {
        self.set_swpdr(swpdr);
        Self::from(self.value)
    }

    /// Configure 180 degree phase delay.
    #[must_use]
    pub fn with_phc(mut self, phc: bool) -> Self {
        self.set_phc(phc);
        Self::from(self.value)
    }
}

/// Resistor setting (value in Ohms).
#[bitsize(3)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Resistor {
    /// No pull-up/down resistor.
    None,

    /// 0.5k pull-up/down, only in off time.
    R0_5kOffOnly,

    /// 1.0k pull-up/down, only in off time.
    R1_0kOffOnly,

    /// 2.0k pull-up/down, only in off time.
    R2_0kOffOnly,

    /// 1.0k pull-up/down, all the time.
    R1_0k,

    /// 2.0k pull-up/down, all the time.
    R2_0k,

    /// 4.0k pull-up/down, all the time.
    R4_0k,

    /// 8.0k pull-up/down, all the time.
    R8_0k,
}

/// Temperature status register.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct TemperatureStatus {
    /// Thermal roll-off setting.
    pub trof: ThermalRollOff,

    /// Temperature point setting.
    pub ts: TemperaturePoint,

    reserved: u4,
}

impl Default for TemperatureStatus {
    fn default() -> Self {
        Self::from(0b0000_0000)
    }
}

impl TemperatureStatus {
    /// Configure thermal roll-off.
    #[must_use]
    pub fn with_trof(mut self, trof: ThermalRollOff) -> Self {
        self.set_trof(trof);
        Self::from(self.value)
    }

    /// Configure temperature point.
    #[must_use]
    pub fn with_ts(mut self, ts: TemperaturePoint) -> Self {
        self.set_ts(ts);
        Self::from(self.value)
    }
}

/// Thermal roll-off setting.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum ThermalRollOff {
    /// 100% of output current.
    P100,

    /// 75% of output current.
    P75,

    /// 55% of output current.
    P55,

    /// 30% of output current.
    P30,
}

/// Temperature point setting.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum TemperaturePoint {
    /// 140 degrees Celsius thermal roll-off start point.
    D140,

    /// 120 degrees Celsius thermal roll-off start point.
    D120,

    /// 100 degrees Celsius thermal roll-off start point.
    D100,

    /// 90 degrees Celsius thermal roll-off start point.
    D90,
}

/// Spread spectrum register.
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct SpreadSpectrum {
    /// Spread spectrum cycle time.
    pub clt: CycleTime,

    /// Spread spectrum range.
    pub rng: Range,

    /// Spread spectrum enable.
    pub ssp: bool,

    reserved: u1,

    /// Sync configure.
    pub sync: Sync,
}

impl Default for SpreadSpectrum {
    fn default() -> Self {
        Self::from(0b0000_0000)
    }
}

impl SpreadSpectrum {
    /// Configure spread spectrum cycle time.
    #[must_use]
    pub fn with_clt(mut self, clt: CycleTime) -> Self {
        self.set_clt(clt);
        Self::from(self.value)
    }

    /// Configure spread spectrum range.
    #[must_use]
    pub fn with_rng(mut self, rng: Range) -> Self {
        self.set_rng(rng);
        Self::from(self.value)
    }

    /// Configure spread spectrum enable.
    #[must_use]
    pub fn with_ssp(mut self, ssp: bool) -> Self {
        self.set_ssp(ssp);
        Self::from(self.value)
    }

    /// Configure sync enable.
    #[must_use]
    pub fn with_sync(mut self, sync: Sync) -> Self {
        self.set_sync(sync);
        Self::from(self.value)
    }
}

/// Spread spectrum cycle time setting.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum CycleTime {
    /// 1980 microseconds.
    U1980,

    /// 1200 microseconds.
    U1200,

    /// 820 microseconds.
    U820,

    /// 660 microseconds.
    U660,
}

/// Spread spectrum range setting.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Range {
    /// +/- 5% range.
    P5,

    /// +/- 15% range.
    P15,

    /// +/- 24% range.
    P24,

    /// +/- 34% range.
    P34,
}

/// Sync setting.
#[bitsize(2)]
#[derive(Debug, FromBits, PartialEq, PartialOrd)]
pub enum Sync {
    /// Disabled, 30k pull-down resistor.
    #[fallback]
    Disabled,

    /// Slave mode, clock input.
    Slave = 0b10,

    /// Master mode, clock output.
    Master,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_command() {
        let cmd = Command::default();
        assert_eq!(cmd.value, 0x50);
    }

    #[test]
    fn modify_page() {
        let cmd = Command::default().with_page(PageNumber::Pg1);
        assert_eq!(cmd.value, 0x51);
    }

    #[test]
    fn modify_rw() {
        let cmd = Command::default().with_rw(CommandType::Write);
        assert_eq!(cmd.value, 0x50);
    }

    #[test]
    fn default_configuration() {
        let cfg = Configuration::default();
        assert_eq!(cfg.value, 0x8);
    }

    #[test]
    fn modify_ssd() {
        let cfg = Configuration::default().with_ssd(true);
        assert_eq!(cfg.value, 0x9);
    }

    #[test]
    fn modify_osde() {
        let cfg = Configuration::default().with_osde(Open::EnableShort);
        assert_eq!(cfg.value, 0xC);
    }

    #[test]
    fn modify_sws() {
        let cfg = Configuration::default().with_sws(SwxSetting::Sw5);
        assert_eq!(cfg.value, 0x68);
    }

    #[test]
    fn default_pur_pdr() {
        let pur_pdr = PullDownUpResistorSelection::default();
        assert_eq!(pur_pdr.value, 0x33);
    }

    #[test]
    fn modify_cspur() {
        let pur_pdr = PullDownUpResistorSelection::default().with_cspur(Resistor::R2_0k);
        assert_eq!(pur_pdr.value, 0x35);
    }

    #[test]
    fn modify_swpdr() {
        let pur_pdr = PullDownUpResistorSelection::default().with_swpdr(Resistor::R8_0k);
        assert_eq!(pur_pdr.value, 0x73);
    }

    #[test]
    fn default_temperature() {
        let temperature = TemperatureStatus::default();
        assert_eq!(temperature.value, 0x0);
    }

    #[test]
    fn modify_trof() {
        let temperature = TemperatureStatus::default().with_trof(ThermalRollOff::P55);
        assert_eq!(temperature.value, 0x2);
    }

    #[test]
    fn modify_ts() {
        let temperature = TemperatureStatus::default().with_ts(TemperaturePoint::D120);
        assert_eq!(temperature.value, 0x4);
    }

    #[test]
    fn default_spread_spectrum() {
        let spread_spectrum = SpreadSpectrum::default();
        assert_eq!(spread_spectrum.value, 0x0);
    }

    #[test]
    fn modify_clt() {
        let spread_spectrum = SpreadSpectrum::default().with_clt(CycleTime::U660);
        assert_eq!(spread_spectrum.value, 0x3);
    }

    #[test]
    fn modify_rng() {
        let spread_spectrum = SpreadSpectrum::default().with_rng(Range::P15);
        assert_eq!(spread_spectrum.value, 0x4);
    }

    #[test]
    fn modify_ssp() {
        let spread_spectrum = SpreadSpectrum::default().with_ssp(true);
        assert_eq!(spread_spectrum.value, 0x10);
    }

    #[test]
    fn modify_sync() {
        let spread_spectrum = SpreadSpectrum::default().with_sync(Sync::Master);
        assert_eq!(spread_spectrum.value, 0xC0);
    }
}
