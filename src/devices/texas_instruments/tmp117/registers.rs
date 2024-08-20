use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub const DEVICE_ID_VALID: u8 = 0x04;
pub const MANUFACTURER_ID_VALID: u16 = 0x0054;

/// At the end of every conversion, the device updates the temperature
/// register with the conversion result. Following a reset, the temperature
/// register reads –256 °C until the first conversion, including averaging, is complete.
#[device_register(super::TMP117)]
#[register(address = 0b00, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Temperature {
    /// The temperature in °C with a resolution of 0.0078125°C/LSB.
    pub raw_temperature: i16,
}

impl Temperature {
    /// Reads the ambient temperature in °C with a resolution of 0.0078125°C/LSB.
    pub fn read_temperature(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(self.read_raw_temperature().into(), 128))
    }
}

/// Temperature alert flag.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertFlag {
    /// The alert is unset.
    #[default]
    Cleared = 0,
    /// The associated alert was triggered.
    Set = 1,
}

/// Conversion mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ConversionMode {
    /// Continuous conversion (power-up default)
    #[default]
    Continuous = 0b00,
    /// Shutdown
    Shutdown = 0b01,
    // This is the same as [`Continuous`] and will read back as [`Continuous`].
    Continuous2 = 0b10,
    /// Oneshot conversion
    Oneshot = 0b11,
}

/// Conversion cycle time.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum ConversionCycleTime {
    /// 15.5ms
    T_0015_5 = 0b000,
    /// 125ms
    T_0125 = 0b001,
    /// 250ms
    T_0250 = 0b010,
    /// 500ms
    T_0500 = 0b011,
    /// 1s (power-up default)
    #[default]
    T_1000 = 0b100,
    /// 4s
    T_4000 = 0b101,
    /// 8s
    T_8000 = 0b110,
    /// 16s
    T_16000 = 0b111,
}

/// Conversion averaging modes.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum AveragingMode {
    /// Disables averaging.
    X_1 = 0b00,
    /// Configures 8x sample averaging. This is the power-on default.
    #[default]
    X_8 = 0b01,
    /// Configures 32x sample averaging.
    X_32 = 0b10,
    /// Configures 64x sample averaging.
    X_64 = 0b11,
}

/// Therm/Alert mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertThermMode {
    /// Power-on default.
    #[default]
    Alert = 0,
    Therm = 1,
}

/// Alert pin polarity.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPinPolarity {
    /// Power-on default.
    #[default]
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Alert pin mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPinMode {
    /// Alert pin reflects the status of the alert flags. (Set if any alert is set).
    #[default]
    AnyAlert = 0,
    /// Alert pin reflects the status of the data-ready flag.
    DataReady = 1,
}

/// The device configuration register.
#[device_register(super::TMP117)]
#[register(address = 0b01, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    /// Set when the conversion result is higher than the high limit.
    /// This flag is cleared on read except in Therm mode, where it is
    /// cleared when the conversion result is lower than the hysteresis.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_high: AlertFlag,
    /// Set when the conversion result is lower than the low limit.
    /// In Therm mode, this flag is always cleared.
    /// This flag is cleared on read.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_low: AlertFlag,
    /// This flag indicates that the conversion is complete and the
    /// temperature register can be read. Every time the temperature
    /// register or configuration register is read, this bit is cleared. This
    /// bit is set at the end of the conversion when the temperature
    /// register is updated. Data ready can be monitored on the ALERT
    /// pin by setting the [`Self::alert_pin_mode`] to [`AlertPinMode::DataReady`].
    pub data_ready: bool,
    /// The value of the flag indicates that the EEPROM is busy during programming or power-up.
    pub eeprom_busy: bool,
    /// Temperature conversion mode (operating mode).
    /// This can be persisted in the EEPROM to set the power-up default,
    /// except for the oneshot mode.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub conversion_mode: ConversionMode,
    /// Amount of time to wait between conversions.
    /// If the time to complete the conversions needed for a given averaging setting is
    /// higher than the conversion setting cycle time, there will be no stand by time
    /// in the conversion cycle.
    /// This bit can be persisted into the EEPROM.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub conversion_cycle_time: ConversionCycleTime,
    /// Determines the number of conversion results that are collected
    /// and averaged before updating the temperature register.
    /// The average is an accumulated average and not a running average.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub averaging_mode: AveragingMode,
    /// Therm/alert mode select.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_therm_mode: AlertThermMode,
    /// Polarity of the alert pin.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_pin_polarity: AlertPinPolarity,
    /// Alert pin output mode.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_pin_mode: AlertPinMode,
    /// Triggers a soft-reset when set. Always reads back as `false`.
    pub soft_reset: bool,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

// FIXME: todo device.persist::<R>() that writes eeprom, waits, checks ready, resets.
