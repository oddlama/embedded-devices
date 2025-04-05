use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub const DEVICE_ID_VALID: u16 = 0x117;

/// At the end of every conversion, the device updates the temperature
/// register with the conversion result. Following a reset, the temperature
/// register reads –256°C until the first conversion, including averaging, is complete.
#[device_register(super::TMP117)]
#[register(address = 0b0000, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Temperature {
    /// The temperature in °C with a resolution of 7.8125m°C/LSB.
    #[register(default = i16::MIN)]
    pub raw_temperature: i16,
}

impl Temperature {
    /// Reads the ambient temperature in °C with a resolution of 7.8125m°C/LSB.
    pub fn read_temperature(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(self.read_raw_temperature().into(), 128))
    }
}

/// Temperature alert flag.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertFlag {
    /// The alert is unset.
    Cleared = 0,
    /// The associated alert was triggered.
    Set = 1,
}

/// Conversion mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ConversionMode {
    /// Continuous conversion (power-up default)
    Continuous = 0b00,
    /// Shutdown
    Shutdown = 0b01,
    // This is the same as [`Continuous`] and will read back as [`Continuous`].
    Continuous2 = 0b10,
    /// Oneshot conversion
    Oneshot = 0b11,
}

/// Conversion cycle time.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
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
    T_1000 = 0b100,
    /// 4s
    T_4000 = 0b101,
    /// 8s
    T_8000 = 0b110,
    /// 16s
    T_16000 = 0b111,
}

/// Conversion averaging modes.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum AveragingMode {
    /// Disables averaging.
    X_1 = 0b00,
    /// Configures 8x sample averaging. This is the power-on default.
    /// Minimum conversion time is 125ms, even if a lower conversion cycle is selected.
    X_8 = 0b01,
    /// Configures 32x sample averaging.
    /// Minimum conversion time is 500ms, even if a lower conversion cycle is selected.
    X_32 = 0b10,
    /// Configures 64x sample averaging.
    /// Minimum conversion time is 1s, even if a lower conversion cycle is selected.
    X_64 = 0b11,
}

impl AveragingMode {
    /// Returns the averaging factor
    pub fn factor(&self) -> u8 {
        match self {
            AveragingMode::X_1 => 1,
            AveragingMode::X_8 => 8,
            AveragingMode::X_32 => 32,
            AveragingMode::X_64 => 64,
        }
    }
}

/// Therm/Alert mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertThermMode {
    /// In this mode, the device compares the conversion result at the end of every
    /// conversion with the values in the low limit register and high limit register.
    /// If the temperature result exceeds the value in the high limit register,
    /// the high alert status flag in the configuration register is set. On the other hand,
    /// if the temperature result is lower than the value in the low limit register,
    /// the low alert status flag in the configuration register is set.
    /// This is the power-on default.
    ///
    /// This mode effectively makes the device behave like a window limit detector.
    /// Thus this mode can be used in applications where detecting if the temperature
    /// goes outside of the specified range is necessary.
    Alert = 0,
    /// In this mode, the device compares the conversion result at the end of every conversion
    /// with the values in the low limit register and high limit register and sets the high alert
    /// status flag in the configuration register if the temperature exceeds the value in the
    /// high limit register. When set, the device clears the high alert status flag if the conversion
    /// result goes below the value in the low limit register.
    ///
    /// Thus, the difference between the high and low limits effectively acts like a hysteresis.
    /// In this mode, the low alert status flag is disabled and always reads 0. Unlike the alert
    /// mode, I2C reads of the configuration register do not affect the status bits. The high alert
    /// status flag is only set or cleared at the end of conversions based on the value of the
    /// temperature result compared to the high and low limits.
    Therm = 1,
}

/// Alert pin polarity.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPinPolarity {
    /// Power-on default.
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Alert pin mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPinMode {
    /// Alert pin reflects the status of the alert flags. (Set if any alert is set).
    AnyAlert = 0,
    /// Alert pin reflects the status of the data-ready flag.
    DataReady = 1,
}

/// The device configuration register.
#[device_register(super::TMP117)]
#[register(address = 0b0001, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    /// Set when the conversion result is higher than the high limit.
    /// This flag is cleared on read except in Therm mode, where it is
    /// cleared when the conversion result is lower than the hysteresis.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertFlag::Cleared)]
    pub alert_high: AlertFlag,
    /// Set when the conversion result is lower than the low limit.
    /// In Therm mode, this flag is always cleared.
    /// This flag is cleared on read.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertFlag::Cleared)]
    pub alert_low: AlertFlag,
    /// This flag indicates that the conversion is complete and the
    /// temperature register can be read. Every time the temperature
    /// register or configuration register is read, this bit is cleared. This
    /// bit is set at the end of the conversion when the temperature
    /// register is updated. Data ready can be monitored on the ALERT
    /// pin by setting the [`Self::alert_pin_mode`] to [`AlertPinMode::DataReady`].
    #[register(default = false)]
    pub data_ready: bool,
    /// The value of the flag indicates that the EEPROM is busy during programming or power-up.
    #[register(default = false)]
    pub eeprom_busy: bool,
    /// Temperature conversion mode (operating mode).
    /// This can be persisted in the EEPROM to set the power-up default,
    /// except for the oneshot mode.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = ConversionMode::Continuous)]
    pub conversion_mode: ConversionMode,
    /// Amount of time to wait between conversions.
    /// If the time to complete the conversions needed for a given averaging setting is
    /// higher than the conversion setting cycle time, there will be no stand by time
    /// in the conversion cycle. See [`AveragingMode`].
    /// This bit can be persisted into the EEPROM.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionCycleTime::T_1000)]
    pub conversion_cycle_time: ConversionCycleTime,
    /// Determines the number of conversion results that are collected
    /// and averaged before updating the temperature register.
    /// The average is an accumulated average and not a running average.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = AveragingMode::X_8)]
    pub averaging_mode: AveragingMode,
    /// Therm/alert mode select.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertThermMode::Alert)]
    pub alert_therm_mode: AlertThermMode,
    /// Polarity of the alert pin.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertPinPolarity::ActiveLow)]
    pub alert_pin_polarity: AlertPinPolarity,
    /// Alert pin output mode.
    /// This can be persisted in the EEPROM to set the power-up default.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertPinMode::AnyAlert)]
    pub alert_pin_mode: AlertPinMode,
    /// Triggers a soft-reset when set. Always reads back as `false`.
    #[register(default = false)]
    pub soft_reset: bool,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

macro_rules! define_temp_limit_register {
    ($name:ident, $address:expr, $value_default:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::TMP117)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            /// The temperature limit in °C with a resolution of 7.8125m°C/LSB.
            #[register(default = $value_default)]
            pub raw_temperature_limit: i16,
        }

        impl $name {
            /// Reads the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
                ThermodynamicTemperature::new::<degree_celsius>(
                    Rational32::new_raw(self.read_raw_temperature_limit().into(), 128).reduced(),
                )
            }

            /// Writes the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            /// The passed temperature will be truncated (rounded down).
            pub fn write_temperature_limit(
                &mut self,
                temperature_limit: ThermodynamicTemperature,
            ) -> Result<(), core::num::TryFromIntError> {
                let temp = temperature_limit.get::<degree_celsius>();
                let temp: i16 = (temp * Rational32::from_integer(128)).to_integer().try_into()?;
                self.write_raw_temperature_limit(temp);
                Ok(())
            }

            /// Writes the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            /// The passed temperature will be truncated (rounded down).
            pub fn with_temperature_limit(
                mut self,
                temperature_limit: ThermodynamicTemperature,
            ) -> Result<Self, core::num::TryFromIntError> {
                self.write_temperature_limit(temperature_limit)?;
                Ok(self)
            }
        }
    };
}

define_temp_limit_register!(
    TemperatureLimitHigh,
    0b0010,
    0x6000i16,
    r#"
This register stores the high limit for comparison with the temperature result
with a resolution is 7.8125m°C/LSB. The range of the register is ±256°C.
Following power-up or a general-call reset, the high-limit register is loaded with the
stored value from the EEPROM. The factory default reset value is 6000h (192°C).
"#
);

define_temp_limit_register!(
    TemperatureLimitLow,
    0b0011,
    i16::MIN,
    r#"
This register stores the low limit for comparison with the temperature result
with a resolution is 7.8125m°C/LSB. The range of the register is ±256°C.
Following power-up or a general-call reset, the low-limit register is loaded with the
stored value from the EEPROM. The factory default reset value is 8000h (-256°C).
"#
);

/// EEPROM lock mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum EepromLockMode {
    /// EEPROM is locked for programming: writes to all EEPROM addresses
    /// (such as configuration, limits, and EEPROM locations 1-4) are written
    /// to registers in digital logic and are not programmed in the EEPROM
    Locked = 0,
    /// EEPROM unlocked for programming: any writes to programmable registers
    /// program the respective location in the EEPROM
    Unlocked = 1,
}

/// The EEPROM unlock register.
#[device_register(super::TMP117)]
#[register(address = 0b0100, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct EepromUnlock {
    /// EEPROM lock mode. Defaults to locked on power-on.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = EepromLockMode::Locked)]
    pub lock_mode: EepromLockMode,
    /// This flag is the mirror of the EEPROM busy flag (bit 12) in the configuration register.
    /// - `false` indicates that the EEPROM is ready, which means that the EEPROM has finished
    ///   the last transaction and is ready to accept new commands.
    /// - `true` indicates that the EEPROM is busy, which means that the EEPROM is currently
    ///    completing a programming operation or performing power-up on reset load
    #[register(default = false)]
    pub busy: bool,
    #[bondrewd(bit_length = 14, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
}

macro_rules! define_eeprom_register {
    ($name:ident, $address:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::TMP117)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            /// The data stored in this register
            pub data: [u8; 2],
        }
    };
}

define_eeprom_register!(
    Eeprom1,
    0b0101,
    r#"
This is a 16-bit register that be used as a scratch pad by the customer to store general-
purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
causes the corresponding EEPROM location to be programmed.

To support NIST traceability do not delete or reprogram the EEPROM1 register.
"#
);

define_eeprom_register!(
    Eeprom2,
    0b0110,
    r#"
This is a 16-bit register that be used as a scratch pad by the customer to store general-
purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
causes the corresponding EEPROM location to be programmed.

To support NIST traceability do not delete or reprogram the EEPROM2 register.
"#
);

define_eeprom_register!(
    Eeprom3,
    0b1000,
    r#"
This is a 16-bit register that be used as a scratch pad by the customer to store general-
purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
causes the corresponding EEPROM location to be programmed.

To support NIST traceability do not delete or reprogram the EEPROM3 register.
"#
);

/// This register may be used as a user-defined temperature offset register during system calibration.
/// The offset will be added to the temperature result after linearization. It has a same resolution
/// of 7.8125 m°C/LSB and same range of ±256°C as the temperature result register. If the added result
/// exceeds value boundaries, then the temperature result will clamp to the maximum or minimum value.
#[device_register(super::TMP117)]
#[register(address = 0b0111, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct TemperatureOffset {
    /// The temperature offset in °C with a resolution of 7.8125m°C/LSB.
    #[register(default = 0x0000)]
    pub raw_temperature_offset: i16,
}

impl TemperatureOffset {
    /// Reads the temperature offset in °C with a resolution of 7.8125m°C/LSB.
    pub fn read_temperature_offset(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(
            Rational32::new_raw(self.read_raw_temperature_offset().into(), 128).reduced(),
        )
    }

    /// Writes the temperature offset in °C with a resolution of 7.8125m°C/LSB.
    /// The passed temperature will be truncated (rounded down).
    pub fn write_temperature_offset(
        &mut self,
        temperature_offset: ThermodynamicTemperature,
    ) -> Result<(), core::num::TryFromIntError> {
        let temp = temperature_offset.get::<degree_celsius>();
        let temp: i16 = (temp * Rational32::from_integer(128)).to_integer().try_into()?;
        self.write_raw_temperature_offset(temp);
        Ok(())
    }
}

/// The device-id and revision register.
#[device_register(super::TMP117)]
#[register(address = 0b1111, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceIdRevision {
    /// Indicates the revision number of the device. 0 indicates the first revision.
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x0)]
    pub device_revision: u8,
    /// The indentifier of this this device. The factory default is [`DEVICE_ID_VALID`] (`0x117`).
    #[bondrewd(bit_length = 12)]
    #[register(default = 0x117)]
    pub device_id: u16,
}
