use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::i2c::codecs::OneByteRegAddrCodec;
use embedded_registers::register;
use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

/// At the end of every conversion, the device updates the temperature
/// register with the conversion result.
#[device_register(super::TMP102)]
#[register(address = 0b0000, mode = "r", i2c_codec = "OneByteRegAddrCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Temperature {
    /// The temperature in °C with a resolution of 7.8125m°C/LSB.
    pub raw_temperature: i16,
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

/// Conversion cycle time.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum ConversionCycleTime {
    /// 4s
    T_4000 = 0b00,
    /// 1s
    T_1000 = 0b01,
    /// 250ms
    T_250 = 0b10,
    /// 125ms
    T_125 = 0b11,
}
impl ConversionCycleTime {
    /// Returns the averaging factor
    pub fn conversion_time_ms(&self) -> u32 {
        match self {
            ConversionCycleTime::T_4000 => 4000,
            ConversionCycleTime::T_1000 => 1000,
            ConversionCycleTime::T_250 => 250,
            ConversionCycleTime::T_125 => 125,
        }
    }
}

/// Therm/Alert mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum InterruptThermMode {
    /// In this mode, the device compares the conversion result at the end of every conversion
    /// with the values in the low limit register and high limit register and sets the high alert
    /// status flag in the configuration register if the temperature exceeds the value in the
    /// high limit register. When set, the device clears the high alert status flag if the conversion
    /// result goes below the value in the low limit register.
    /// This is the power-on default.
    ///
    /// Thus, the difference between the high and low limits effectively acts like a hysteresis.
    /// In this mode, the low alert status flag is disabled and always reads 0. Unlike the alert
    /// mode, I2C reads of the configuration register do not affect the status bits. The high alert
    /// status flag is only set or cleared at the end of conversions based on the value of the
    /// temperature result compared to the high and low limits.
    Therm = 0,
    /// In this mode, the device compares the conversion result at the end of every
    /// conversion with the values in the low limit register and high limit register.
    /// If the temperature result exceeds the value in the high limit register,
    /// the high alert status flag in the configuration register is set. On the other hand,
    /// if the temperature result is lower than the value in the low limit register,
    /// the alert status flag in the configuration register is set. It is cleared on temperature
    /// register read.
    ///
    /// This mode effectively makes the device behave like a window limit detector.
    /// Thus this mode can be used in applications where detecting if the temperature
    /// goes outside of the specified range is necessary.
    Interrupt = 1,
}

/// Alert pin polarity.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPinPolarity {
    /// Power-on default.
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Fault Queue
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum FaultQueue {
    /// Alert is triggered at the first event
    F_1 = 0b00,
    /// Two consecutive events are required to set the Alert
    F_2 = 0b01,
    /// Four consecutive events are required to set the Alert
    F_4 = 0b10,
    /// Six consecutive events are required to set the Alert
    F_6 = 0b11,
}

/// Resolution. The TMP102 only supports 12 bits resolution. Read-only.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum Resolution {
    /// 12 bits resolution
    R_12 = 0b11,
}

/// The device configuration register.
#[device_register(super::TMP102)]
#[register(address = 0b0001, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    #[register(default = false)]
    pub oneshot: bool,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = Resolution::R_12)]
    pub resolution: Resolution,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FaultQueue::F_1)]
    pub fault_queue: FaultQueue,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertPinPolarity::ActiveLow)]
    pub alert_polarity: AlertPinPolarity,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = InterruptThermMode::Therm)]
    pub alert_mode: InterruptThermMode,
    #[register(default = false)]
    pub shutdown: bool,
    /// Amount of time to wait between conversions.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = ConversionCycleTime::T_250)]
    pub conversion_cycle_time: ConversionCycleTime,
    /// Set when the conversion result is higher than the high limit.
    /// This flag is cleared on read except in Therm mode, where it is
    /// cleared when the conversion result is lower than the hysteresis.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertFlag::Cleared)]
    pub alert: AlertFlag,
    #[register(default = false)]
    pub extended: bool,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    #[register(default = 0)]
    pub reserved: u8,
}

macro_rules! define_temp_limit_register {
    ($name:ident, $address:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::TMP102)]
        #[register(address = $address, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            /// The temperature limit in °C with a resolution of 7.8125m°C/LSB.
            pub raw_temperature_limit: i16,
        }

        impl $name {
            /// Reads the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
                ThermodynamicTemperature::new::<degree_celsius>(self.read_raw_temperature_limit() as f64 / 128.0)
            }

            /// Writes the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            /// The passed temperature will be truncated (rounded down).
            pub fn write_temperature_limit(
                &mut self,
                temperature_limit: ThermodynamicTemperature,
            ) -> Result<(), core::num::TryFromIntError> {
                let temp = temperature_limit.get::<degree_celsius>();
                let temp: i16 = ((temp * 128.0) as i32).try_into()?;
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
    r#"
This register stores the high limit for comparison with the temperature result
with a resolution is 7.8125m°C/LSB (but the ). The range of the register is ±256°C.
Following power-up or a general-call reset, the high-limit register is loaded with the
stored value from the EEPROM. The factory default reset value is 6000h (192°C).
"#
);

define_temp_limit_register!(
    TemperatureLimitLow,
    0b0011,
    r#"
This register stores the low limit for comparison with the temperature result
with a resolution is 7.8125m°C/LSB. The range of the register is ±256°C.
Following power-up or a general-call reset, the low-limit register is loaded with the
stored value from the EEPROM. The factory default reset value is 8000h (-256°C).
"#
);
