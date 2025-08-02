use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub type TMP102I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = TMP102I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ super::TMP102 ]

    /// Temperature alert flag
    enum AlertFlag: u8{1} {
        /// The alert is unset
        0 Cleared,
        /// The associated alert was triggered
        1 Set,
    }

    /// Conversion cycle time
    #[allow(non_camel_case_types)]
    enum ConversionCycleTime: u8{2} {
        /// 4s
        0b00 T_4000,
        /// 1s
        0b01 T_1000,
        /// 250ms
        0b10 T_250,
        /// 125ms
        0b11 T_125,
    }

    /// Therm/Alert mode
    enum InterruptThermMode: u8{1} {
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
        0 Therm,
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
        1 Interrupt,
    }

    /// Alert pin polarity
    enum AlertPinPolarity: u8{1} {
        /// Power-on default
        0 ActiveLow,
        1 ActiveHigh,
    }

    /// Fault Queue
    #[allow(non_camel_case_types)]
    enum FaultQueue: u8{2} {
        /// Alert is triggered at the first event
        0b00 F_1,
        /// Two consecutive events are required to set the Alert
        0b01 F_2,
        /// Four consecutive events are required to set the Alert
        0b10 F_4,
        /// Six consecutive events are required to set the Alert
        0b11 F_6,
    }

    /// Resolution. The TMP102 only supports 12 bits resolution. Read-only.
    #[allow(non_camel_case_types)]
    enum Resolution: u8{2} {
        /// 12 bits resolution
        0b11 R_12,
        _ Invalid,
    }

    /// At the end of every conversion, the device updates the temperature
    /// register with the conversion result.
    register Temperature(addr = 0b0000, mode = r, size = 2) {
        /// The temperature in °C with a resolution of 7.8125m°C/LSB.
        raw_temperature: i16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// The device configuration register
    register Configuration(addr = 0b0001, mode = rw, size = 2) {
        oneshot: bool = false,
        resolution: Resolution = Resolution::R_12,
        fault_queue: FaultQueue = FaultQueue::F_1,
        alert_polarity: AlertPinPolarity = AlertPinPolarity::ActiveLow,
        alert_mode: InterruptThermMode = InterruptThermMode::Therm,
        shutdown: bool = false,
        /// Amount of time to wait between conversions.
        conversion_cycle_time: ConversionCycleTime = ConversionCycleTime::T_250,
        /// Set when the conversion result is higher than the high limit.
        /// This flag is cleared on read except in Therm mode, where it is
        /// cleared when the conversion result is lower than the hysteresis.
        alert: AlertFlag = AlertFlag::Cleared,
        extended: bool = false,
        /// Reserved bits
        _: u8{4},
    }

    /// This register stores the high limit for comparison with the temperature result
    /// with a resolution is 7.8125m°C/LSB (but the ). The range of the register is ±256°C.
    /// Following power-up or a general-call reset, the high-limit register is loaded with the
    /// stored value from the EEPROM. The factory default reset value is 6000h (192°C).
    register TemperatureLimitHigh(addr = 0b0010, mode = rw, size = 2) {
        /// The temperature limit in °C with a resolution of 7.8125m°C/LSB.
        raw_temperature_limit: i16 = 0x6000 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// This register stores the low limit for comparison with the temperature result
    /// with a resolution is 7.8125m°C/LSB. The range of the register is ±256°C.
    /// Following power-up or a general-call reset, the low-limit register is loaded with the
    /// stored value from the EEPROM. The factory default reset value is 8000h (-256°C).
    register TemperatureLimitLow(addr = 0b0011, mode = rw, size = 2) {
        /// The temperature limit in °C with a resolution of 7.8125m°C/LSB.
        raw_temperature_limit: i16 = i16::MIN => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }
}

impl ConversionCycleTime {
    /// Returns the conversion time in milliseconds
    pub fn conversion_time_ms(&self) -> u32 {
        match self {
            ConversionCycleTime::T_4000 => 4000,
            ConversionCycleTime::T_1000 => 1000,
            ConversionCycleTime::T_250 => 250,
            ConversionCycleTime::T_125 => 125,
        }
    }
}
