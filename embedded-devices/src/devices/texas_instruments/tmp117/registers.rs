use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub type TMP117I2cCodec = OneByteRegAddrCodec;

pub const DEVICE_ID_VALID: u16 = 0x117;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = TMP117I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ super::TMP117 ]

    /// Temperature alert flag.
    enum AlertFlag: u8{1} {
        /// The alert is unset.
        0 Cleared,
        /// The associated alert was triggered.
        1 Set,
    }

    /// Conversion mode.
    enum ConversionMode: u8{2} {
        /// Continuous conversion (power-up default)
        0b00 Continuous,
        /// Shutdown
        0b01 Shutdown,
        /// This is the same as Continuous and will read back as Continuous.
        0b10 Continuous2,
        /// Oneshot conversion
        0b11 Oneshot,
    }

    /// Conversion cycle time.
    #[allow(non_camel_case_types)]
    enum ConversionCycleTime: u8{3} {
        /// 15.5ms
        0b000 T_15_5,
        /// 125ms
        0b001 T_125,
        /// 250ms
        0b010 T_250,
        /// 500ms
        0b011 T_500,
        /// 1s (power-up default)
        0b100 T_1000,
        /// 4s
        0b101 T_4000,
        /// 8s
        0b110 T_8000,
        /// 16s
        0b111 T_16000,
    }

    /// Conversion averaging modes.
    #[allow(non_camel_case_types)]
    enum AveragingMode: u8{2} {
        /// Disables averaging.
        0b00 X_1,
        /// Configures 8x sample averaging. This is the power-on default.
        /// Minimum conversion time is 125ms, even if a lower conversion cycle is selected.
        0b01 X_8,
        /// Configures 32x sample averaging.
        /// Minimum conversion time is 500ms, even if a lower conversion cycle is selected.
        0b10 X_32,
        /// Configures 64x sample averaging.
        /// Minimum conversion time is 1s, even if a lower conversion cycle is selected.
        0b11 X_64,
    }

    /// Therm/Alert mode.
    enum AlertThermMode: u8{1} {
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
        0 Alert,
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
        1 Therm,
    }

    /// Alert pin polarity.
    enum AlertPinPolarity: u8{1} {
        /// Power-on default.
        0 ActiveLow,
        1 ActiveHigh,
    }

    /// Alert pin mode.
    enum AlertPinMode: u8{1} {
        /// Alert pin reflects the status of the alert flags. (Set if any alert is set).
        0 AnyAlert,
        /// Alert pin reflects the status of the data-ready flag.
        1 DataReady,
    }

    /// EEPROM lock mode.
    enum EepromLockMode: u8{1} {
        /// EEPROM is locked for programming: writes to all EEPROM addresses
        /// (such as configuration, limits, and EEPROM locations 1-4) are written
        /// to registers in digital logic and are not programmed in the EEPROM
        0 Locked,
        /// EEPROM unlocked for programming: any writes to programmable registers
        /// program the respective location in the EEPROM
        1 Unlocked,
    }

    /// At the end of every conversion, the device updates the temperature
    /// register with the conversion result. Following a reset, the temperature
    /// register reads –256°C until the first conversion, including averaging, is complete.
    register Temperature(addr = 0b0000, mode = r, size = 2) {
        /// The temperature in °C with a resolution of 7.8125m°C/LSB.
        raw_temperature: i16 = i16::MIN => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// The device configuration register.
    register Configuration(addr = 0b0001, mode = rw, size = 2) {
        /// Set when the conversion result is higher than the high limit.
        /// This flag is cleared on read except in Therm mode, where it is
        /// cleared when the conversion result is lower than the hysteresis.
        alert_high: AlertFlag = AlertFlag::Cleared,
        /// Set when the conversion result is lower than the low limit.
        /// In Therm mode, this flag is always cleared.
        /// This flag is cleared on read.
        alert_low: AlertFlag = AlertFlag::Cleared,
        /// This flag indicates that the conversion is complete and the
        /// temperature register can be read. Every time the temperature
        /// register or configuration register is read, this bit is cleared. This
        /// bit is set at the end of the conversion when the temperature
        /// register is updated. Data ready can be monitored on the ALERT
        /// pin by setting the alert_pin_mode to AlertPinMode::DataReady.
        data_ready: bool = false,
        /// The value of the flag indicates that the EEPROM is busy during programming or power-up.
        eeprom_busy: bool = false,
        /// Temperature conversion mode (operating mode).
        /// This can be persisted in the EEPROM to set the power-up default,
        /// except for the oneshot mode.
        conversion_mode: ConversionMode = ConversionMode::Continuous,
        /// Amount of time to wait between conversions.
        /// If the time to complete the conversions needed for a given averaging setting is
        /// higher than the conversion setting cycle time, there will be no stand by time
        /// in the conversion cycle. See AveragingMode.
        /// This bit can be persisted into the EEPROM.
        /// This can be persisted in the EEPROM to set the power-up default.
        conversion_cycle_time: ConversionCycleTime = ConversionCycleTime::T_1000,
        /// Determines the number of conversion results that are collected
        /// and averaged before updating the temperature register.
        /// The average is an accumulated average and not a running average.
        /// This can be persisted in the EEPROM to set the power-up default.
        averaging_mode: AveragingMode = AveragingMode::X_8,
        /// Therm/alert mode select.
        /// This can be persisted in the EEPROM to set the power-up default.
        alert_therm_mode: AlertThermMode = AlertThermMode::Alert,
        /// Polarity of the alert pin.
        /// This can be persisted in the EEPROM to set the power-up default.
        alert_pin_polarity: AlertPinPolarity = AlertPinPolarity::ActiveLow,
        /// Alert pin output mode.
        /// This can be persisted in the EEPROM to set the power-up default.
        alert_pin_mode: AlertPinMode = AlertPinMode::AnyAlert,
        /// Triggers a soft-reset when set. Always reads back as `false`.
        soft_reset: bool = false,
        /// Reserved bit
        _: u8{1},
    }

    /// This register stores the high limit for comparison with the temperature result
    /// with a resolution is 7.8125m°C/LSB. The range of the register is ±256°C.
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

    /// The EEPROM unlock register.
    register EepromUnlock(addr = 0b0100, mode = rw, size = 2) {
        /// EEPROM lock mode. Defaults to locked on power-on.
        lock_mode: EepromLockMode = EepromLockMode::Locked,
        /// This flag is the mirror of the EEPROM busy flag (bit 12) in the configuration register.
        /// - `false` indicates that the EEPROM is ready, which means that the EEPROM has finished
        ///   the last transaction and is ready to accept new commands.
        /// - `true` indicates that the EEPROM is busy, which means that the EEPROM is currently
        ///    completing a programming operation or performing power-up on reset load
        busy: bool = false,
        /// Reserved bits
        _: u16{14},
    }

    /// This is a 16-bit register that be used as a scratch pad by the customer to store general-
    /// purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
    /// locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
    /// causes the corresponding EEPROM location to be programmed.
    ///
    /// To support NIST traceability do not delete or reprogram the EEPROM1 register.
    register Eeprom1(addr = 0b0101, mode = rw, size = 2) {
        /// The data stored in this register
        data: [u8; 2] = [0u8; 2],
    }

    /// This is a 16-bit register that be used as a scratch pad by the customer to store general-
    /// purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
    /// locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
    /// causes the corresponding EEPROM location to be programmed.
    ///
    /// To support NIST traceability do not delete or reprogram the EEPROM2 register.
    register Eeprom2(addr = 0b0110, mode = rw, size = 2) {
        /// The data stored in this register
        data: [u8; 2] = [0u8; 2],
    }

    /// This register may be used as a user-defined temperature offset register during system calibration.
    /// The offset will be added to the temperature result after linearization. It has a same resolution
    /// of 7.8125 m°C/LSB and same range of ±256°C as the temperature result register. If the added result
    /// exceeds value boundaries, then the temperature result will clamp to the maximum or minimum value.
    register TemperatureOffset(addr = 0b0111, mode = rw, size = 2) {
        /// The temperature offset in °C with a resolution of 7.8125m°C/LSB.
        raw_temperature_offset: i16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// This is a 16-bit register that be used as a scratch pad by the customer to store general-
    /// purpose data. This register has a corresponding EEPROM location. Writes to this address when the EEPROM is
    /// locked write data into the register and not to the EEPROM. Writes to this register when the EEPROM is unlocked
    /// causes the corresponding EEPROM location to be programmed.
    ///
    /// To support NIST traceability do not delete or reprogram the EEPROM3 register.
    register Eeprom3(addr = 0b1000, mode = rw, size = 2) {
        /// The data stored in this register
        data: [u8; 2] = [0u8; 2],
    }

    /// The device-id and revision register.
    register DeviceIdRevision(addr = 0b1111, mode = r, size = 2) {
        /// Indicates the revision number of the device. 0 indicates the first revision.
        device_revision: u8{4} = 0x0,
        /// The indentifier of this this device. The factory default is DEVICE_ID_VALID (0x117).
        device_id: u16{12} = 0x117,
    }
}

impl ConversionCycleTime {
    /// Conversion cycle time in microseconds.
    pub fn interval_us(&self) -> u32 {
        match self {
            ConversionCycleTime::T_15_5 => 15_500,
            ConversionCycleTime::T_125 => 125_000,
            ConversionCycleTime::T_250 => 250_000,
            ConversionCycleTime::T_500 => 500_000,
            ConversionCycleTime::T_1000 => 1_000_000,
            ConversionCycleTime::T_4000 => 4_000_000,
            ConversionCycleTime::T_8000 => 8_000_000,
            ConversionCycleTime::T_16000 => 16_000_000,
        }
    }
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
