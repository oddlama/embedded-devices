use crate::devices::microchip::mcp9808::MCP9808Register;
use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub type MCP9808I2cCodec = OneByteRegAddrCodec;

pub const DEVICE_ID_VALID: u8 = 0x04;
pub const MANUFACTURER_ID_VALID: u16 = 0x0054;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = MCP9808I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ MCP9808 ]

    /// T_{UPPER} and T_{LOWER} Limit Hysteresis bits.
    ///
    /// This cannot be altered when either of the lock bits are set (CriticalLock::Locked / WindowLock::Locked).
    /// This bit can be programmed in Shutdown mode.
    #[allow(non_camel_case_types)]
    enum Hysteresis: u8{2} {
        /// 0.0°C (power-up default)
        0b00 Deg_0_0C,
        /// 1.5°C
        0b01 Deg_1_5C,
        /// 3.0°C
        0b10 Deg_3_0C,
        /// 6.0°C
        0b11 Deg_6_0C,
    }

    /// Shutdown mode bit.
    /// In shutdown, all power-consuming activities are disabled, though all registers can be written to or read.
    ///
    /// This cannot be set to `1` when either of the lock bits are set (CriticalLock::Locked / WindowLock::Locked).
    /// However, it can be cleared to `0` for continuous conversion while locked.
    enum ShutdownMode: u8{1} {
        /// Continuous conversion (power-up default)
        0 Continuous,
        /// Shutdown (Low-Power mode)
        1 Shutdown,
    }

    /// T_CRIT lock bit.
    ///
    /// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
    enum CriticalLock: u8{1} {
        /// T_CRIT register can be written (power-up default)
        0 Unlocked,
        /// T_CRIT register cannot be written
        1 Locked,
    }

    /// T_UPPER and T_LOWER Window Lock bit.
    ///
    /// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
    enum WindowLock: u8{1} {
        /// T_UPPER and T_LOWER registers can be written (power-up default)
        0 Unlocked,
        /// T_UPPER and T_LOWER registers cannot be written
        1 Locked,
    }

    /// Interrupt Clear bit.
    ///
    /// This bit cannot be set to `1` in Shutdown mode,
    /// but it can be cleared after the device enters Shutdown Mode.
    enum InterruptClear: u8{1} {
        /// No effect (power-up default)
        0 NoEffect,
        /// Clear interrupt output. When read, this bit returns to `0` (InterruptClear::NoEffect)
        1 ClearInterruptOutput,
    }

    /// Alert Output Status bit.
    ///
    /// This bit cannot be changed in Shutdown mode.
    /// However, if the Alert output is configured as Interrupt mode, and the host controller
    /// clears the interrupt bit by reading InterruptClear in Shutdown mode,
    /// then this bit will also be cleared to `0` AlertStatus::NotAsserted.
    enum AlertStatus: u8{1} {
        /// Alert output is not asserted by the device (power-up default)
        0 NotAsserted,
        /// Alert output is asserted as a comparator/Interrupt or critical temperature output
        1 Asserted,
    }

    /// Alert Output Control bit.
    ///
    /// This cannot be altered when either of the lock bits are set (CriticalLock::Locked / WindowLock::Locked).
    /// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
    enum AlertControl: u8{1} {
        /// power-up default
        0 Disabled,
        1 Enabled,
    }

    /// Alert Output Select bit.
    ///
    /// This cannot be altered when the window lock bit is set (WindowLock::Locked).
    /// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
    enum AlertSelect: u8{1} {
        /// Alert output for T_UPPER, T_LOWER and T_CRIT (power-up default)
        0 All,
        /// T_A > T CRIT only (T_UPPER and T_LOWER temperature boundaries are disabled)
        1 TCritOnly,
    }

    /// Alert Output Polarity bit.
    ///
    /// This cannot be altered when either of the lock bits are set (CriticalLock::Locked / WindowLock::Locked).
    /// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
    enum AlertPolarity: u8{1} {
        /// power-up default; pull-up resistor required
        0 ActiveLow,
        1 ActiveHigh,
    }

    /// Alert Output Mode bit.
    ///
    /// This cannot be altered when either of the lock bits are set (CriticalLock::Locked / WindowLock::Locked).
    /// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
    enum AlertMode: u8{1} {
        /// Comparator output (power-up default)
        0 Comparator,
        /// Interrupt output
        1 Interrupt,
    }

    /// Temperature resolution. Affects both sensor accuracy and conversion time.
    #[allow(non_camel_case_types)]
    enum TemperatureResolution: u8{2} {
        /// +0.5°C (t_CONV = 30 ms typical)
        0b00 Deg_0_5C,
        /// +0.25°C (t_CONV = 65 ms typical)
        0b01 Deg_0_25C,
        /// +0.125°C (t_CONV = 130 ms typical)
        0b10 Deg_0_125C,
        /// +0.0625°C (power-up default, t_CONV = 250 ms typical)
        0b11 Deg_0_0625C,
    }

    /// The device configuration register.
    ///
    /// The MCP9808 has a 16-bit Configuration register that allows the user
    /// to set various functions for a robust temperature monitoring system.
    register Configuration(addr = 0b0001, mode = rw, size = 2) {
        /// Reserved bits
        _: u8{5},
        hysteresis: Hysteresis = Hysteresis::Deg_0_0C,
        shutdown_mode: ShutdownMode = ShutdownMode::Continuous,
        critical_lock: CriticalLock = CriticalLock::Unlocked,
        window_lock: WindowLock = WindowLock::Unlocked,
        interrupt_clear: InterruptClear = InterruptClear::NoEffect,
        alert_status: AlertStatus = AlertStatus::NotAsserted,
        alert_control: AlertControl = AlertControl::Disabled,
        alert_select: AlertSelect = AlertSelect::All,
        alert_polarity: AlertPolarity = AlertPolarity::ActiveLow,
        alert_mode: AlertMode = AlertMode::Comparator,
    }

    /// The device-id and revision register.
    register DeviceIdRevision(addr = 0b0111, mode = r, size = 2) {
        /// The Device ID for the MCP9808 is `0x04`.
        device_id: u8 = 0x04,
        /// The revision begins with 0x00 for the first release, with the number
        /// being incremented as revised versions are released.
        device_revision: u8 = 0x00,
    }

    /// The manufacturer ID register.
    register ManufacturerId(addr = 0b0110, mode = r, size = 2) {
        /// The Manufacturer ID for the MCP9808 is `0x0054`.
        manufacturer_id: u16 = 0x0054,
    }

    /// The device resolution register.
    ///
    /// This register allows the user to change the sensor resolution.
    /// The Power-on Reset default resolution is +0.0625°C.
    register Resolution(addr = 0b1000, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{6},
        temperature_resolution: TemperatureResolution = TemperatureResolution::Deg_0_0625C,
    }

    /// The MCP9808 uses a band gap temperature sensor circuit to output analog voltage
    /// proportional to absolute temperature. An internal ΔΣ ADC is used to convert
    /// the analog voltage to a digital word.
    ///
    /// The ambient temperature register bits are double-buffered. Therefore, the user
    /// can access the register, while in the background, the MCP9808 performs an
    /// Analog-to-Digital conversion. The temperature data from the ΔΣ ADC is loaded
    /// in parallel to the TA register at t_CONV refresh rate.
    ///
    /// In addition, the register contains three bits to reflect the alert pin state.
    /// This allows the user to identify the cause of the Alert output trigger.
    /// These are not affected by the status of the Alert Output Configuration
    /// in the configuration register.
    ///
    /// The three least significant temperature bits may stay `0` depending on the
    /// resolution register.
    register AmbientTemperature(addr = 0b0101, mode = r, size = 2) {
        /// Whether T_A is greater than or equal to T_CRIT
        is_critical: bool = false,
        /// Whether T_A is greater than T_UPPER
        is_upper: bool = false,
        /// Whether T_A is lower than T_LOWER
        is_lower: bool = false,
        /// The ambient temperature in °C with a resolution of 0.0625°C/LSB.
        raw_temperature: i16{13} = 0 {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 16f64,
        },
    }

    /// Alert Temperature Upper Boundary Trip register (T_UPPER).
    /// Power-Up Default for T_UPPER is 0°C
    ///
    /// If the alerting feature is enabled in the configuration and the ambient temperature
    /// exceeds the value specified here, the MCP9808 asserts an alert output.
    register TemperatureLimitUpper(addr = 0b0010, mode = rw, size = 2) {
        /// Reserved bits
        _: u8{3},
        /// The temperature limit in °C with a resolution of 0.25°C/LSB.
        raw_temperature_limit: i16{11} = 0 {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 4f64,
        },
        /// Reserved bits
        _: u8{2},
    }

    /// Alert Temperature Lower Boundary Trip register (T_LOWER).
    /// Power-Up Default for T_LOWER is 0°C
    ///
    /// If the alerting feature is enabled in the configuration and the ambient temperature
    /// exceeds the value specified here, the MCP9808 asserts an alert output.
    register TemperatureLimitLower(addr = 0b0011, mode = rw, size = 2) {
        /// Reserved bits
        _: u8{3},
        /// The temperature limit in °C with a resolution of 0.25°C/LSB.
        raw_temperature_limit: i16{11} = 0 {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 4f64,
        },
        /// Reserved bits
        _: u8{2},
    }

    /// The Critical Temperature Trip register (T_CRIT).
    /// Power-Up Default for T_CRIT is 0°C
    ///
    /// If the alerting feature is enabled in the configuration and the ambient temperature
    /// exceeds the value specified here, the MCP9808 asserts an alert output.
    register TemperatureLimitCrit(addr = 0b0100, mode = rw, size = 2) {
        /// Reserved bits
        _: u8{3},
        /// The temperature limit in °C with a resolution of 0.25°C/LSB.
        raw_temperature_limit: i16{11} = 0 {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 4f64,
        },
        /// Reserved bits
        _: u8{2},
    }
}

impl TemperatureResolution {
    pub fn conversion_time_us(&self) -> u32 {
        match self {
            TemperatureResolution::Deg_0_5C => 30_000,
            TemperatureResolution::Deg_0_25C => 65_000,
            TemperatureResolution::Deg_0_125C => 130_000,
            TemperatureResolution::Deg_0_0625C => 250_000,
        }
    }
}
