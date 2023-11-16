use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub const DEVICE_ID_VALID: u8 = 0x04;
pub const MANUFACTURER_ID_VALID: u16 = 0x0054;

/// T_{UPPER} and T_{LOWER} Limit Hysteresis bits.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode.
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum Hysteresis {
    /// 0.0°C (power-up default)
    #[default]
    Deg_0_0C = 0b00,
    /// 1.5°C
    Deg_1_5C = 0b01,
    /// 3.0°C
    Deg_3_0C = 0b10,
    /// 6.0°C
    Deg_6_0C = 0b11,
}

/// Shutdown mode bit.
/// In shutdown, all power-consuming activities are disabled, though all registers can be written to or read.
///
/// This cannot be set to `1` when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// However, it can be cleared to `0` for continuous conversion while locked.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ShutdownMode {
    /// Continuous conversion (power-up default)
    #[default]
    Continuous = 0,
    /// Shutdown (Low-Power mode)
    Shutdown = 1,
}

/// T_CRIT lock bit.
///
/// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum CriticalLock {
    /// T_CRIT register can be written (power-up default)
    #[default]
    Unlocked = 0,
    /// T_CRIT register cannot be written
    Locked = 1,
}

/// T_UPPER and T_LOWER Window Lock bit.
///
/// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum WindowLock {
    /// T_UPPER and T_LOWER registers can be written (power-up default)
    #[default]
    Unlocked = 0,
    /// T_UPPER and T_LOWER registers cannot be written
    Locked = 1,
}

/// Interrupt Clear bit.
///
/// This bit cannot be set to `1` in Shutdown mode,
/// but it can be cleared after the device enters Shutdown Mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum InterruptClear {
    /// No effect (power-up default)
    #[default]
    NoEffect = 0,
    /// Clear interrupt output. When read, this bit returns to `0` ([InterruptClear::NoEffect])
    ClearInterruptOutput = 1,
}

/// Alert Output Status bit.
///
/// This bit cannot be changed in Shutdown mode.
/// However, if the Alert output is configured as Interrupt mode, and the host controller
/// clears the interrupt bit by reading [InterruptClear] in Shutdown mode,
/// then this bit will also be cleared to `0` [AlertStatus::NotAsserted].
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertStatus {
    /// Alert output is not asserted by the device (power-up default)
    #[default]
    NotAsserted = 0,
    /// Alert output is asserted as a comparator/Interrupt or critical temperature output
    Asserted = 1,
}

/// Alert Output Control bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertControl {
    /// power-up default
    #[default]
    Disabled = 0,
    Enabled = 1,
}

/// Alert Output Select bit.
///
/// This cannot be altered when the window lock bit is set ([WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertSelect {
    /// Alert output for T_UPPER, T_LOWER and T_CRIT (power-up default)
    #[default]
    All = 0,
    /// T_A > T CRIT only (T_UPPER and T_LOWER temperature boundaries are disabled)
    TCritOnly = 1,
}

/// Alert Output Polarity bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPolarity {
    /// power-up default; pull-up resistor required
    #[default]
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Alert Output Mode bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertMode {
    /// Comparator output (power-up default)
    #[default]
    Comparator = 0,
    /// Interrupt output
    Interrupt = 1,
}

/// The device configuration register.
///
/// The MCP9808 has a 16-bit Configuration register that allows the user
/// to set various functions for a robust temperature monitoring system.
#[device_register(super::MCP9808)]
#[register(address = 0b0001, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub hysteresis: Hysteresis,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub shutdown_mode: ShutdownMode,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub critical_lock: CriticalLock,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub window_lock: WindowLock,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub interrupt_clear: InterruptClear,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_status: AlertStatus,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_control: AlertControl,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_select: AlertSelect,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_polarity: AlertPolarity,
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub alert_mode: AlertMode,
}

/// The device-id and revision register.
#[device_register(super::MCP9808)]
#[register(address = 0b0111, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceIdRevision {
    /// The Device ID for the MCP9808 is `0x04`.
    device_id: u8,
    /// The revision begins with 0x00 for the first release, with the number
    /// being incremented as revised versions are released.
    device_revision: u8,
}

/// The device-id and revision register.
#[device_register(super::MCP9808)]
#[register(address = 0b0110, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ManufacturerId {
    /// The Manufacturer ID for the MCP9808 is `0x0054`.
    manufacturer_id: u16,
}

/// Temperature resolution. Affects both sensor accuracy and conversion time.
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum TemperatureResolution {
    /// +0.5°C (t_CONV = 30 ms typical)
    Deg_0_5C = 0b00,
    /// +0.25°C (t_CONV = 65 ms typical)
    Deg_0_25C = 0b01,
    /// +0.125°C (t_CONV = 130 ms typical)
    Deg_0_125C = 0b10,
    /// +0.0625°C (power-up default, t_CONV = 250 ms typical)
    #[default]
    Deg_0_0625C = 0b11,
}

/// The device resolution register.
///
/// This register allows the user to change the sensor resolution.
/// The Power-on Reset default resolution is +0.0625°C.
#[device_register(super::MCP9808)]
#[register(address = 0b1000, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Resolution {
    #[bondrewd(bit_length = 6, reserve)]
    #[allow(dead_code)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub temperature_resolution: TemperatureResolution,
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
#[device_register(super::MCP9808)]
#[register(address = 0b0101, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AmbientTemperature {
    /// Whether T_A is greater than or equal to T_CRIT
    pub is_critical: bool,
    /// Whether T_A is greater than T_UPPER
    pub is_upper: bool,
    /// Whether T_A is lower than T_LOWER
    pub is_lower: bool,
    /// The ambient temperature in °C with a resolution of 0.0625°C/LSB.
    #[bondrewd(bit_length = 13)]
    pub raw_temperature: i16,
}

impl AmbientTemperature {
    /// Reads the ambient temperature in °C with a resolution of 0.0625°C/LSB.
    pub fn read_temperature(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(self.read_raw_temperature().into(), 16))
    }
}

macro_rules! define_temp_limit_register {
    ($name:ident, $address:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::MCP9808)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            #[bondrewd(bit_length = 3, reserve)]
            #[allow(dead_code)]
            reserved0: u8,

            /// The temperature limit in °C with a resolution of 0.25°C/LSB.
            #[bondrewd(bit_length = 11)]
            pub raw_temperature_limit: i16,

            #[bondrewd(bit_length = 2, reserve)]
            #[allow(dead_code)]
            reserved1: u8,
        }

        impl $name {
            /// Reads the ambient temperature in °C with a resolution of 0.25°C/LSB.
            pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
                ThermodynamicTemperature::new::<degree_celsius>(
                    Rational32::new_raw(self.read_raw_temperature_limit().into(), 4).reduced(),
                )
            }

            /// Writes the ambient temperature in °C with a resolution of 0.25°C/LSB.
            /// The passed temperature will be truncated (ounded down).
            pub fn write_temperature_limit(
                &mut self,
                temperature_limit: ThermodynamicTemperature,
            ) -> Result<(), core::num::TryFromIntError> {
                let temp = temperature_limit.get::<degree_celsius>();
                let temp: i16 = (temp * Rational32::from_integer(4)).to_integer().try_into()?;
                self.write_raw_temperature_limit(temp);
                Ok(())
            }
        }
    };
}

define_temp_limit_register!(
    TemperatureLimitUpper,
    0b0010,
    r#"
The Alert Temperature Upper Boundary Trip register (T_UPPER).
Power-Up Default for T_UPPER is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);

define_temp_limit_register!(
    TemperatureLimitLower,
    0b0011,
    r#"
Alert Temperature Lower Boundary Trip register (T_LOWER).
Power-Up Default for T_LOWER is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);

define_temp_limit_register!(
    TemperatureLimitCrit,
    0b0100,
    r#"
The Critical Temperature Trip register (T_CRIT).
Power-Up Default for T_CRIT is 0°C

If the alerting feature is enabled in the configuration and the ambient temperature
exceeds the value specified here, the MCP9808 asserts an alert output.
"#
);
