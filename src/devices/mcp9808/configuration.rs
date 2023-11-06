use bondrewd::BitfieldEnum;
use embedded_registers::register;

/// T_{UPPER} and T_{LOWER} Limit Hysteresis bits.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode.
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum Hysteresis {
    /// 0.0°C (power-up default)
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ShutdownMode {
    /// Continuous conversion (power-up default)
    Continuous = 0,
    /// Shutdown (Low-Power mode)
    Shutdown = 1,
}

/// T_CRIT lock bit.
///
/// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum CriticalLock {
    /// T_CRIT register can be written (power-up default)
    Unlocked = 0,
    /// T_CRIT register cannot be written
    Locked = 1,
}

/// T_UPPER and T_LOWER Window Lock bit.
///
/// When enabled, this bit remains set to `1` (locked) until cleared by a Power-on Reset.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum WindowLock {
    /// T_UPPER and T_LOWER registers can be written (power-up default)
    Unlocked = 0,
    /// T_UPPER and T_LOWER registers cannot be written
    Locked = 1,
}

/// Interrupt Clear bit.
///
/// This bit cannot be set to `1` in Shutdown mode,
/// but it can be cleared after the device enters Shutdown Mode.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum InterruptClear {
    /// No effect (power-up default)
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertStatus {
    /// Alert output is not asserted by the device (power-up default)
    NotAsserted = 0,
    /// Alert output is asserted as a comparator/Interrupt or critical temperature output
    Asserted = 1,
}

/// Alert Output Control bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertControl {
    /// power-up default
    Disabled = 0,
    Enabled = 1,
}

/// Alert Output Select bit.
///
/// This cannot be altered when the window lock bit is set ([WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertSelect {
    /// Alert output for T_UPPER, T_LOWER and T_CRIT (power-up default)
    All = 0,
    /// T_A > T CRIT only (T_UPPER and T_LOWER temperature boundaries are disabled)
    TCritOnly = 1,
}

/// Alert Output Polarity bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPolarity {
    /// power-up default; pull-up resistor required
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Alert Output Mode bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertMode {
    /// Comparator output (power-up default)
    Comparator = 0,
    /// Interrupt output
    Interrupt = 1,
}

/// The device configuration register.
///
/// The MCP9808 has a 16-bit Configuration register that allows the user
/// to set various functions for a robust temperature monitoring system.
// TODO #[register(MCP9808, address = 0b001, read, write)]
#[register(address = 0b001, read, write)]
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
