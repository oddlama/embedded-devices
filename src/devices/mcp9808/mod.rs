use bondrewd::BitfieldEnum;
use defmt::{info, Format};
use embedded_hal_async::i2c;
use embedded_registers::{Register, RegisterRead, RegisterWrite};

use self::reg::MCP9808Register;

/// All possible errors in this crate
#[derive(Debug, Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
}

/// T_{UPPER} and T_{LOWER} Limit Hysteresis bits.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode.
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
#[bondrewd_enum(u8)]
pub enum AlertSelect {
    /// Alert output for TUPPER, TLOWER and TCRIT (power-up default)
    All = 0,
    TCritOnly = 1,
}

/// Alert Output Polarity bit.
///
/// This cannot be altered when either of the lock bits are set ([CriticalLock::Locked] / [WindowLock::Locked]).
/// This bit can be programmed in Shutdown mode, but the Alert output will not assert or deassert.
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
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
#[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
#[bondrewd_enum(u8)]
pub enum AlertMode {
    /// power-up default
    //#[bitfield_enum_fallback]
    Comparator = 0,
    Interrupt = 1,
}

pub mod reg {
    use embedded_registers::{register, Register};

    use super::*;

    /// Full representation of the configuration register.
    #[register(address = 0b001, read, write)]
    #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
    pub struct Config {
        #[rustfmt::skip] #[bondrewd(bit_length = 5, reserve)] #[allow(dead_code)] reserved: u8,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 2)] pub hysteresis: Hysteresis,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub shutdown_mode: ShutdownMode,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub critical_lock: CriticalLock,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub window_lock: WindowLock,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub interrupt_clear: InterruptClear,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub alert_status: AlertStatus,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub alert_control: AlertControl,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub alert_select: AlertSelect,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub alert_polarity: AlertPolarity,
        #[rustfmt::skip] #[bondrewd(enum_primitive = "u8", bit_length = 1)] pub alert_mode: AlertMode,
    }

    /// Full representation of the device id register.
    #[register(address = 0b111, read)]
    #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
    pub struct DeviceId {
        device_id: u8,
        revision: u8,
    }

    pub trait MCP9808Register: Register {}
    impl MCP9808Register for Config {}
    impl MCP9808Register for DeviceId {}
}

/// An MCP9808 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `SENSOR_ADDRESS` from this package, unless there is some kind
/// of special address translating hardware in use.
pub struct MCP9808<I> {
    i2c: I,
    address: u8,
}

impl<I> MCP9808<I>
where
    I: i2c::I2c + i2c::ErrorType,
{
    /// Initializes the MCP9808 driver with the default address.
    ///
    /// This consumes the I2C bus `I`.
    /// Before you can get measurements, you must call the `init` method which configures the sensor.
    pub fn new(i2c: I) -> Self {
        MCP9808::new_with_address(i2c, 0b0011000)
    }

    /// Initializes the MCP9808 driver with the given address.
    ///
    /// This consumes the I2C bus `I`.
    /// Before you can get measurements, you must call the `init` method which configures the sensor.
    pub fn new_with_address(i2c: I, address: u8) -> Self {
        Self { i2c, address }
    }

    #[inline]
    pub async fn read_register<R>(&mut self) -> Result<R, Error<I::Error>>
    where
        R: MCP9808Register + Register + RegisterRead,
    {
        R::read_i2c(&mut self.i2c, self.address).await.map_err(Error::Bus)
    }

    #[inline]
    pub async fn write_register<R>(&mut self, register: &R) -> Result<(), Error<I::Error>>
    where
        R: MCP9808Register + Register + RegisterWrite,
    {
        register
            .write_i2c(&mut self.i2c, self.address)
            .await
            .map_err(Error::Bus)
    }

    pub async fn init(&mut self) -> Result<(), Error<I::Error>> {
        let device_id = self.read_register::<reg::DeviceId>().await?;
        info!("{:?}", device_id);

        // TODO expose init error for this!
        //assert_eq!(config.device_id, 0x04);

        Ok(())
    }
}
