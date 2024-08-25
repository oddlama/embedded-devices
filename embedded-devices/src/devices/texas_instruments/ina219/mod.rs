//! # INA219
//!
//! The INA219 is a current shunt and power monitor with an I2C- or SMBUS-compatible interface.
//! The device monitors both shunt voltage drop and bus supply voltage, with programmable
//! conversion times and filtering. A programmable calibration value, combined with
//! an internal multiplier, enables direct readouts of current in amperes. An additional
//! multiplying register calculates power in watts. The I2C- or SMBUS-compatible
//! interface features 16 programmable addresses.
//!
//! The INA219 is available in two grades: A and B. The B grade version has higher accuracy
//! and higher precision specifications.
//!
//! The INA219 senses across shunts on buses that can vary from 0 to 26 V. The device uses
//! a single 3- to 5.5-V supply, drawing a maximum of 1 mA of supply current. The INA219
//! operates from –40°C to 125°C.
//!
//! ## Usage
//!
//! ```
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{RegisterInterface, WritableRegister};

pub mod address;
pub mod registers;

type INA219I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// The INA219 is a 12-bit current shunt and power monitor that can sense
/// on buses with 0-26V. It has a programmable gain amplifier to measure
/// full-scale shunt voltage ranges from 40mV to 320mV.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct INA219<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

crate::simple_device::i2c!(
    INA219,
    self::address::Address,
    SevenBitAddress,
    INA219I2cCodec,
    "init=wanted"
);

#[device_impl]
impl<I: RegisterInterface> INA219<I> {
}
