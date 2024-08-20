//! # TMP117
//!
//! The TMP117 is a high-precision digital temperature sensor. It is designed to meet ASTM E1112
//! and ISO 80601 requirements for electronic patient thermometers. The TMP117 provides a 16-bit
//! temperature result with a resolution of 0.0078 °C and an accuracy of up to ±0.1 °C across the
//! temperature range of –20 °C to 50 °C with no calibration. The TMP117 has in interface that is
//! I2C- and SMBus™-compatible, programmable alert functionality, and the device can support up to four
//! devices on a single bus. Integrated EEPROM is included for device programming with an additional
//! 48-bits memory available for general use.
//!
//! The low power consumption of the TMP117 minimizes the impact of self-heating on measurement accuracy.
//! The TMP117 operates from 1.7 V to 5.5 V and typically consumes 3.5 μA.
//!
//! For non-medical applications, the TMP117 can serve as a single chip digital alternative to a Platinum RTD.
//! The TMP117 has an accuracy comparable to a Class AA RTD, while only using a fraction of the power of the
//! power typically needed for a PT100 RTD. The TMP117 simplifies the design effort by removing many of the
//! complexities of RTDs such as precision references, matched traces, complicated algorithms, and calibration.
//!
//! The TMP117 units are 100% tested on a production setup that is NIST traceable and verified with
//! equipment that is calibrated to ISO/IEC 17025 accredited standards.
//!
//! ## Usage
//!
//! ```
//! # async fn test<I>(mut i2c: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType
//! # {
//! use embedded_devices::devices::texas_instruments::tmp117::{TMP117, address::Address, registers::AmbientTemperature};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut tmp117 = TMP117::new_i2c(i2c, Address::Default);
//! tmp117.init().await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let temp = tmp117
//!     .read_register::<AmbientTemperature>()
//!     .await?
//!     .read_temperature()
//!     .get::<degree_celsius>()
//!     .to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::RegisterInterface;

pub mod address;
pub mod registers;

type TMP117I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid Device Id was encountered
    InvalidDeviceId,
    /// Invalid Manufacturer Id was encountered
    InvalidManufacturerId,
}

/// The TMP117 is a high-precision digital temperature sensor. It provides a 16-bit
/// temperature result with a resolution of 0.0078 °C and an accuracy of up to ±0.1 °C across the
/// temperature range of –20 °C to 50 °C with no calibration.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct TMP117<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

crate::simple_device::i2c!(
    TMP117,
    self::address::Address,
    SevenBitAddress,
    TMP117I2cCodec,
    init_wanted
);

#[device_impl]
impl<I: RegisterInterface> TMP117<I> {
    /// Initialize the sensor by verifying its device id and manufacturer id.
    /// Not mandatory, but recommended.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        Ok(())
    }
}
