//! # MCP9808
//!
//! Microchip Technology Inc.'s MCP9808 digital temperature sensor converts temperatures between
//! -20°C and +100°C to a digital word with ±0.25°C/±0.5°C (typical/maximum) accuracy.
//!
//! The MCP9808 comes with user-programmable registers that provide flexibility for temperature sensing
//! applications. The registers allow user-selectable settings such as Shutdown or Low-Power modes and
//! the specification of temperature Alert window limits and critical output limits.
//!
//! When the temperature changes beyond the specified boundary limits, the MCP9808 outputs an Alert signal.
//! The user has the option of setting the Alert output signal polarity as an active-low or active-high
//! comparator output for thermostat operation, or as a temperature Alert interrupt output for
//! microprocessor-based systems. The Alert output can also be configured as a critical temperature output only.
//!
//! This sensor has an industry standard 400 kHz, 2-wire, SMBus/I2C compatible serial interface,
//! allowing up to eight or sixteen sensors to be controlled with a single serial bus.
//! These features make the MCP9808 ideal for sophisticated, multi-zone, temperature-monitoring applications.
//!
//! ## Usage
//!
//! ```
//! # async fn test<I>(mut i2c: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType
//! # {
//! use embedded_devices::devices::microchip::mcp9808::{MCP9808, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut mcp9808 = MCP9808::new_i2c(i2c, Address::Default);
//! mcp9808.init().await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let temp = mcp9808
//!     .read_ambient_temperature()
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

/// Microchip Technology Inc.'s MCP9808 digital temperature sensor converts temperatures between
/// -20°C and +100°C to a digital word with ±0.25°C/±0.5°C (typical/maximum) accuracy.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct MCP9808<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

crate::simple_device::i2c!(MCP9808, self::address::Address, init_wanted);

#[device_impl]
impl<I> MCP9808<I>
where
    I: RegisterInterface,
{
    /// Initialize the sensor by verifying its device id and manufacturer id.
    /// Not mandatory, but recommended.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        use self::registers::DeviceIdRevision;
        use self::registers::ManufacturerId;

        let device_id = self.read_register::<DeviceIdRevision>().await.map_err(InitError::Bus)?;
        if device_id.read_device_id() != self::registers::DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId);
        }

        let manufacturer_id = self.read_register::<ManufacturerId>().await.map_err(InitError::Bus)?;
        if manufacturer_id.read_manufacturer_id() != self::registers::MANUFACTURER_ID_VALID {
            return Err(InitError::InvalidManufacturerId);
        }

        Ok(())
    }
}
