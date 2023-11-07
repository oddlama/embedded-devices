use embedded_hal_async::i2c;
use embedded_registers::{Register, RegisterRead, RegisterWrite};

pub mod address;
pub mod configuration;
pub mod device_id_revision;
pub mod manufacturer_id;
pub mod resolution;
pub mod temperature;

use crate::devices::mcp9808::address::Address;
use crate::devices::mcp9808::device_id_revision::{DeviceIdRevision, DEVICE_ID_VALID};
use crate::devices::mcp9808::manufacturer_id::{ManufacturerId, MANUFACTURER_ID_VALID};

/// All possible errors in this crate
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid Device Id was encountered
    InvalidDeviceId,
    /// Invalid Manufacturer Id was encountered
    InvalidManufacturerId,
}

/// An MCP9808 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `SENSOR_ADDRESS` from this package, unless there is some kind
/// of special address translating hardware in use.
pub struct MCP9808<I>
where
    I: i2c::I2c + i2c::ErrorType,
{
    /// I2c interface
    i2c: I,
    /// Device address
    address: u8,
}

impl<I> MCP9808<I>
where
    I: i2c::I2c + i2c::ErrorType,
{
    /// Initializes the MCP9808 driver with the given address.
    ///
    /// This consumes the I2C bus `I`.
    /// Before you retrieve measurements, you should call the `init` method which asserts that
    /// the sensor is working correctly.
    pub fn new(i2c: I, address: Address) -> Self {
        Self {
            i2c,
            address: address.into(),
        }
    }

    /// Reads the given register from the device
    #[inline]
    pub async fn read_register<R>(&mut self) -> Result<R, I::Error>
    where
        R: Register + RegisterRead,
    {
        R::read_i2c(&mut self.i2c, self.address).await
    }

    /// Writes the register to the device
    #[inline]
    pub async fn write_register<R>(&mut self, register: &R) -> Result<(), I::Error>
    where
        R: Register + RegisterWrite,
    {
        register.write_i2c(&mut self.i2c, self.address).await
    }

    /// Initialize the sensor and verify its device id and manufacturer id.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        let device_id = self.read_register::<DeviceIdRevision>().await.map_err(InitError::Bus)?;
        if device_id.read_device_id() != DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId);
        }

        let manufacturer_id = self.read_register::<ManufacturerId>().await.map_err(InitError::Bus)?;
        if manufacturer_id.read_manufacturer_id() != MANUFACTURER_ID_VALID {
            return Err(InitError::InvalidManufacturerId);
        }

        Ok(())
    }
}
