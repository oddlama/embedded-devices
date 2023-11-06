use defmt::info;
use embedded_hal_async::i2c;
use embedded_registers::{Register, RegisterRead, RegisterWrite};

pub mod address;
pub mod configuration;
pub mod device_id_revision;
pub mod manufacturer_id;

use crate::devices::mcp9808::address::Address;
use crate::devices::mcp9808::device_id_revision::{DeviceIdRevision, DEVICE_ID_VALID};
use crate::devices::mcp9808::manufacturer_id::{ManufacturerId, MANUFACTURER_ID_VALID};

/// All possible errors in this crate
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
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
    /// Before you can get measurements, you must call the `init` method which configures the sensor.
    pub fn new(i2c: I, address: Address) -> Self {
        Self {
            i2c,
            address: address.into(),
        }
    }

    /// Reads the given register from the device
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, Error<I::Error>>
    where
        R: Register + RegisterRead,
    {
        R::read_i2c(&mut self.i2c, self.address).await.map_err(Error::Bus)
    }

    /// Writes the register to the device
    #[inline]
    async fn write_register<R>(&mut self, register: &R) -> Result<(), Error<I::Error>>
    where
        R: Register + RegisterWrite,
    {
        register
            .write_i2c(&mut self.i2c, self.address)
            .await
            .map_err(Error::Bus)
    }

    pub async fn init(&mut self) -> Result<(), Error<I::Error>> {
        let device_id = self.read_register::<DeviceIdRevision>().await?;
        if device_id.read_device_id() != DEVICE_ID_VALID {
            // TODO     return Error()
        }

        let manufacturer_id = self.read_register::<ManufacturerId>().await?;
        if manufacturer_id.read_manufacturer_id() != MANUFACTURER_ID_VALID {
            // TODO     return Error()
        }
        info!("{:?} {:?}", device_id, manufacturer_id);

        // TODO expose init error for this!
        //assert_eq!(config.device_id, 0x04);

        Ok(())
    }
}
