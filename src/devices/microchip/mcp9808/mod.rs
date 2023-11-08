use embedded_registers::i2c::{I2cDeviceAsync, I2cDeviceSync};
use embedded_registers::{RegisterInterfaceAsync, RegisterInterfaceSync};

pub mod address;
pub mod configuration;
pub mod device_id_revision;
pub mod manufacturer_id;
pub mod resolution;
pub mod temperature;

use self::address::Address;
use self::configuration::ConfigurationRegisterAccessorAsync;
use self::device_id_revision::{DeviceIdRevision, DEVICE_ID_VALID};
use self::manufacturer_id::{ManufacturerId, MANUFACTURER_ID_VALID};

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

//#[derive(SimpleDevice)]
//#[i2c(Address)]
/// An MCP9808 device on the specified bus `I`.
pub struct MCP9808<I> {
    /// The device on the bus interface
    interface: I,
}

impl<I> MCP9808<I2cDeviceAsync<I>>
where
    I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
{
    /// Initializes the MCP9808 driver with the given address.
    ///
    /// This consumes the I2C bus `I`.
    /// Before you retrieve measurements, you should call the `init` method which asserts that
    /// the sensor is working correctly.
    pub fn new_i2c_async(interface: I, address: Address) -> Self {
        Self {
            interface: I2cDeviceAsync {
                interface,
                address: address.into(),
            },
        }
    }
}

impl<I> MCP9808<I2cDeviceSync<I>>
where
    I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
{
    /// Initializes the MCP9808 driver with the given address.
    ///
    /// This consumes the I2C bus `I`.
    /// Before you retrieve measurements, you should call the `init` method which asserts that
    /// the sensor is working correctly.
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
            interface: I2cDeviceSync {
                interface,
                address: address.into(),
            },
        }
    }
}

impl<I> MCP9808<I>
where
    I: RegisterInterfaceAsync,
{
    /// Initialize the sensor and verify its device id and manufacturer id.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        //let device_id = self.read_register::<DeviceIdRevision>().await.map_err(InitError::Bus)?;
        //if device_id.read_device_id() != DEVICE_ID_VALID {
        //    return Err(InitError::InvalidDeviceId);
        //}

        //let manufacturer_id = self.read_register::<ManufacturerId>().await.map_err(InitError::Bus)?;
        //if manufacturer_id.read_manufacturer_id() != MANUFACTURER_ID_VALID {
        //    return Err(InitError::InvalidManufacturerId);
        //}

        Ok(())
    }
}

pub trait MCP9808RegisterProviderAsync<I>: embedded_registers::RegisterInterfaceOwnerAsync<I>
where
    I: RegisterInterfaceAsync,
{
}
pub trait MCP9808RegisterProviderSync<I>: embedded_registers::RegisterInterfaceOwnerSync<I>
where
    I: RegisterInterfaceSync,
{
}

impl<I> embedded_registers::RegisterInterfaceOwnerAsync<I> for MCP9808<I>
where
    I: RegisterInterfaceAsync,
{
    fn interface(&mut self) -> &mut I {
        &mut self.interface
    }
}
impl<I> embedded_registers::RegisterInterfaceOwnerSync<I> for MCP9808<I>
where
    I: RegisterInterfaceSync,
{
    fn interface(&mut self) -> &mut I {
        &mut self.interface
    }
}
impl<I> MCP9808RegisterProviderAsync<I> for MCP9808<I> where I: RegisterInterfaceAsync {}
impl<I> MCP9808RegisterProviderSync<I> for MCP9808<I> where I: RegisterInterfaceSync {}
