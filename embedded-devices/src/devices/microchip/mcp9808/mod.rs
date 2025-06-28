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
//! ## Usage (sync)
//!
//! ```rust, only_if(sync)
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::microchip::mcp9808::{MCP9808Sync, address::Address, registers::AmbientTemperature};
//! use embedded_devices::sensors::SensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut mcp9808 = MCP9808Sync::new_i2c(i2c, Address::Default);
//! mcp9808.init().unwrap();
//!
//! // Read the current temperature in °C
//! let temp = mcp9808.measure(&mut Delay)?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::microchip::mcp9808::{MCP9808Async, address::Address, registers::AmbientTemperature};
//! use embedded_devices::sensors::SensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut mcp9808 = MCP9808Async::new_i2c(i2c, Address::Default);
//! mcp9808.init().await.unwrap();
//!
//! // Read the current temperature in °C
//! let temp = mcp9808.measure(&mut Delay).await?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl, sensor};
use registers::Resolution;
use uom::si::f64::ThermodynamicTemperature;

pub mod address;
pub mod registers;

type MCP9808I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

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

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Measured temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
}

/// Microchip Technology Inc.'s MCP9808 digital temperature sensor converts temperatures between
/// -20°C and +100°C to a digital word with ±0.25°C/±0.5°C (typical/maximum) accuracy.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct MCP9808<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> MCP9808<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, MCP9808I2cCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(interface: I, address: self::address::Address) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> MCP9808<I> {
    /// Initializes the sensor by verifying its device id and manufacturer id.
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

#[sensor(Temperature)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface, Sensor),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> crate::sensors::Sensor for MCP9808<I> {
    type Error = I::Error;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will power up the sensor, wait until a conversion is
    /// ready and return to sleep afterwards.
    async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Self::Measurement, Self::Error> {
        use self::registers::{AmbientTemperature, Configuration, ShutdownMode};

        // Enable conversions
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_shutdown_mode(ShutdownMode::Continuous);
        self.write_register(reg_conf).await?;

        // Wait until conversion is ready
        let resolution = self.read_register::<Resolution>().await?;
        let conversion_time = 1000 + resolution.read_temperature_resolution().conversion_time_us();
        delay.delay_us(conversion_time).await;

        // Go to sleep
        reg_conf.write_shutdown_mode(ShutdownMode::Shutdown);
        self.write_register(reg_conf).await?;

        // Read and return the temperature
        let temperature = self.read_register::<AmbientTemperature>().await?.read_temperature();
        Ok(Measurement { temperature })
    }
}
