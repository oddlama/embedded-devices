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
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_interfaces::TransportError<(), I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::microchip::mcp9808::{MCP9808Sync, address::Address, registers::AmbientTemperature};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut mcp9808 = MCP9808Sync::new_i2c(delay, i2c, Address::Default);
//! mcp9808.init().unwrap();
//!
//! // Read the current temperature in °C
//! let temp = mcp9808.measure()?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_interfaces::TransportError<(), I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::microchip::mcp9808::{MCP9808Async, address::Address, registers::AmbientTemperature};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut mcp9808 = MCP9808Async::new_i2c(delay, i2c, Address::Default);
//! mcp9808.init().await.unwrap();
//!
//! // Read the current temperature in °C
//! let temp = mcp9808.measure().await?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::{device, device_impl, sensor};
use embedded_interfaces::TransportError;
use uom::si::f64::ThermodynamicTemperature;

use crate::utils::from_bus_error;

pub mod address;
pub mod registers;

use self::registers::{AmbientTemperature, Configuration, DeviceIdRevision, ManufacturerId, Resolution, ShutdownMode};

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum InitError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Invalid Device Id was encountered
    #[error("invalid device id")]
    InvalidDeviceId,
    /// Invalid Manufacturer Id was encountered
    #[error("invalid manufacturer id")]
    InvalidManufacturerId,
}

from_bus_error!(InitError);

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
pub struct MCP9808<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> MCP9808<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: self::address::Address) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> MCP9808<D, I> {
    /// Initializes the sensor by verifying its device id and manufacturer id.
    /// Not mandatory, but recommended.
    pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
        let device_id = self.read_register::<DeviceIdRevision>().await?;
        if device_id.read_device_id() != self::registers::DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId);
        }

        let manufacturer_id = self.read_register::<ManufacturerId>().await?;
        if manufacturer_id.read_manufacturer_id() != self::registers::MANUFACTURER_ID_VALID {
            return Err(InitError::InvalidManufacturerId);
        }

        Ok(())
    }
}

#[sensor(Temperature)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        OneshotSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::sensor::OneshotSensor
    for MCP9808<D, I>
{
    type Error = TransportError<(), I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will power up the sensor, wait until a conversion is
    /// ready and return to sleep afterwards.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        // Enable conversions
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_shutdown_mode(ShutdownMode::Continuous);
        self.write_register(reg_conf).await?;

        // Wait until conversion is ready
        let resolution = self.read_register::<Resolution>().await?;
        let conversion_time = 1000 + resolution.read_temperature_resolution().conversion_time_us();
        self.delay.delay_us(conversion_time).await;

        // Go to sleep
        reg_conf.write_shutdown_mode(ShutdownMode::Shutdown);
        self.write_register(reg_conf).await?;

        // Read and return the temperature
        let temperature = self.read_register::<AmbientTemperature>().await?.read_temperature();
        Ok(Measurement { temperature })
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ContinuousSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::sensor::ContinuousSensor
    for MCP9808<D, I>
{
    type Error = TransportError<(), I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        self.write_register(reg_conf.with_shutdown_mode(ShutdownMode::Continuous))
            .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        self.write_register(reg_conf.with_shutdown_mode(ShutdownMode::Shutdown))
            .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        // Wait until conversion is ready
        let resolution = self.read_register::<Resolution>().await?;
        let conversion_time = resolution.read_temperature_resolution().conversion_time_us();
        Ok(conversion_time)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let temperature = self.read_register::<AmbientTemperature>().await?.read_temperature();
        Ok(Some(Measurement { temperature }))
    }

    /// Not supported, always returns true.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    /// Opportunistically waits one conversion interval and returns the measurement.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        let interval = self.measurement_interval_us().await?;
        self.delay.delay_us(interval).await;
        self.current_measurement().await.map(Option::unwrap)
    }
}
