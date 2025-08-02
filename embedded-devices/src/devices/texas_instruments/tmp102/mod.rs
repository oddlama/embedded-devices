//! The TMP102 device is a digital temperature sensor designed for NTC/PTC
//! thermistor replacement where high accuracy is required. The device offers an
//! accuracy of ±0.5°C without requiring calibration or external component
//! signal conditioning. Device temperature sensors are highly linear and do not
//! require complex calculations or lookup tables to derive the temperature. The
//! on-chip 12-bit ADC offers resolutions down to 0.0625°C.
//!
//! ## Usage (sync)
//!
//! ```rust, no_run
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_interfaces::TransportError<(), I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp102::{TMP102Sync, address::Address, registers::Temperature};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp102 = TMP102Sync::new_i2c(delay, i2c, Address::Gnd);
//!
//! // Read the latest temperature conversion in °C
//! let temp = tmp102
//!     .read_temperature()?
//!     .get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp102.measure()?
//!     .temperature.get::<degree_celsius>();
//! println!("Oneshot temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, no_run
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_interfaces::TransportError<(), I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp102::{TMP102Async, address::Address, registers::Temperature};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp102 = TMP102Async::new_i2c(delay, i2c, Address::Gnd);
//!
//! // Read the latest temperature conversion in °C
//! let temp = tmp102
//!     .read_temperature().await?
//!     .get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp102.measure().await?
//!     .temperature.get::<degree_celsius>();
//! println!("Oneshot temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::{forward_register_fns, sensor};
use embedded_interfaces::TransportError;
use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::registers::{Configuration, Temperature};

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Measured temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
}

/// The TMP102 is a general purpose digital temperature sensor. It provides a 13-bit
/// temperature result with a resolution of 0.0625 °C and an accuracy of up to ±3 °C across the
/// temperature range of –40 °C to 125 °C with no calibration.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct TMP102<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
}

pub trait TMP102Register {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> TMP102<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: self::address::Address) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}

#[forward_register_fns]
#[sensor(Temperature)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> TMP102<D, I> {
    /// Read the last temperature measured
    pub async fn read_temperature(&mut self) -> Result<ThermodynamicTemperature, TransportError<(), I::BusError>> {
        let reg_conf = self.read_register::<Configuration>().await?;

        // Read and return the temperature
        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        if reg_conf.read_extended() {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 128.0))
        } else {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 256.0))
        }
    }
}

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
    for TMP102<D, I>
{
    type Error = TransportError<(), I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will set `shutdown` in [`self::registers::Configuration´].
    /// which will cause the device to perform a single conversion a return to sleep mode afterwards.
    ///
    /// This function will initialize the measurement, wait until the data is acquired and return
    /// the temperature.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        self.write_register(reg_conf.with_oneshot(true).with_shutdown(false))
            .await?;

        // Wait for the duration of the conversion
        self.delay.delay_us(12_500).await;

        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        if reg_conf.read_extended() {
            Ok(Measurement {
                temperature: ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 128.0),
            })
        } else {
            Ok(Measurement {
                temperature: ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 256.0),
            })
        }
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
    for TMP102<D, I>
{
    type Error = TransportError<(), I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        self.write_register(reg_conf.with_oneshot(false).with_shutdown(false))
            .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        self.write_register(reg_conf.with_oneshot(false).with_shutdown(true))
            .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        let reg_conf = self.read_register::<Configuration>().await?;
        let conversion_time_us = reg_conf.read_conversion_cycle_time().conversion_time_ms() * 1000;
        Ok(conversion_time_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        let reg_conf = self.read_register::<Configuration>().await?;
        if reg_conf.read_extended() {
            Ok(Some(Measurement {
                temperature: ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 128.0),
            }))
        } else {
            Ok(Some(Measurement {
                temperature: ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 256.0),
            }))
        }
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
