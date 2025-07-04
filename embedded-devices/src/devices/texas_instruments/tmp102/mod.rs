//! # TMP102
//!
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
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_registers::RegisterError<(), I::Error>>
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
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_registers::RegisterError<(), I::Error>>
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

use embedded_devices_derive::{device, device_impl, sensor};
use embedded_registers::RegisterError;
use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

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
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct TMP102<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> {
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
impl<D, I> TMP102<D, embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> TMP102<D, I> {
    /// Read the last temperature measured
    pub async fn read_temperature(&mut self) -> Result<ThermodynamicTemperature, RegisterError<(), I::BusError>> {
        use self::registers::{Configuration, Temperature};

        // Read current configuration to determine conversion ratio
        let reg_conf = self.read_register::<Configuration>().await?;

        // Read and return the temperature
        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        if reg_conf.read_extended() {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 128.0))
        } else {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(raw_temp as f64 / 256.0))
        }
    }

    pub async fn set_continuous(&mut self) -> Result<(), RegisterError<(), I::BusError>> {
        use self::registers::Configuration;

        // Read current configuration and update it for continuous mode
        let reg_conf = self.read_register::<Configuration>().await?;

        // Initiate measurement
        self.write_register(reg_conf.with_oneshot(false).with_extended(true).with_shutdown(false))
            .await?;

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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::OneshotSensor for TMP102<D, I> {
    type Error = RegisterError<(), I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will set `shutdown` in [`self::registers::Configuration´].
    /// which will cause the device to perform a single conversion a return to sleep mode afterwards.
    ///
    /// This function will initialize the measurement, wait until the data is acquired and return
    /// the temperature.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        use self::registers::{Configuration, Temperature};

        // Read current configuration to determine conversion ratio and delay
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_oneshot(true);

        // Initiate measurement
        self.write_register(reg_conf).await?;

        // Wait for the duration of the conversion
        let active_conversion_time = reg_conf.read_conversion_cycle_time().conversion_time_ms() + 10;
        self.delay.delay_ms(active_conversion_time).await;

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
