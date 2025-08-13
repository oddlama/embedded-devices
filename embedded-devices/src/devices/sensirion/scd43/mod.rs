//! The SCD43 is a photoacoustic NDIR CO2 sensor from Sensirion's SCD4x family which features high
//! accuracy, large measurement range of 400 - 5000 ppm, single shot operation feature and an
//! inbuilt SHT4x temperature and humidity sensor for measurement compensation.
//!
//! The SCD4x is Sensirion's second generation series of optical CO2 sensors. The sensor series
//! builds on the photoacoustic NDIR sensing principle and Sensirion's patented PASens® and CMOSens®
//! technology to offer high accuracy at an attractive price and small form factor.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::scd43::InitError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd43::{SCD43Sync, address::Address};
//! use embedded_devices::sensor::ContinuousSensorSync;
//! use uom::si::{
//!    mass_concentration::microgram_per_cubic_meter,
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut scd43 = SCD43Sync::new_i2c(delay, i2c, Address::Default);
//! scd43.init()?;
//! scd43.start_measuring()?;
//!
//! // Read measurements
//! let measurement = scd43.next_measurement()?;
//! let humidity = measurement.relative_humidity.get::<percent>();
//! let temperature = measurement.temperature.get::<degree_celsius>();
//! let co2 = measurement.co2_concentration.get::<part_per_million>();
//! println!("Current measurement: {:?}%RH, {:?}°C, {:?}ppm CO₂", humidity, temperature, co2);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::scd43::InitError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd43::{SCD43Async, address::Address};
//! use embedded_devices::sensor::ContinuousSensorAsync;
//! use uom::si::{
//!    mass_concentration::microgram_per_cubic_meter,
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut scd43 = SCD43Async::new_i2c(delay, i2c, Address::Default);
//! scd43.init().await?;
//! scd43.start_measuring().await?;
//!
//! // Read measurements
//! let measurement = scd43.next_measurement().await?;
//! let humidity = measurement.relative_humidity.get::<percent>();
//! let temperature = measurement.temperature.get::<degree_celsius>();
//! let co2 = measurement.co2_concentration.get::<part_per_million>();
//! println!("Current measurement: {:?}%RH, {:?}°C, {:?}ppm CO₂", humidity, temperature, co2);
//! # Ok(())
//! # }
//! # }
//! ```

use self::commands::{
    GetDataReady, GetSensorVariant, MeasureSingleShot, PerformForcedRecalibration, ReadMeasurement, Reinit,
    StartPeriodicMeasurement, StopPeriodicMeasurement, WakeUp,
};
use embedded_devices_derive::{forward_command_fns, sensor};
use uom::si::f64::{Ratio, ThermodynamicTemperature};

use super::{
    commands::Crc8Error,
    scd4x::commands::{DataReadyStatus, SensorVariant, TargetCo2Concentration},
};

pub use super::scd4x::address;
pub mod commands;

/// Any CRC or Bus related error
pub type TransportError<E> = embedded_interfaces::TransportError<Crc8Error, E>;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, thiserror::Error)]
pub enum InitError<BusError> {
    /// Transport error
    #[error("transport error")]
    Transport(#[from] TransportError<BusError>),
    /// Invalid sensor variant was encountered
    #[error("invalid sensor variant {0:?}")]
    InvalidSensorVariant(SensorVariant),
}

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Ambient relative humidity
    #[measurement(RelativeHumidity)]
    pub relative_humidity: Ratio,
    /// Ambient temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
    /// Current CO₂ concentration
    #[measurement(Co2Concentration)]
    pub co2_concentration: Ratio,
}

/// The SCD43 is a photoacoustic NDIR CO2 sensor from Sensirion's SCD4x family which features high
/// accuracy, large measurement range of 400 - 5000 ppm, single shot operation feature and an
/// inbuilt SHT4x temperature and humidity sensor for measurement compensation.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        I2cDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct SCD43<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> {
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
impl<D, I> SCD43<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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

pub trait SCD43Command {}

#[forward_command_fns]
#[sensor(RelativeHumidity, Temperature, Co2Concentration)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> SCD43<D, I> {
    /// Initializes the sensor by stopping any ongoing measurement, resetting the device and
    /// verifying the sensor variant.
    pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
        use crate::device::ResettableDevice;

        // Datasheet specifies 30ms before sensor has reached idle state after power-up.
        self.delay.delay_ms(30).await;
        self.reset().await?;

        // Verify sensor variant
        match self.execute::<GetSensorVariant>(()).await?.read_variant() {
            SensorVariant::r#SCD43 => Ok(()),
            variant => Err(InitError::InvalidSensorVariant(variant)),
        }
    }

    /// Performs forced recalibration (FRC) of the CO2 signal. See the datasheet of the SCD4x
    /// sensor (which is used in this sensor) for details how the forced recalibration shall be
    /// used.
    ///
    /// After power-on wait at least 1000 ms and after stopping a measurement 600 ms before sending
    /// this command. This function will take about 500 ms to complete.
    ///
    /// Returns None if the recalibration failed, otherwise the correction in PPM.
    pub async fn perform_forced_recalibration(
        &mut self,
        target_co2_concentration: Ratio,
    ) -> Result<Option<Ratio>, TransportError<I::BusError>> {
        let frc_correction = self
            .execute::<PerformForcedRecalibration>(
                TargetCo2Concentration::default().with_target_co2_concentration(target_co2_concentration),
            )
            .await?;

        Ok((frc_correction.read_raw_correction() != u16::MAX).then(|| frc_correction.read_correction()))
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> crate::device::ResettableDevice
    for SCD43<D, I>
{
    type Error = TransportError<I::BusError>;

    /// Resets the sensor by stopping any ongoing measurement, and resetting the device.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Try to wake the sensor up
        let _ = self.execute::<WakeUp>(()).await;
        // Try to stop measurement if it is ongoing, otherwise ignore
        let _ = self.execute::<StopPeriodicMeasurement>(()).await;
        // Reset
        self.execute::<Reinit>(()).await?;
        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ContinuousSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> crate::sensor::ContinuousSensor
    for SCD43<D, I>
{
    type Error = TransportError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StartPeriodicMeasurement>(()).await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StopPeriodicMeasurement>(()).await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        Ok(5_000_000)
    }

    /// Returns the most recent measurement.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let measurement = self.execute::<ReadMeasurement>(()).await?;
        Ok(Some(Measurement {
            relative_humidity: measurement.read_relative_humidity(),
            temperature: measurement.read_temperature(),
            co2_concentration: measurement.read_co2_concentration(),
        }))
    }

    /// Check if new measurements are available.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.execute::<GetDataReady>(()).await?.read_data_ready() == DataReadyStatus::Ready)
    }

    /// Wait indefinitely until new measurements are available and return them. Checks whether data
    /// is ready in intervals of 100ms.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        loop {
            if self.is_measurement_ready().await? {
                return self.current_measurement().await?.ok_or_else(|| {
                    TransportError::Unexpected("measurement was not ready even though we expected it to be")
                });
            }
            self.delay.delay_ms(100).await;
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ContinuousSensor,
        OneshotSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> crate::sensor::OneshotSensor
    for SCD43<D, I>
{
    type Error = TransportError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        use crate::sensor::ContinuousSensor;
        self.execute::<MeasureSingleShot>(()).await?;
        self.next_measurement().await
    }
}
