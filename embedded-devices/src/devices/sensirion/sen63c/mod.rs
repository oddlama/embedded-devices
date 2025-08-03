//! The SEN63C is a particulate matter (PM), CO₂, temperature and relative humidity sensor sensor
//! from Sensition's SEN6x sensor module family.
//!
//! The SEN6x sensor module family is an air quality platform that combines critical parameters
//! such as particulate matter, relative humidity, temperature, VOC, NOx and either CO2 or
//! formaldehyde, all in one compact package.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::sen63c::TransportError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::sen63c::{SEN63CSync, address::Address};
//! use embedded_devices::sensor::ContinuousSensorSync;
//! use uom::si::{
//!    mass_concentration::microgram_per_cubic_meter,
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut sen63c = SEN63CSync::new_i2c(delay, i2c, Address::Default);
//! sen63c.init()?;
//! sen63c.start_measuring()?;
//!
//! // [...] wait ~1h for PM results to stabilize
//! // Read measurements
//! let measurement = sen63c.next_measurement()?;
//! let pm1 = measurement.pm1_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm2_5 = measurement.pm2_5_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm4 = measurement.pm4_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm10 = measurement.pm10_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let humidity = measurement.relative_humidity.unwrap().get::<percent>();
//! let temperature = measurement.temperature.unwrap().get::<degree_celsius>();
//! let co2 = measurement.co2_concentration.unwrap().get::<part_per_million>();
//! println!("Current measurement: {:?} µg/m³ PM1, {:?} µg/m³ PM2.5, {:?} µg/m³ PM4, {:?} µg/m³ PM10, {:?}%RH, {:?}°C, {:?}ppm CO₂",
//!     pm1, pm2_5, pm4, pm10, humidity, temperature, co2
//! );
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::sen63c::TransportError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::sen63c::{SEN63CAsync, address::Address};
//! use embedded_devices::sensor::ContinuousSensorAsync;
//! use uom::si::{
//!    mass_concentration::microgram_per_cubic_meter,
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut sen63c = SEN63CAsync::new_i2c(delay, i2c, Address::Default);
//! sen63c.init().await?;
//! sen63c.start_measuring().await?;
//!
//! // [...] wait ~1h for PM results to stabilize
//! // Read measurements
//! let measurement = sen63c.next_measurement().await?;
//! let pm1 = measurement.pm1_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm2_5 = measurement.pm2_5_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm4 = measurement.pm4_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm10 = measurement.pm10_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let humidity = measurement.relative_humidity.unwrap().get::<percent>();
//! let temperature = measurement.temperature.unwrap().get::<degree_celsius>();
//! let co2 = measurement.co2_concentration.unwrap().get::<part_per_million>();
//! println!("Current measurement: {:?} µg/m³ PM1, {:?} µg/m³ PM2.5, {:?} µg/m³ PM4, {:?} µg/m³ PM10, {:?}%RH, {:?}°C, {:?}ppm CO₂",
//!     pm1, pm2_5, pm4, pm10, humidity, temperature, co2
//! );
//! # Ok(())
//! # }
//! # }
//! ```

use self::commands::{
    DeviceReset, GetDataReady, PerformForcedCo2Recalibration, ReadMeasuredValues, StartContinuousMeasurement,
    StopMeasurement,
};
use embedded_devices_derive::{forward_command_fns, sensor};
use uom::si::f64::{MassConcentration, Ratio, ThermodynamicTemperature};

pub use super::sen6x::address;
use super::{
    commands::Crc8Error,
    sen6x::commands::{DataReadyStatus, TargetCo2Concentration},
};
pub mod commands;

/// Any CRC or Bus related error
pub type TransportError<E> = embedded_interfaces::TransportError<Crc8Error, E>;

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// PM1 concentration
    #[measurement(Pm1Concentration)]
    pub pm1_concentration: Option<MassConcentration>,
    /// PM2.5 concentration
    #[measurement(Pm2_5Concentration)]
    pub pm2_5_concentration: Option<MassConcentration>,
    /// PM4 concentration
    #[measurement(Pm4Concentration)]
    pub pm4_concentration: Option<MassConcentration>,
    /// PM10 concentration
    #[measurement(Pm10Concentration)]
    pub pm10_concentration: Option<MassConcentration>,
    /// Ambient relative humidity
    #[measurement(RelativeHumidity)]
    pub relative_humidity: Option<Ratio>,
    /// Ambient temperature
    #[measurement(Temperature)]
    pub temperature: Option<ThermodynamicTemperature>,
    /// Current CO₂ concentration
    #[measurement(Co2Concentration)]
    pub co2_concentration: Option<Ratio>,
}

/// The SEN63C is a particulate matter (PM), CO₂, temperature and relative humidity sensor sensor
/// from Sensition's SEN6x sensor module family.
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
pub struct SEN63C<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
}

pub trait SEN63CCommand {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> SEN63C<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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

#[forward_command_fns]
#[sensor(
    Pm1Concentration,
    Pm2_5Concentration,
    Pm4Concentration,
    Pm10Concentration,
    RelativeHumidity,
    Temperature,
    Co2Concentration
)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> SEN63C<D, I> {
    /// Initializes the sensor by stopping any ongoing measurement, and resetting the device.
    pub async fn init(&mut self) -> Result<(), TransportError<I::BusError>> {
        use crate::device::ResettableDevice;

        // Datasheet specifies 100ms before I2C communication may be started
        self.delay.delay_ms(100).await;
        self.reset().await?;

        Ok(())
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
            .execute::<PerformForcedCo2Recalibration>(
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
    for SEN63C<D, I>
{
    type Error = TransportError<I::BusError>;

    /// Resets the sensor by stopping any ongoing measurement, and resetting the device.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Try to stop measurement if it is ongoing, otherwise ignore
        let _ = self.execute::<StopMeasurement>(()).await;
        // Reset
        self.execute::<DeviceReset>(()).await?;
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
    for SEN63C<D, I>
{
    type Error = TransportError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StartContinuousMeasurement>(()).await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StopMeasurement>(()).await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        Ok(1_000_000)
    }

    /// Returns the most recent measurement.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let measurement = self.execute::<ReadMeasuredValues>(()).await?;
        Ok(Some(Measurement {
            pm1_concentration: (measurement.read_raw_mass_concentration_pm1() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm1()),
            pm2_5_concentration: (measurement.read_raw_mass_concentration_pm2_5() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm2_5()),
            pm4_concentration: (measurement.read_raw_mass_concentration_pm4() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm4()),
            pm10_concentration: (measurement.read_raw_mass_concentration_pm10() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm10()),
            relative_humidity: (measurement.read_raw_relative_humidity() != i16::MAX)
                .then(|| measurement.read_relative_humidity()),
            temperature: (measurement.read_raw_temperature() != i16::MAX).then(|| measurement.read_temperature()),
            co2_concentration: (measurement.read_raw_co2_concentration() != u16::MAX)
                .then(|| measurement.read_co2_concentration()),
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
