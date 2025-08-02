//! The SEN60 is a particulate matter (PM) sensor from Sensition's SEN6x sensor module family.
//!
//! The SEN6x sensor module family is an air quality platform that combines critical parameters
//! such as particulate matter, relative humidity, temperature, VOC, NOx and either CO2 or
//! formaldehyde, all in one compact package.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::sen60::TransportError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::sen60::{SEN60Sync, address::Address};
//! use embedded_devices::sensor::ContinuousSensorSync;
//! use uom::si::mass_concentration::microgram_per_cubic_meter;
//!
//! // Create and initialize the device
//! let mut sen60 = SEN60Sync::new_i2c(delay, i2c, Address::Default);
//! sen60.init()?;
//! sen60.start_measuring()?;
//!
//! // [...] wait ~1h for PM results to stabilize
//! // Read measurements
//! let measurement = sen60.next_measurement()?;
//! let pm1 = measurement.pm1_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm2_5 = measurement.pm2_5_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm4 = measurement.pm4_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm10 = measurement.pm10_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! println!("Current measurement: {:?} µg/m³ PM1, {:?} µg/m³ PM2.5, {:?} µg/m³ PM4, {:?} µg/m³ PM10", pm1, pm2_5, pm4, pm10);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::sen60::TransportError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::sen60::{SEN60Async, address::Address};
//! use embedded_devices::sensor::ContinuousSensorAsync;
//! use uom::si::mass_concentration::microgram_per_cubic_meter;
//!
//! // Create and initialize the device
//! let mut sen60 = SEN60Async::new_i2c(delay, i2c, Address::Default);
//! sen60.init().await?;
//! sen60.start_measuring().await?;
//!
//! // [...] wait ~1h for PM results to stabilize
//! // Read measurements
//! let measurement = sen60.next_measurement().await?;
//! let pm1 = measurement.pm1_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm2_5 = measurement.pm2_5_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm4 = measurement.pm4_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! let pm10 = measurement.pm10_concentration.unwrap().get::<microgram_per_cubic_meter>();
//! println!("Current measurement: {:?} µg/m³ PM1, {:?} µg/m³ PM2.5, {:?} µg/m³ PM4, {:?} µg/m³ PM10", pm1, pm2_5, pm4, pm10);
//! # Ok(())
//! # }
//! # }
//! ```

use self::commands::{
    DeviceReset, GetDataReady, ReadMeasuredValuesMassConcentrationOnly, StartContinuousMeasurement, StopMeasurement,
};
use commands::DataReadyStatus;
use embedded_devices_derive::{forward_command_fns, sensor};
use uom::si::f64::MassConcentration;

use super::commands::Crc8Error;

pub mod address;
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
}

/// The SEN60 is a particulate matter (PM) sensor from Sensition's SEN6x sensor module family.
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
pub struct SEN60<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
}

pub trait SEN60Command {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> SEN60<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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
#[sensor(Pm1Concentration, Pm2_5Concentration, Pm4Concentration, Pm10Concentration)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> SEN60<D, I> {
    /// Initializes the sensor by stopping any ongoing measurement, and resetting the device.
    pub async fn init(&mut self) -> Result<(), TransportError<I::BusError>> {
        use crate::device::ResettableDevice;

        // Datasheet specifies 100ms before I2C communication may be started
        self.delay.delay_ms(100).await;
        self.reset().await?;

        Ok(())
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
    for SEN60<D, I>
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
    for SEN60<D, I>
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
        let measurement = self.execute::<ReadMeasuredValuesMassConcentrationOnly>(()).await?;
        Ok(Some(Measurement {
            pm1_concentration: (measurement.read_raw_mass_concentration_pm1() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm1()),
            pm2_5_concentration: (measurement.read_raw_mass_concentration_pm2_5() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm2_5()),
            pm4_concentration: (measurement.read_raw_mass_concentration_pm4() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm4()),
            pm10_concentration: (measurement.read_raw_mass_concentration_pm10() != u16::MAX)
                .then(|| measurement.read_mass_concentration_pm10()),
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
                return self.current_measurement().await.map(Option::unwrap);
            }
            self.delay.delay_ms(100).await;
        }
    }
}
