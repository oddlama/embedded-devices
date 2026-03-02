//! The STCC4 is Sensirion's miniature CO₂ sensor based on the thermal conductivity measurement
//! principle combined with CMOSens® technology. It features an integrated SHT4x temperature and
//! humidity sensor for on-board compensation and supports continuous, single-shot, and sleep modes.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::stcc4::InitError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::stcc4::{STCC4Sync, address::Address};
//! use embedded_devices::sensor::ContinuousSensorSync;
//! use uom::si::{
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut stcc4 = STCC4Sync::new_i2c(delay, i2c, Address::Default);
//! stcc4.init()?;
//! stcc4.start_measuring()?;
//!
//! // Read measurements
//! let measurement = stcc4.next_measurement()?;
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
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::stcc4::InitError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::stcc4::{STCC4Async, address::Address};
//! use embedded_devices::sensor::ContinuousSensorAsync;
//! use uom::si::{
//!    ratio::{part_per_million, percent},
//!    thermodynamic_temperature::degree_celsius,
//! };
//!
//! // Create and initialize the device
//! let mut stcc4 = STCC4Async::new_i2c(delay, i2c, Address::Default);
//! stcc4.init().await?;
//! stcc4.start_measuring().await?;
//!
//! // Read measurements
//! let measurement = stcc4.next_measurement().await?;
//! let humidity = measurement.relative_humidity.get::<percent>();
//! let temperature = measurement.temperature.get::<degree_celsius>();
//! let co2 = measurement.co2_concentration.get::<part_per_million>();
//! println!("Current measurement: {:?}%RH, {:?}°C, {:?}ppm CO₂", humidity, temperature, co2);
//! # Ok(())
//! # }
//! # }
//! ```

use self::commands::{
    ExitSleepMode, GetProductId, MeasureSingleShot, PerformFactoryReset, PerformForcedRecalibration, ReadMeasurement,
    StartContinuousMeasurement, StopContinuousMeasurement, TargetCo2Concentration,
};
use embedded_devices_derive::{forward_command_fns, sensor};
use uom::si::f64::{Ratio, ThermodynamicTemperature};

use super::commands::Crc8Error;

pub mod address;
pub mod commands;

/// The expected product ID for the STCC4.
const EXPECTED_PRODUCT_ID: u32 = 0x0901_018A;

/// Any CRC or bus related error
pub type TransportError<E> = embedded_interfaces::TransportError<Crc8Error, E>;

/// Errors that can occur during device initialization
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, thiserror::Error)]
pub enum InitError<BusError> {
    /// Transport error
    #[error("transport error")]
    Transport(#[from] TransportError<BusError>),
    /// The product ID read from the device did not match the expected STCC4 product ID
    #[error("invalid product ID {0:#010X}, expected {EXPECTED_PRODUCT_ID:#010X}")]
    InvalidProductId(u32),
}

/// Measurement data from the STCC4 sensor
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Current CO₂ concentration
    #[measurement(Co2Concentration)]
    pub co2_concentration: Ratio,
    /// Ambient temperature from the internal SHT4x sensor
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
    /// Ambient relative humidity from the internal SHT4x sensor
    #[measurement(RelativeHumidity)]
    pub relative_humidity: Ratio,
    /// Sensor status word. Bit 14 (2nd MSB of high byte) = 1 when in testing mode.
    pub status: u16,
}

/// The STCC4 is Sensirion's miniature CO₂ sensor based on the thermal conductivity principle
/// and CMOSens® technology.
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
pub struct STCC4<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> {
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
impl<D, I> STCC4<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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

/// Marker trait for commands that can be executed on the STCC4.
pub trait STCC4Command {}

#[forward_command_fns]
#[sensor(Co2Concentration, Temperature, RelativeHumidity)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), CommandInterface,),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> STCC4<D, I> {
    /// Initializes the sensor by waking it up, stopping any ongoing measurement, and verifying
    /// the product ID.
    ///
    /// # Power-on timing
    ///
    /// The datasheet requires at least 10 ms between VDD reaching the operating voltage and
    /// the first I2C command. The caller must ensure this delay has elapsed before calling
    /// `init`. Calling `init` immediately after `new_i2c` without an external delay may result
    /// in a failed initialization.
    pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
        // Wake up from sleep if needed (NACK is expected, ignore error)
        let _ = self.execute::<ExitSleepMode>(()).await;
        // Stop any ongoing measurement (ignore error if already idle)
        let _ = self.execute::<StopContinuousMeasurement>(()).await;

        // Verify product identity
        let pid = self.execute::<GetProductId>(()).await?;
        let id = u32::from_be_bytes(pid.read_product_id());
        if id != EXPECTED_PRODUCT_ID {
            return Err(InitError::InvalidProductId(id));
        }

        Ok(())
    }

    /// Performs forced recalibration (FRC) of the CO₂ signal.
    ///
    /// Operate the sensor for at least 30 s in continuous mode (or 30 single-shot measurements)
    /// before issuing FRC. After stopping continuous measurement, wait the specified execution time
    /// before calling this function.
    ///
    /// Returns `None` if the recalibration failed, otherwise the signed correction in ppm.
    pub async fn perform_forced_recalibration(
        &mut self,
        target_co2_concentration: Ratio,
    ) -> Result<Option<Ratio>, TransportError<I::BusError>> {
        let result = self
            .execute::<PerformForcedRecalibration>(
                TargetCo2Concentration::default().with_target_co2_concentration(target_co2_concentration),
            )
            .await?;

        Ok((result.read_raw_correction() != u16::MAX).then(|| result.read_correction()))
    }

    /// Resets the FRC and ASC algorithm history and re-enables the initial bypass phase.
    ///
    /// Returns `true` if the factory reset succeeded, `false` if it failed.
    pub async fn perform_factory_reset(&mut self) -> Result<bool, TransportError<I::BusError>> {
        let result = self.execute::<PerformFactoryReset>(()).await?;
        Ok(result.read_result() != u16::MAX)
    }

    /// Wakes the sensor from sleep mode into idle mode.
    ///
    /// The STCC4 does not acknowledge the wake-up command, so any NACK is silently ignored.
    /// After calling this, wait at least 5 ms before communicating with the sensor.
    pub async fn exit_sleep_mode(&mut self) {
        let _ = self.execute::<ExitSleepMode>(()).await;
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
    for STCC4<D, I>
{
    type Error = TransportError<I::BusError>;

    /// Resets the sensor by waking it up, stopping any ongoing measurement, and verifying
    /// it is alive by reading the product ID.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        // Wake up from sleep if needed (NACK is expected, ignore error)
        let _ = self.execute::<ExitSleepMode>(()).await;
        // Stop any ongoing measurement (ignore error if already idle)
        let _ = self.execute::<StopContinuousMeasurement>(()).await;
        // Verify sensor is responsive
        self.execute::<GetProductId>(()).await?;
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
    for STCC4<D, I>
{
    type Error = TransportError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous CO₂ measurement with a 1 s sampling interval.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StartContinuousMeasurement>(()).await?;
        Ok(())
    }

    /// Stops continuous measurement and returns the sensor to idle mode.
    ///
    /// This command takes up to 1200 ms to complete.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        self.execute::<StopContinuousMeasurement>(()).await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds (1 s).
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        Ok(1_000_000)
    }

    /// The STCC4 does not support a GetDataReady command. This always returns `true`.
    ///
    /// Since data availability cannot be checked without attempting a read, callers should
    /// ensure that `measurement_interval_us` has elapsed since the last measurement
    /// before calling `next_measurement` or `current_measurement`.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    /// Attempts to read the most recent measurement.
    ///
    /// The STCC4 responds with a NACK if no measurement data is available yet, which surfaces
    /// as a bus error. Callers should wait at least `measurement_interval_us` after
    /// calling `start_measuring` before reading.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let m = self.execute::<ReadMeasurement>(()).await?;
        Ok(Some(Measurement {
            co2_concentration: m.read_co2_concentration(),
            temperature: m.read_temperature(),
            relative_humidity: m.read_relative_humidity(),
            status: m.read_status(),
        }))
    }

    /// Attempts to read the next measurement without polling or retrying.
    ///
    /// The STCC4 has no GetDataReady command and signals data unavailability via NACK, which
    /// surfaces as a transport error. The caller is responsible for timing — wait at least
    /// `measurement_interval_us` since the last successful read before calling this.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        let m = self.execute::<ReadMeasurement>(()).await?;
        Ok(Measurement {
            co2_concentration: m.read_co2_concentration(),
            temperature: m.read_temperature(),
            relative_humidity: m.read_relative_humidity(),
            status: m.read_status(),
        })
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        CommandInterface,
        OneshotSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::commands::CommandInterface> crate::sensor::OneshotSensor
    for STCC4<D, I>
{
    type Error = TransportError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a single-shot measurement.
    ///
    /// Wakes the sensor from sleep (NACK ignored), triggers a single-shot measurement (waits
    /// 500 ms internally), then reads and returns the measurement data.
    ///
    /// # Initial operation / bypass phase
    ///
    /// During the **initial operation period** (first ~1 hour of cumulative sensor use after
    /// manufacture or [`PerformFactoryReset`]), the sensor outputs a fixed placeholder value of
    /// **390 ppm** for the **first 2 single-shot measurements** of each power-on. Power-cycling
    /// resets this counter, so a board that is powered off between measurements will **always**
    /// return 390 ppm until the initial operation period is complete.
    ///
    /// To obtain a real reading before initial operation is complete, discard the first two
    /// measurements and use the third:
    ///
    /// ```rust,ignore
    /// let _ = sensor.measure().await; // discard bypass measurement 1
    /// let _ = sensor.measure().await; // discard bypass measurement 2
    /// let measurement = sensor.measure().await?; // real reading (~1.5 s after power-on)
    /// ```
    ///
    /// The initial operation period ends once the first ASC state has been saved, which requires
    /// approximately 1 hour of cumulative operation (360 single-shot measurements at a 10 s
    /// interval, or 1 hour of continuous measurement mode). After that, this behavior no longer
    /// occurs on subsequent power cycles.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        // Wake up from sleep if needed (NACK is expected, ignore error)
        let _ = self.execute::<ExitSleepMode>(()).await;
        // Trigger single-shot measurement; the executor already waits 500 ms
        self.execute::<MeasureSingleShot>(()).await?;
        // Data is ready after MeasureSingleShot completes
        let m = self.execute::<ReadMeasurement>(()).await?;
        Ok(Measurement {
            co2_concentration: m.read_co2_concentration(),
            temperature: m.read_temperature(),
            relative_humidity: m.read_relative_humidity(),
            status: m.read_status(),
        })
    }
}
