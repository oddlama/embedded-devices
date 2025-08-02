//! # BMP390
//!
//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
//! module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
//! mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
//! driven devices such as mobile phones, GPS modules or watches.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::bosch::bmp390::InitError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp390::{BMP390Sync, address::Address, Configuration};
//! use embedded_devices::devices::bosch::bmp390::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::pressure::pascal;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bmp390 = BMP390Sync::new_i2c(delay, i2c, Address::Primary);
//! bmp390.init()?;
//! // Enable sensors
//! bmp390.configure(Configuration {
//!     temperature_oversampling: Some(Oversampling::X_32),
//!     pressure_oversampling: Some(Oversampling::X_32),
//!     iir_filter: IIRFilter::Disabled,
//! })?;
//!
//! // Read the current temperature in °C
//! let measurement = bmp390.measure().unwrap();
//! let temp = measurement.temperature.expect("should be enabled").get::<degree_celsius>();
//! let pressure = measurement.pressure.expect("should be enabled").get::<pascal>();
//! println!("Current measurement: {:?}°C, {:?} Pa", temp, pressure);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::bosch::bmp390::InitError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp390::{BMP390Async, address::Address, Configuration};
//! use embedded_devices::devices::bosch::bmp390::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::pressure::pascal;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bmp390 = BMP390Async::new_i2c(delay, i2c, Address::Primary);
//! bmp390.init().await?;
//! // Enable sensors
//! bmp390.configure(Configuration {
//!     temperature_oversampling: Some(Oversampling::X_32),
//!     pressure_oversampling: Some(Oversampling::X_32),
//!     iir_filter: IIRFilter::Disabled,
//! }).await?;
//!
//! // Read the current temperature in °C
//! let measurement = bmp390.measure().await.unwrap();
//! let temp = measurement.temperature.expect("should be enabled").get::<degree_celsius>();
//! let pressure = measurement.pressure.expect("should be enabled").get::<pascal>();
//! println!("Current measurement: {:?}°C, {:?} Pa", temp, pressure);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::{forward_register_fns, sensor};
use embedded_interfaces::TransportError;
use uom::si::f64::{Pressure, ThermodynamicTemperature};
use uom::si::pressure::pascal;
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use crate::utils::from_bus_error;

use self::address::Address;
use self::registers::{
    BurstMeasurements, ChipId, Cmd, Config, DataRateControl, IIRFilter, Oversampling, OversamplingControl,
    PowerControl, SensorMode, TrimmingCoefficients, TrimmingCoefficientsUnpacked,
};

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum InitError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Invalid chip id was encountered in `init`
    #[error("invalid chip id {0:#02x}")]
    InvalidChip(u8),
}

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum MeasurementError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// The calibration data was not yet read from the device, but a measurement was requested. Call `init` or `calibrate` first.
    #[error("not yet calibrated")]
    NotCalibrated,
}

from_bus_error!(InitError);
from_bus_error!(MeasurementError);

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Current temperature
    #[measurement(Temperature)]
    pub temperature: Option<ThermodynamicTemperature>,
    /// Current pressure or None if the sensor reported and invalid value
    #[measurement(Pressure)]
    pub pressure: Option<Pressure>,
}

/// Common configuration values for the BME280 sensor.
/// The power-on-reset default is to set all oversampling settings to 1X
/// and use no IIR filter.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// The oversampling rate for temperature mesurements or None to disable this measurement
    pub temperature_oversampling: Option<Oversampling>,
    /// The oversampling rate for pressure mesurements or None to disable this measurement
    pub pressure_oversampling: Option<Oversampling>,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            temperature_oversampling: Some(Oversampling::X_1),
            pressure_oversampling: Some(Oversampling::X_1),
            iir_filter: IIRFilter::Disabled,
        }
    }
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure
#[derive(Debug, Clone, Copy)]
pub(super) struct TFine(i32);

#[derive(Debug)]
pub(super) struct CalibrationData(TrimmingCoefficientsUnpacked);

impl CalibrationData {
    pub(super) fn compensate_temperature(&self, uncompensated: u32) -> (ThermodynamicTemperature, TFine) {
        let v1: u64 = (uncompensated as u64) - ((self.0.par_t1 as u64) << 8);
        let v2: u64 = self.0.par_t2 as u64 * v1;
        let v3: u64 = v1 * v1;
        let v4: i64 = (v3 as i64) * (self.0.par_t3 as i64);
        let v5: i64 = ((v2 as i64) << 18) + v4;
        let t_fine: i32 = (v5 >> 32) as i32;
        let temperature = (t_fine * 25) as f64 / (100 << 14) as f64;

        (
            ThermodynamicTemperature::new::<degree_celsius>(temperature),
            TFine(t_fine),
        )
    }

    pub(super) fn compensate_pressure(&self, uncompensated: u32, t_fine: TFine) -> Pressure {
        let t_fine = t_fine.0;

        let v1 = t_fine as i64 * t_fine as i64;
        let v3 = ((v1 >> 6) * t_fine as i64) >> 8;
        let v4 = (self.0.par_p8 as i64 * v3) >> 5;
        let v5 = (self.0.par_p7 as i64 * v1) << 4;
        let v6 = (self.0.par_p6 as i64 * t_fine as i64) << 22;
        let offset = ((self.0.par_p5 as i64) << 47) + v4 + v5 + v6;
        let v2 = ((self.0.par_p4 as i64) * v3) >> 5;
        let v4 = (self.0.par_p3 as i64 * v1) << 2;
        let v5 = ((self.0.par_p2 as i64 - 16384) * t_fine as i64) << 21;
        let sensitivity = (((self.0.par_p1 as i64 - 16384) << 46) + v2 + v4 + v5) >> 24;
        let v1 = (sensitivity * uncompensated as i64) >> 13;
        let v2 = (self.0.par_p10 as i64) * (t_fine as i64);
        let v3 = v2 + ((self.0.par_p9 as i64) << 16);
        let v4 = (v3 * uncompensated as i64) >> 13;
        let v5 = (v4 * uncompensated as i64) >> 9;
        let v6 = uncompensated as i64 * uncompensated as i64;
        let v2 = ((self.0.par_p11 as i64) * v6) >> 16;
        let v3 = (v2 * uncompensated as i64) >> 7;
        let v4 = (offset / 4) + v1 + v5 + v3;
        let pressure = ((v4 >> 32) * 25) as i32;
        let pressure = pressure as f64 / (100 << 8) as f64;
        Pressure::new::<pascal>(pressure)
    }
}

/// The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
/// module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
/// mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
/// driven devices such as mobile phones, GPS modules or watches.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BMP390<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// Calibration data
    pub(super) calibration_data: Option<CalibrationData>,
}

pub trait BMP390Register {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> BMP390<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: Address) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::i2c::I2cDevice::new(interface, address.into()),
            calibration_data: None,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> BMP390<D, embedded_interfaces::spi::SpiDevice<I>>
where
    I: hal::spi::r#SpiDevice,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_spi(delay: D, interface: I) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::spi::SpiDevice::new(interface),
            calibration_data: None,
        }
    }
}

#[forward_register_fns]
#[sensor(Temperature, Pressure)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> BMP390<D, I> {
    /// Initialize the sensor by performing a soft-reset, verifying its chip id
    /// and reading calibration data.
    ///
    /// Beware that by default all internal sensors are disabled. Please
    /// call [`Self::configure`] after initialization to enable the sensors,
    /// otherwise measurement may return valid-looking but static values.
    pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
        use crate::device::ResettableDevice;

        // Soft-reset device
        self.reset().await?;

        // Verify chip id
        let chip = self.read_register::<ChipId>().await?.read_chip();
        if let self::registers::Chip::Invalid(x) = chip {
            return Err(InitError::InvalidChip(x));
        }

        // Read calibration data
        self.calibrate().await?;
        Ok(())
    }

    /// Reads the calibration registers from the device to
    /// compensate measurements. It is required to call this once
    /// before taking any measurements. Calling [`Self::init`] will
    /// automatically do this.
    pub async fn calibrate(&mut self) -> Result<(), TransportError<(), I::BusError>> {
        let coefficients = self.read_register::<TrimmingCoefficients>().await?.unpack();
        self.calibration_data = Some(CalibrationData(coefficients));

        Ok(())
    }

    /// Configures common sensor settings. Sensor must be in sleep mode for this to work. To
    /// configure advanced settings, please directly update the respective registers.
    pub async fn configure(&mut self, config: Configuration) -> Result<(), TransportError<(), I::BusError>> {
        self.write_register(
            PowerControl::default()
                .with_temperature_enable(config.temperature_oversampling.is_some())
                .with_pressure_enable(config.pressure_oversampling.is_some()),
        )
        .await?;

        let mut oversampling = OversamplingControl::default();
        if let Some(ov) = config.temperature_oversampling {
            oversampling.write_temperature_oversampling(ov);
        }
        if let Some(ov) = config.pressure_oversampling {
            oversampling.write_pressure_oversampling(ov);
        }
        self.write_register(oversampling).await?;
        self.write_register(Config::default().with_iir_filter(config.iir_filter))
            .await?;

        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::device::ResettableDevice
    for BMP390<D, I>
{
    type Error = TransportError<(), I::BusError>;

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 10ms, which is automatically awaited before allowing further communication.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        self.write_register(self::registers::Command::default().with_command(Cmd::Reset))
            .await?;
        self.delay.delay_ms(10).await;
        Ok(())
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
    for BMP390<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let power_ctrl = self.read_register::<PowerControl>().await?;
        self.write_register(power_ctrl.with_sensor_mode(SensorMode::Forced))
            .await?;

        let temperature_enable = power_ctrl.read_temperature_enable();
        let pressure_enable = power_ctrl.read_pressure_enable();

        if !temperature_enable && !pressure_enable {
            return Ok(Measurement {
                temperature: None,
                pressure: None,
            });
        }

        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_m = self.read_register::<OversamplingControl>().await?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See section 3.9.2 of the datasheet for more information.
        let max_measurement_delay_us = 234 + (392 + 2020 * o_p.factor()) + (163 + o_t.factor() * 2020);
        self.delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self.read_register::<BurstMeasurements>().await?;
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let temperature_enable = power_ctrl.read_temperature_enable();
        let pressure_enable = power_ctrl.read_pressure_enable();

        let (temperature, pressure) = if temperature_enable {
            let (temp, t_fine) = cal.compensate_temperature(raw_data.read_temperature_value());
            let press = pressure_enable.then(|| cal.compensate_pressure(raw_data.read_pressure_value(), t_fine));
            (Some(temp), press)
        } else {
            (None, None)
        };

        Ok(Measurement { temperature, pressure })
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
    for BMP390<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let power_ctrl = self.read_register::<PowerControl>().await?;
        self.write_register(power_ctrl.with_sensor_mode(SensorMode::Normal))
            .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let power_ctrl = self.read_register::<PowerControl>().await?;
        self.write_register(power_ctrl.with_sensor_mode(SensorMode::Sleep))
            .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_m = self.read_register::<OversamplingControl>().await?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See section 3.9.2 of the datasheet for more information.
        let t_measure_us = 234 + (392 + 2020 * o_p.factor()) + (163 + o_t.factor() * 2020);

        let data_rate_ctrl = self.read_register::<DataRateControl>().await?;
        let t_standby_us = data_rate_ctrl.read_data_rate().interval_us();
        Ok(t_measure_us + t_standby_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let raw_data = self.read_register::<BurstMeasurements>().await?;
        let power_ctrl = self.read_register::<PowerControl>().await?;
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let temperature_enable = power_ctrl.read_temperature_enable();
        let pressure_enable = power_ctrl.read_pressure_enable();

        let (temperature, pressure) = if temperature_enable {
            let (temp, t_fine) = cal.compensate_temperature(raw_data.read_temperature_value());
            let press = pressure_enable.then(|| cal.compensate_pressure(raw_data.read_pressure_value(), t_fine));
            (Some(temp), press)
        } else {
            (None, None)
        };

        Ok(Some(Measurement { temperature, pressure }))
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
