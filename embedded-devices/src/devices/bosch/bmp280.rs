//! # BMP280
//!
//! The BMP280 is a combined digital pressure and temperature sensor based on proven
//! sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package with
//! a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
//! consumption allow the implementation in battery driven devices such as handsets, GPS modules or
//! watches. The BMP280 is register and performance compatible to the Bosch Sensortec BMP280 digital
//! pressure sensor.
//!
//! The BMP280 achieves high performance in all applications requiring temperature and pressure
//! measurement. These emerging applications of home automation control, in-door navigation, fitness as
//! well as GPS refinement require a high accuracy and a low TCO at the same time.
//!
//! - The pressure sensor is an absolute barometric pressure sensor with extremely high accuracy and
//!   resolution and drastically lower noise than the Bosch Sensortec BMP180.
//! - The integrated temperature sensor has been optimized for lowest noise and highest resolution. Its
//!   output is used for temperature compensation of the pressure sensor and can also be
//!   used for estimation of the ambient temperature.
//!
//! The sensor provides both SPI and I²C interfaces and can be supplied using 1.71 to 3.6 V for the
//! sensor supply V DD and 1.2 to 3.6 V for the interface supply V DDIO. Measurements can be triggered by
//! the host or performed in regular intervals. When the sensor is disabled, current consumption drops to
//! 0.1 μA.
//!
//! BMP280 can be operated in three power modes:
//! - sleep mode
//! - normal mode
//! - forced mode
//!
//! In order to tailor data rate, noise, response time and current consumption to the needs of the user, a
//! variety of oversampling modes, filter modes and data rates can be selected.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::bosch::bme280::InitError<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp280::{BMP280Sync, Configuration};
//! use embedded_devices::devices::bosch::bme280::address::Address;
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::pressure::pascal;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bmp280 = BMP280Sync::new_i2c(delay, i2c, Address::Primary);
//! bmp280.init()?;
//! // Enable sensors
//! bmp280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! })?;
//!
//! // Read measurement
//! let measurement = bmp280.measure().unwrap();
//! let temp = measurement.temperature.get::<degree_celsius>();
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
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::bosch::bme280::InitError<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp280::{BMP280Async, Configuration};
//! use embedded_devices::devices::bosch::bme280::address::Address;
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::pressure::pascal;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bmp280 = BMP280Async::new_i2c(delay, i2c, Address::Primary);
//! bmp280.init().await?;
//! // Enable sensors
//! bmp280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! }).await?;
//!
//! // Read measurement
//! let measurement = bmp280.measure().await.unwrap();
//! let temp = measurement.temperature.get::<degree_celsius>();
//! let pressure = measurement.pressure.expect("should be enabled").get::<pascal>();
//! println!("Current measurement: {:?}°C, {:?} Pa", temp, pressure);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::sensor;
use embedded_registers::TransportError;
use uom::si::f64::{Pressure, ThermodynamicTemperature};

use super::bme280::{
    registers::{BurstMeasurementsPT, Config, ControlMeasurement, IIRFilter, Oversampling, SensorMode},
    BME280CommonAsync, BME280CommonSync, MeasurementError,
};

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Current temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
    /// Current pressure or None if the sensor reported and invalid value
    #[measurement(Pressure)]
    pub pressure: Option<Pressure>,
}

/// The BMP280 is a combined digital pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
#[cfg(feature = "sync")]
pub type BMP280Sync<D, I> = BME280CommonSync<D, I, false>;

/// The BMP280 is a combined digital pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
#[cfg(feature = "async")]
pub type BMP280Async<D, I> = BME280CommonAsync<D, I, false>;

/// Common configuration values for the BMP280 sensor.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// The oversampling rate for temperature mesurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pub pressure_oversampling: Oversampling,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            temperature_oversampling: Oversampling::X_1,
            pressure_oversampling: Oversampling::X_1,
            iir_filter: IIRFilter::Disabled,
        }
    }
}

#[sensor(Temperature, Pressure)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> BME280Common<D, I, false> {
    /// Configures common sensor settings. Sensor must be in sleep mode for this to work.
    /// Check sensor mode beforehand and call [`Self::reset`] if necessary. To configure
    /// advanced settings, please directly update the respective registers.
    pub async fn configure(&mut self, config: Configuration) -> Result<(), TransportError<(), I::BusError>> {
        self.write_register(
            ControlMeasurement::default()
                .with_temperature_oversampling(config.temperature_oversampling)
                .with_pressure_oversampling(config.pressure_oversampling),
        )
        .await?;

        let mut reg_config = self.read_register::<Config>().await?;
        reg_config.write_filter(config.iir_filter);
        self.write_register(reg_config).await?;

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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::OneshotSensor
    for BME280Common<D, I, false>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read and compensate the
    /// returned raw data using the calibration data.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        self.write_register(reg_ctrl_m.with_sensor_mode(SensorMode::Forced))
            .await?;

        // Use current oversampling config to determine required measurement delay
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See chapter 9 of the datasheet for more information.
        let mut max_measurement_delay_us = 1250 + 2300 * o_t.factor();
        if o_p.factor() > 0 {
            max_measurement_delay_us += 575 + 2300 * o_p.factor();
        }

        self.delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self.read_register::<BurstMeasurementsPT>().await?.read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = cal.compensate_pressure(raw_data.pressure.pressure, t_fine);

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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::ContinuousSensor
    for BME280Common<D, I, false>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        self.write_register(reg_ctrl_m.with_sensor_mode(SensorMode::Normal))
            .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        self.write_register(reg_ctrl_m.with_sensor_mode(SensorMode::Sleep))
            .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        let reg_config = self.read_register::<Config>().await?;
        let t_standby_us = reg_config.read_standby_time().time_us();

        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See chapter 9 of the datasheet for more information.
        let mut t_measure_us = 1250 + 2300 * o_t.factor();
        if o_p.factor() > 0 {
            t_measure_us += 575 + 2300 * o_p.factor();
        }
        Ok(t_measure_us + t_standby_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        let raw_data = self.read_register::<BurstMeasurementsPT>().await?.read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = (o_p != Oversampling::Disabled)
            .then(|| cal.compensate_pressure(raw_data.pressure.pressure, t_fine))
            .flatten();

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
