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
//! In order to tailor data rate, noise, response time and current consumption to the needs of the user, a
//! variety of oversampling modes, filter modes and data rates can be selected.
//!
//! ## Usage
//!
//! ```
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::bosch::bme280::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp280::BMP280;
//! use embedded_devices::devices::bosch::bme280::address::Address;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bmp280 = BMP280::new_i2c(i2c, Address::Primary);
//! bmp280.init(&mut Delay).await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = bmp280.measure(&mut Delay).await?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_registers::RegisterInterface;
use uom::si::rational32::{Pressure, ThermodynamicTemperature};

use super::bme280::{
    registers::{BurstMeasurementsPT, Config, ControlMeasurement, IIRFilter, Oversampling, SensorMode},
    BME280Common, Error,
};

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current pressure or None if the sensor reported and invalid value
    pub pressure: Option<Pressure>,
}

/// The BMP280 is a combined digital pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
pub type BMP280<I> = BME280Common<I, false>;

/// Common configuration values for the BMP280 sensor.
/// The power-on-reset default is to set all oversampling settings to 1X
/// and use no IIR filter.
#[derive(Debug, Clone, Default)]
pub struct Configuration {
    /// The oversampling rate for temperature mesurements
    temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pressure_oversampling: Oversampling,
    /// The iir filter to use
    iir_filter: IIRFilter,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I: RegisterInterface> BME280Common<I, false> {
    /// Configures common sensor settings. Sensor must be in sleep mode for this to work.
    /// Check sensor mode beforehand and call [`Self::reset`] if necessary. To configure
    /// advanced settings, please directly update the respective registers.
    pub async fn configure<D: hal::delay::DelayNs>(&mut self, config: &Configuration) -> Result<(), Error<I::Error>> {
        self.write_register(
            &ControlMeasurement::default()
                .with_temperature_oversampling(config.temperature_oversampling)
                .with_pressure_oversampling(config.pressure_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        let mut reg_config = self.read_register::<Config>().await.map_err(Error::Bus)?;
        reg_config.write_filter(config.iir_filter);
        self.write_register(&reg_config).await.map_err(Error::Bus)?;

        Ok(())
    }

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        self.write_register(&ControlMeasurement::default().with_sensor_mode(SensorMode::Forced))
            .await
            .map_err(Error::Bus)?;

        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await.map_err(Error::Bus)?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See chapter 9 of the datasheet for more information.
        let mut max_measurement_delay_us = 1250 + 2300 * o_t.factor();
        if o_p.factor() > 0 {
            max_measurement_delay_us += 575 + 2300 * o_p.factor();
        }

        delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self
            .read_register::<BurstMeasurementsPT>()
            .await
            .map_err(Error::Bus)?
            .read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(Error::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = cal.compensate_pressure(raw_data.pressure.pressure, t_fine);

        Ok(Measurements { temperature, pressure })
    }
}
