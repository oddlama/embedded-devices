// TODO add docs for bmp

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
    /// Check sensor mode beforehand and call [`reset`] if necessary. To configure
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
