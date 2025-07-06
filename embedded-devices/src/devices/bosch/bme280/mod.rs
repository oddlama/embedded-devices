//! # BME280
//!
//! The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
//! sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package with
//! a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
//! consumption allow the implementation in battery driven devices such as handsets, GPS modules or
//! watches. The BME280 is register and performance compatible to the Bosch Sensortec BMP280 digital
//! pressure sensor.
//!
//! The BME280 achieves high performance in all applications requiring humidity and pressure
//! measurement. These emerging applications of home automation control, in-door navigation, fitness as
//! well as GPS refinement require a high accuracy and a low TCO at the same time.
//!
//! - The humidity sensor provides an extremely fast response time for fast context awareness applications
//!   and high overall accuracy over a wide temperature range.
//! - The pressure sensor is an absolute barometric pressure sensor with extremely high accuracy and
//!   resolution and drastically lower noise than the Bosch Sensortec BMP180.
//! - The integrated temperature sensor has been optimized for lowest noise and highest resolution. Its
//!   output is used for temperature compensation of the pressure and humidity sensors and can also be
//!   used for estimation of the ambient temperature.
//!
//! The sensor provides both SPI and I²C interfaces and can be supplied using 1.71 to 3.6 V for the
//! sensor supply V DD and 1.2 to 3.6 V for the interface supply V DDIO. Measurements can be triggered by
//! the host or performed in regular intervals. When the sensor is disabled, current consumption drops to
//! 0.1 μA.
//!
//! BME280 can be operated in three power modes:
//! - sleep mode
//! - normal mode
//! - forced mode
//!
//! In order to tailor data rate, noise, response time and current consumption to the needs of the user,
//! a variety of oversampling modes, filter modes and data rates can be selected.
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
//! use embedded_devices::devices::bosch::bme280::{BME280Sync, Configuration, address::Address};
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::pressure::pascal;
//! use uom::si::ratio::percent;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bme280 = BME280Sync::new_i2c(delay, i2c, Address::Primary);
//! bme280.init()?;
//! // Enable sensors
//! bme280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     humidity_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! })?;
//!
//! // Read measurement
//! let measurement = bme280.measure().unwrap();
//! let temp = measurement.temperature.get::<degree_celsius>();
//! let pressure = measurement.pressure.expect("should be enabled").get::<pascal>();
//! let humidity = measurement.humidity.expect("should be enabled").get::<percent>();
//! println!("Current measurement: {:?}°C, {:?} Pa, {:?}%RH", temp, pressure, humidity);
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
//! use embedded_devices::devices::bosch::bme280::{BME280Async, Configuration, address::Address};
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::pressure::pascal;
//! use uom::si::ratio::percent;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bme280 = BME280Async::new_i2c(delay, i2c, Address::Primary);
//! bme280.init().await?;
//! // Enable sensors
//! bme280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     humidity_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! }).await?;
//!
//! // Read measurement
//! let measurement = bme280.measure().await.unwrap();
//! let temp = measurement.temperature.get::<degree_celsius>();
//! let pressure = measurement.pressure.expect("should be enabled").get::<pascal>();
//! let humidity = measurement.humidity.expect("should be enabled").get::<percent>();
//! println!("Current measurement: {:?}°C, {:?} Pa, {:?}%RH", temp, pressure, humidity);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::{device, device_impl, sensor};
use embedded_registers::TransportError;
use uom::si::f64::{Pressure, Ratio, ThermodynamicTemperature};
use uom::si::pressure::pascal;
use uom::si::ratio::percent;
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use crate::utils::from_bus_error;

use self::address::Address;
use self::registers::{
    BurstMeasurementsPTH, Config, ControlHumidity, ControlMeasurement, IIRFilter, Id, Oversampling, SensorMode,
    TrimmingParameters1, TrimmingParameters2,
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
    pub temperature: ThermodynamicTemperature,
    /// Current pressure or None if the sensor reported and invalid value
    #[measurement(Pressure)]
    pub pressure: Option<Pressure>,
    /// Current relative humidity
    #[measurement(RelativeHumidity)]
    pub humidity: Option<Ratio>,
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure and humidity
#[derive(Debug, Clone, Copy)]
pub(super) struct TFine(i32);

/// The common base for both BME280 and BMP280.
/// For a full description and usage examples, refer to [BME280](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BME280Common<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface, const IS_BME: bool> {
    /// The delay provider
    pub(super) delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// Calibration data
    pub(super) calibration_data: Option<CalibrationData>,
}

/// The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
#[cfg(feature = "sync")]
pub type BME280Sync<D, I> = BME280CommonSync<D, I, true>;

/// The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
#[cfg(feature = "async")]
pub type BME280Async<D, I> = BME280CommonAsync<D, I, true>;

/// Common configuration values for the BME280 sensor.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// The oversampling rate for temperature measurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure measurements
    pub pressure_oversampling: Oversampling,
    /// The oversampling rate for humidity measurements
    pub humidity_oversampling: Oversampling,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            temperature_oversampling: Oversampling::X_1,
            pressure_oversampling: Oversampling::X_1,
            humidity_oversampling: Oversampling::X_1,
            iir_filter: IIRFilter::Disabled,
        }
    }
}

#[derive(Debug)]
pub(super) struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

impl CalibrationData {
    pub fn new(params1: self::registers::TrimmingParameters1, params2: self::registers::TrimmingParameters2) -> Self {
        let params1 = params1.read_all();
        let params2 = params2.read_all();
        Self {
            dig_t1: params1.dig_t1,
            dig_t2: params1.dig_t2,
            dig_t3: params1.dig_t3,
            dig_p1: params1.dig_p1,
            dig_p2: params1.dig_p2,
            dig_p3: params1.dig_p3,
            dig_p4: params1.dig_p4,
            dig_p5: params1.dig_p5,
            dig_p6: params1.dig_p6,
            dig_p7: params1.dig_p7,
            dig_p8: params1.dig_p8,
            dig_p9: params1.dig_p9,
            dig_h1: params1.dig_h1,
            dig_h2: params2.dig_h2,
            dig_h3: params2.dig_h3,
            dig_h4: (params2.dig_h4_msb as i16 * 16) | ((params2.dig_h5_lsn_h4_lsn as i16) & 0x0F),
            dig_h5: (params2.dig_h5_msb as i16 * 16) | (((params2.dig_h5_lsn_h4_lsn as i16) & 0xF0) >> 4),
            dig_h6: params2.dig_h6,
        }
    }

    pub(super) fn compensate_temperature(&self, uncompensated: u32) -> (ThermodynamicTemperature, TFine) {
        let dig_t1 = self.dig_t1 as i32;
        let dig_t2 = self.dig_t2 as i32;
        let dig_t3 = self.dig_t3 as i32;

        let var1 = (uncompensated >> 3) as i32 - (dig_t1 << 1);
        let var1 = (var1 * dig_t2) >> 11;
        let var2 = (uncompensated >> 4) as i32 - dig_t1;
        let var2 = (((var2 * var2) >> 12) * dig_t3) >> 14;
        let t_fine = var1 + var2;
        let temperature = (t_fine * 5 + 128) as f64 / (100.0 * 256.0);

        (
            ThermodynamicTemperature::new::<degree_celsius>(temperature),
            TFine(t_fine),
        )
    }

    pub(super) fn compensate_pressure(&self, uncompensated: u32, t_fine: TFine) -> Option<Pressure> {
        let t_fine = t_fine.0;

        let dig_p1 = self.dig_p1 as i64;
        let dig_p2 = self.dig_p2 as i64;
        let dig_p3 = self.dig_p3 as i64;
        let dig_p4 = self.dig_p4 as i64;
        let dig_p5 = self.dig_p5 as i64;
        let dig_p6 = self.dig_p6 as i64;
        let dig_p7 = self.dig_p7 as i64;
        let dig_p8 = self.dig_p8 as i64;
        let dig_p9 = self.dig_p9 as i64;

        let var1 = t_fine as i64 - 128000;
        let var2 = var1 * var1 * dig_p6;
        let var2 = var2 + ((var1 * dig_p5) << 17);
        let var2 = var2 + (dig_p4 << 35);
        let var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12);
        let var1 = (((1i64 << 47) + var1) * dig_p1) >> 33;
        if var1 == 0 {
            return None;
        }

        let var4: i64 = 0x100000i64 - uncompensated as i64;
        let var4 = (((var4 << 31) - var2) * 3125) / var1;
        let var1 = (dig_p9 * (var4 >> 13) * (var4 >> 13)) >> 25;
        let var2 = (dig_p8 * var4) >> 19;
        let var4 = ((var4 + var1 + var2) >> 8) + (dig_p7 << 4);
        let pressure = (var4 as i32 as f64) / 256.0;
        Some(Pressure::new::<pascal>(pressure))
    }

    fn compensate_humidity(&self, uncompensated: u16, t_fine: TFine) -> Ratio {
        let t_fine = t_fine.0;

        let dig_h1 = self.dig_h1 as i32;
        let dig_h2 = self.dig_h2 as i32;
        let dig_h3 = self.dig_h3 as i32;
        let dig_h4 = self.dig_h4 as i32;
        let dig_h5 = self.dig_h5 as i32;
        let dig_h6 = self.dig_h6 as i32;

        let var1 = t_fine - 76800i32;
        let uncompensated = (uncompensated as i32) << 14;
        let var5 = (((uncompensated - (dig_h4 << 20)) - (dig_h5 * var1)) + 0x4000) >> 15;
        let var2 = (var1 * dig_h6) >> 10;
        let var3 = (var1 * dig_h3) >> 11;
        let var4 = ((var2 * (var3 + 0x8000)) >> 10) + 0x200000;
        let var2 = ((var4 * dig_h2) + 0x2000) >> 14;
        let var3 = var5 * var2;
        let var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
        let var5 = var3 - ((var4 * dig_h1) >> 4);
        let var5 = var5.clamp(0, 0x19000000);
        let var5 = var5 >> 12;

        let humidity = var5 as f64 / (1i32 << 10) as f64;
        Ratio::new::<percent>(humidity)
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I, const IS_BME: bool> BME280Common<D, embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>, IS_BME>
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
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
            calibration_data: None,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I, const IS_BME: bool> BME280Common<D, embedded_registers::spi::SpiDevice<I>, IS_BME>
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
            interface: embedded_registers::spi::SpiDevice::new(interface),
            calibration_data: None,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface, const IS_BME: bool> BME280Common<D, I, IS_BME> {
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
        let chip = self.read_register::<Id>().await?.read_chip();
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
        let params1 = self.read_register::<TrimmingParameters1>().await?;
        let params2 = self.read_register::<TrimmingParameters2>().await?;
        self.calibration_data = Some(CalibrationData::new(params1, params2));

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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface, const IS_BME: bool>
    crate::device::ResettableDevice for BME280Common<D, I, IS_BME>
{
    type Error = TransportError<(), I::BusError>;

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 2ms, which is automatically awaited before allowing further communication.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        self.write_register(self::registers::Reset::default()).await?;
        self.delay.delay_ms(2).await;
        Ok(())
    }
}

// BME280 only:

#[sensor(Temperature, Pressure, RelativeHumidity)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> BME280Common<D, I, true> {
    /// Configures common sensor settings. Sensor must be in sleep mode for this to work. To
    /// configure advanced settings, please directly update the respective registers.
    pub async fn configure(&mut self, config: Configuration) -> Result<(), TransportError<(), I::BusError>> {
        self.write_register(ControlHumidity::default().with_oversampling(config.humidity_oversampling))
            .await?;

        // This must happen after ControlHumidity, otherwise the former will not have any effect
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
    for BME280Common<D, I, true>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    ///
    /// Specific measurements will only be included if they were enabled beforehand by calling
    /// [`Self::calibrate`]. Pressure and humidity measurement specifically require
    /// temperature measurements to be enabled.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        self.write_register(reg_ctrl_m.with_sensor_mode(SensorMode::Forced))
            .await?;

        // Use current oversampling config to determine required measurement delay
        let reg_ctrl_h = self.read_register::<ControlHumidity>().await?;
        let o_h = reg_ctrl_h.read_oversampling();
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See chapter 9 of the datasheet for more information.
        let mut max_measurement_delay_us = 1250 + 2300 * o_t.factor();
        if o_p.factor() > 0 {
            max_measurement_delay_us += 575 + 2300 * o_p.factor();
        }
        if o_h.factor() > 0 {
            max_measurement_delay_us += 575 + 2300 * o_h.factor();
        }

        self.delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self.read_register::<BurstMeasurementsPTH>().await?.read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = (o_p != Oversampling::Disabled)
            .then(|| cal.compensate_pressure(raw_data.pressure.pressure, t_fine))
            .flatten();
        let humidity =
            (o_h != Oversampling::Disabled).then(|| cal.compensate_humidity(raw_data.humidity.humidity, t_fine));

        Ok(Measurement {
            temperature,
            pressure,
            humidity,
        })
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
    for BME280Common<D, I, true>
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
        let reg_ctrl_h = self.read_register::<ControlHumidity>().await?;

        let o_h = reg_ctrl_h.read_oversampling();
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See chapter 9 of the datasheet for more information.
        let mut t_measure_us = 1250 + 2300 * o_t.factor();
        if o_p.factor() > 0 {
            t_measure_us += 575 + 2300 * o_p.factor();
        }
        if o_h.factor() > 0 {
            t_measure_us += 575 + 2300 * o_h.factor();
        }
        Ok(t_measure_us + t_standby_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await?;
        let reg_ctrl_h = self.read_register::<ControlHumidity>().await?;

        let o_h = reg_ctrl_h.read_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        let raw_data = self.read_register::<BurstMeasurementsPTH>().await?.read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = (o_p != Oversampling::Disabled)
            .then(|| cal.compensate_pressure(raw_data.pressure.pressure, t_fine))
            .flatten();
        let humidity =
            (o_h != Oversampling::Disabled).then(|| cal.compensate_humidity(raw_data.humidity.humidity, t_fine));

        Ok(Some(Measurement {
            temperature,
            pressure,
            humidity,
        }))
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
