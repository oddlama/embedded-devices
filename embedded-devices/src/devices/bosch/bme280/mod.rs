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
//! ```rust, only_if(sync)
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::bosch::bme280::Error<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bme280::{BME280Sync, Configuration, address::Address};
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bme280 = BME280Sync::new_i2c(i2c, Address::Primary);
//! bme280.init(&mut Delay).unwrap();
//! bme280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     humidity_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! }).unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = bme280.measure(&mut Delay)?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::bosch::bme280::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bme280::{BME280Async, Configuration, address::Address};
//! use embedded_devices::devices::bosch::bme280::registers::{IIRFilter, Oversampling};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bme280 = BME280Async::new_i2c(i2c, Address::Primary);
//! bme280.init(&mut Delay).await.unwrap();
//! bme280.configure(Configuration {
//!     temperature_oversampling: Oversampling::X_16,
//!     pressure_oversampling: Oversampling::X_16,
//!     humidity_oversampling: Oversampling::X_16,
//!     iir_filter: IIRFilter::Disabled,
//! }).await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = bme280.measure(&mut Delay).await?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::ratio::percent;
use uom::si::rational32::{Pressure, Ratio, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::address::Address;
use self::registers::{
    BurstMeasurementsPTH, Chip, Config, ControlHumidity, ControlMeasurement, IIRFilter, Id, Oversampling, SensorMode,
    TrimmingParameters1, TrimmingParameters2,
};

type BME280SpiCodec = embedded_registers::spi::codecs::SimpleCodec<1, 6, 0, 7, true, 0>;
type BME280I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur when using this device
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid chip id was encountered in `init`
    InvalidChip(Chip),
    /// The calibration data was not yet read from the device, but a measurement was requested. Call `init` or `calibrate` first.
    NotCalibrated,
    /// NVM data copy is still in progress.
    NvmCopyInProgress,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current pressure or None if the sensor reported and invalid value
    pub pressure: Option<Pressure>,
    /// Current relative humidity
    pub humidity: Option<Ratio>,
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure and humidity
#[derive(Debug, Clone, Copy)]
pub(super) struct TFine(i32);

/// The common base for both BME280 and BMP280.
/// For a full description and usage examples, refer to the for the [BME280](self) and [BMP280](super::bmp280).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BME280Common<I: embedded_registers::RegisterInterface, const IS_BME: bool> {
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
pub type BME280Sync<I> = BME280CommonSync<I, true>;

/// The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
#[cfg(feature = "async")]
pub type BME280Async<I> = BME280CommonAsync<I, true>;

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
        let temperature = Rational32::new_raw(t_fine * 5 + 128, 100 * 256);

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
        let pressure = Rational32::new_raw(var4 as i32, 256);
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

        let humidity = Rational32::new_raw(var5, 1i32 << 10);
        Ratio::new::<percent>(humidity)
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, const IS_BME: bool>
    BME280Common<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, BME280I2cCodec>, IS_BME>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
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
impl<I, const IS_BME: bool> BME280Common<embedded_registers::spi::SpiDevice<I, BME280SpiCodec>, IS_BME>
where
    I: hal::spi::r#SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_spi(interface: I) -> Self {
        Self {
            interface: embedded_registers::spi::SpiDevice::new(interface),
            calibration_data: None,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface, const IS_BME: bool> BME280Common<I, IS_BME> {
    /// Initialize the sensor by performing a soft-reset, verifying its chip id
    /// and reading calibration data.
    ///
    /// Beware that by default, all internal sensors are disabled. Please
    /// call [`Self::configure`] after initialization to enable the sensors,
    /// otherwise measurement may return valid-looking but static values.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        // Soft-reset device
        self.reset(delay).await?;

        // Verify chip id
        let chip = self.read_register::<Id>().await.map_err(Error::Bus)?.read_chip();
        if let self::registers::Chip::Invalid(_) = chip {
            return Err(Error::InvalidChip(chip));
        }

        // Read calibration data
        self.calibrate().await.map_err(Error::Bus)?;
        Ok(())
    }

    /// Reads the calibration registers from the device to
    /// compensate measurements. It is required to call this once
    /// before taking any measurements. Calling [`Self::init`] will
    /// automatically do this.
    pub async fn calibrate(&mut self) -> Result<(), I::Error> {
        let params1 = self.read_register::<TrimmingParameters1>().await?;
        let params2 = self.read_register::<TrimmingParameters2>().await?;
        self.calibration_data = Some(CalibrationData::new(params1, params2));

        Ok(())
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 2ms, which is automatically awaited before allowing further communication.
    ///
    /// This will try resetting up to 5 times in case of an error.
    pub async fn reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            match self.try_reset(delay).await {
                Ok(()) => return Ok(()),
                Err(Error::<I::Error>::NvmCopyInProgress) => continue,
                Err(e) => return Err(e),
            }
        }

        Err(Error::<I::Error>::NvmCopyInProgress)
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 2ms, which is automatically awaited before allowing further communication.
    ///
    /// This will check the status register for success, returning an error otherwise.
    pub async fn try_reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        self.write_register(self::registers::Reset::default())
            .await
            .map_err(Error::Bus)?;
        delay.delay_ms(2).await;

        if self
            .read_register::<self::registers::Status>()
            .await
            .map_err(Error::Bus)?
            .read_update()
        {
            return Err(Error::NvmCopyInProgress);
        }
        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> BME280Common<I, true> {
    /// Configures common sensor settings. Sensor must be in sleep mode for this to work.
    /// Check sensor mode beforehand and call [`Self::reset`] if necessary. To configure
    /// advanced settings, please directly update the respective registers.
    pub async fn configure(&mut self, config: Configuration) -> Result<(), Error<I::Error>> {
        self.write_register(ControlHumidity::default().with_oversampling(config.humidity_oversampling))
            .await
            .map_err(Error::Bus)?;

        // This must happen after ControlHumidity, otherwise the former will not have any effect
        self.write_register(
            ControlMeasurement::default()
                .with_temperature_oversampling(config.temperature_oversampling)
                .with_pressure_oversampling(config.pressure_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        let mut reg_config = self.read_register::<Config>().await.map_err(Error::Bus)?;
        reg_config.write_filter(config.iir_filter);
        self.write_register(reg_config).await.map_err(Error::Bus)?;

        Ok(())
    }

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    ///
    /// All measurements will only be included if they were enabled beforehand by calling
    /// [`Self::calibrate`]. Pressure and humitidy measurements specifically require temperature
    /// measurements to be enabled.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        self.write_register(ControlMeasurement::default().with_sensor_mode(SensorMode::Forced))
            .await
            .map_err(Error::Bus)?;

        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_h = self.read_register::<ControlHumidity>().await.map_err(Error::Bus)?;
        let o_h = reg_ctrl_h.read_oversampling();

        let reg_ctrl_m = self.read_register::<ControlMeasurement>().await.map_err(Error::Bus)?;
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

        delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self
            .read_register::<BurstMeasurementsPTH>()
            .await
            .map_err(Error::Bus)?
            .read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(Error::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = (o_p != Oversampling::Disabled)
            .then(|| cal.compensate_pressure(raw_data.pressure.pressure, t_fine))
            .flatten();
        let humidity =
            (o_h != Oversampling::Disabled).then(|| cal.compensate_humidity(raw_data.humidity.humidity, t_fine));

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
        })
    }
}
