//! # BME280 / BMP280
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
//! In order to tailor data rate, noise, response time and current consumption to the needs of the user, a
//! variety of oversampling modes, filter modes and data rates can be selected.
//!
//! ## Usage (BME280)
//!
//! ```
//! # async fn test<I>(mut i2c: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType
//! # {
//! use embedded_devices::devices::microchip::bme280::{BME280, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bme280 = BME280::new_i2c(i2c, Address::Primary);
//! bme280.init().await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let temp = bme280
//!     .read_ambient_temperature()
//!     .await?
//!     .read_temperature()
//!     .get::<degree_celsius>()
//!     .to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (BMP280)
//!
//! ```
//! # async fn test<I>(mut i2c: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType
//! # {
//! use embedded_devices::devices::microchip::bmp280::{BMP280, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bmp280 = BMP280::new_i2c(i2c, Address::Primary);
//! bmp280.init().await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let temp = bmp280
//!     .read_ambient_temperature()
//!     .await?
//!     .read_temperature()
//!     .get::<degree_celsius>()
//!     .to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{i2c::I2cDevice, RegisterInterface};
use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::ratio::percent;
use uom::si::rational32::{Pressure, Ratio, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::address::Address;

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid ChipId was encountered
    InvalidChipId(registers::ChipId),
}

/// All possible errors that may occur in measurement
#[derive(Debug, defmt::Format)]
pub enum MeasurementError<BusError> {
    /// Bus error
    Bus(BusError),
    /// The calibration data was not yet read from the device. Call `init` or `calibrate` first.
    NotCalibrated,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current pressure or None if the sensor reported and invalid value
    pub pressure: Option<Pressure>,
    /// Current relative humidity
    pub humidity: Ratio,
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure and humidity
#[derive(Debug, Clone, Copy)]
pub struct TFine(i32);

// TODO create a second device for BMP280.
// TODO measure, reset, config, ... functions for normal use
// TODO spi support missing

/// The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
/// sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
/// a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
/// consumption allow the implementation in battery driven devices such as handsets, GPS modules or
/// watches.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct BME280<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
    /// Calibration data
    calibration_data: Option<CalibrationData>,
}

#[derive(Debug)]
struct CalibrationData {
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

    fn compensate_temperature(&self, uncompensated: u32) -> (ThermodynamicTemperature, TFine) {
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

    fn compensate_pressure(&self, uncompensated: u32, t_fine: TFine) -> Option<Pressure> {
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

        let var4: i64 = 1048576i64 - uncompensated as i64;
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
        let var5 = (((uncompensated - (dig_h4 << 20)) - (dig_h5 * var1)) + 16384) >> 15;
        let var2 = (var1 * dig_h6) >> 10;
        let var3 = (var1 * dig_h3) >> 11;
        let var4 = ((var2 * (var3 + 32768)) >> 10) + 2097152;
        let var2 = ((var4 * dig_h2) + 8192) >> 14;
        let var3 = var5 * var2;
        let var4 = ((var3 >> 15) * (var3 >> 15)) >> 7;
        let var5 = var3 - ((var4 * dig_h1) >> 4);
        let var5 = if var5 < 0 {
            0
        } else if var5 > 419430400 {
            419430400
        } else {
            var5
        };

        let humidity = Rational32::new_raw(var5, 1i32 << 22);
        Ratio::new::<percent>(humidity)
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> BME280<I2cDevice<I>>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
            interface: I2cDevice {
                interface,
                address: address.into(),
            },
            calibration_data: None,
        }
    }
}

#[device_impl]
impl<I: RegisterInterface> BME280<I> {
    /// Initialize the sensor by performing a soft-reset, verifying its chip id
    /// and reading calibration data.
    pub async fn init<D: hal::delay::DelayUs>(&mut self, delay: &mut D) -> Result<(), InitError<I::Error>> {
        use self::registers::Id;

        // Soft-reset device
        self.reset(delay).await.map_err(InitError::Bus)?;

        // Verify chip id
        let chip_id = self.read_register::<Id>().await.map_err(InitError::Bus)?.read_chip_id();
        if let self::registers::ChipId::Invalid(_) = chip_id {
            return Err(InitError::InvalidChipId(chip_id));
        }

        // Read calibration data
        self.calibrate().await.map_err(InitError::Bus)?;
        Ok(())
    }

    /// Reads the calibration registers from the device to
    /// compensate measurements. It is required to call this once
    /// before taking any measurements. Calling [`Self::init`] will
    /// automatically do this.
    pub async fn calibrate(&mut self) -> Result<(), I::Error> {
        use self::registers::TrimmingParameters1;
        use self::registers::TrimmingParameters2;

        let params1 = self.read_register::<TrimmingParameters1>().await?;
        let params2 = self.read_register::<TrimmingParameters2>().await?;
        self.calibration_data = Some(CalibrationData::new(params1, params2));

        Ok(())
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 2ms, which is automatically awaited before allowing further communication.
    ///
    /// This will also check the status register for success, returning an error otherwise.
    #[inline]
    pub async fn reset<D: hal::delay::DelayUs>(&mut self, delay: &mut D) -> Result<(), I::Error> {
        self.write_register(&self::registers::Reset::default()).await?;
        delay.delay_ms(2).await;
        // TODO read status register
        Ok(())
    }

    // TODO reset_try_up_to try_reset

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    pub async fn measure<D: hal::delay::DelayUs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurements, MeasurementError<I::Error>> {
        use self::registers::SensorMode;
        use self::registers::{BurstMeasurementsPTH, ControlMeasurement};

        self.write_register(&ControlMeasurement::default().with_sensor_mode(SensorMode::Forced))
            .await
            .map_err(MeasurementError::Bus)?;

        // TODO calc wait time
        delay.delay_ms(40).await;

        let raw_data = self
            .read_register::<BurstMeasurementsPTH>()
            .await
            .map_err(MeasurementError::Bus)?
            .read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(MeasurementError::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = cal.compensate_pressure(raw_data.pressure.pressure, t_fine);
        let humidity = cal.compensate_humidity(raw_data.humidity.humidity, t_fine);

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
        })
    }
}
