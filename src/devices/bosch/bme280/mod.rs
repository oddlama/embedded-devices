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
use embedded_registers::RegisterInterface;

pub mod address;
pub mod registers;

// TODO create a second device for BMP280.
// TODO make proc macro instead of rules! macro, so we can use
//      arbitrary extra <I, ...> generics
// TODO make #simple_device_register able to specify multiple parents
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
}

crate::simple_device::i2c!(BME280, self::address::Address, init_required);
// TODO crate::simple_device::spi!(BME280, self::address::Address, init_required);

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid ChipId was encountered
    InvalidChipId(registers::ChipId),
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
    t_fine: i32,
}

impl CalibrationData {
    pub fn new(params1: registers::TrimmingParameters1, params2: registers::TrimmingParameters2) -> Self {
        let params1 = params1.read_all();
        let params2 = params2.read_all();
        let dig_h4 = (params2.dig_h4_msb as i16 * 16) | ((params2.dig_h5_lsn_h4_lsn as i16) & 0x0F);
        let dig_h5 = (params2.dig_h5_msb as i16 * 16) | (((params2.dig_h5_lsn_h4_lsn as i16) & 0xF0) >> 4);
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
            dig_h4,
            dig_h5,
            dig_h6: params2.dig_h6,
            t_fine: 0,
        }
    }
}

#[device_impl]
impl<I> BME280<I>
where
    I: RegisterInterface,
{
    /// Initialize the sensor by performing a soft-reset, verifying its chip id
    /// and reading calibration data.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        self.reset().await?;

        let chip_id = self.read_id().await.map_err(InitError::Bus)?.read_chip_id();
        if let self::registers::ChipId::Invalid(_) = chip_id {
            return Err(InitError::InvalidChipId(chip_id));
        }

        self.calibrate().await?;

        Ok(())
    }

    ///
    pub async fn calibrate(&mut self) -> Result<(), InitError<I::Error>> {
        let params1 = self.read_trimming_parameters_1().await.map_err(InitError::Bus)?;
        let params2 = self.read_trimming_parameters_2().await.map_err(InitError::Bus)?;
        self.calibration_data = CalibrationData::new(params1, params2);

        Ok(())
    }

    ///
    pub async fn reset(&mut self) -> Result<(), InitError<I::Error>> {
        Ok(())
    }
}
