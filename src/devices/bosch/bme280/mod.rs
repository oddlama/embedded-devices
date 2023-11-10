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

use embedded_registers::RegisterInterface;

pub mod address;
pub mod registers;

// TODO create a second device for BMP280.
// TODO make proc macro instead of rules! macro, so we can use
//      arbitrary extra <I, ...> generics
// TODO make #simple_device_register able to specify multiple parents
// TODO read_all is missing.
// TODO measure, reset, config, ... functions for normal use
// TODO spi support missing

crate::simple_device::device!(
    BME280,
    r#"
The BME280 is a combined digital humidity, pressure and temperature sensor based on proven
sensing principles. The sensor module is housed in an extremely compact metal-lid LGA package.
a footprint of only 2.5 × 2.5 mm² with a height of 0.93 mm. Its small dimensions and its low power
consumption allow the implementation in battery driven devices such as handsets, GPS modules or
watches.

For a full description and usage examples, refer to the [module documentation](self).
"#
);

crate::simple_device::i2c!(BME280, self::address::Address, init_wanted);
// TODO crate::simple_device::spi!(BME280, self::address::Address, init_wanted);

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid Device Id was encountered
    InvalidDeviceId,
    /// Invalid Manufacturer Id was encountered
    InvalidManufacturerId,
}

#[maybe_async_cfg::maybe(sync(not(feature = "async")), async(feature = "async"), keep_self)]
impl<I> BME280<I>
where
    I: RegisterInterface,
{
    /// Initialize the sensor by verifying its device id and manufacturer id.
    /// Not mandatory, but recommended.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        //let device_id = self.read_device_id_revision().await.map_err(InitError::Bus)?;
        //if device_id.read_device_id() != self::device_id_revision::DEVICE_ID_VALID {
        //    return Err(InitError::InvalidDeviceId);
        //}

        Ok(())
    }

    // TODO reset()
}
