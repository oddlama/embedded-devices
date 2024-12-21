//! # TMP102
//!
//! The TMP102 device is a digital temperature sensor designed for NTC/PTC
//! thermistor replacement where high accuracy is required. The device offers an
//! accuracy of ±0.5°C without requiring calibration or external component
//! signal conditioning. Device temperature sensors are highly linear and do not
//! require complex calculations or lookup tables to derive the temperature. The
//! on-chip 12-bit ADC offers resolutions down to 0.0625°C.
//!
//! ## Usage
//!
//! ```no_run
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp102::{TMP102, address::Address, registers::Temperature};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp102 = TMP102::new_i2c(i2c, Address::Gnd);
//! tmp102.init(&mut Delay).await.unwrap();
//!
//! // Read the latest temperature conversion in °C and convert it to a float
//! let temp = tmp102
//!     .read_register::<Temperature>()
//!     .await?
//!     .read_temperature()
//!     .get::<degree_celsius>()
//!     .to_f32();
//! println!("Current temperature: {:?}°C", temp);
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp102.oneshot(&mut Delay).await?.get::<degree_celsius>().to_f32();
//! println!("Oneshot temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::RegisterInterface;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

type TMP102I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// The TMP102 is a general purpose digital temperature sensor. It provides a 13-bit
/// temperature result with a resolution of 0.0625 °C and an accuracy of up to ±3 °C across the
/// temperature range of –40 °C to 125 °C with no calibration.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct TMP102<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

crate::simple_device::i2c!(TMP102, self::address::Address, SevenBitAddress, TMP102I2cCodec);

#[device_impl]
impl<I: RegisterInterface> TMP102<I> {
    /// Performs a one-shot measurement. This will set `shutdown` in [`self::registers::Configuration´].
    /// which will cause the device to perform a single conversion a return to sleep mode afterwards.
    ///
    /// This function will initialize the measurement, wait until the data is acquired and return
    /// the temperature.
    pub async fn oneshot<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<ThermodynamicTemperature, I::Error> {
        use self::registers::{Configuration, Temperature};

        // Read current configuration to determine conversion ratio and delay
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_oneshot(true);

        // Initiate measurement
        self.write_register(&reg_conf).await?;

        // Wait for the duration of the conversion
        let active_conversion_time = reg_conf.read_conversion_cycle_time().conversion_time_ms() + 10;
        delay.delay_ms(active_conversion_time).await;

        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        if reg_conf.read_extended() {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(
                raw_temp, 128,
            )))
        } else {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(
                raw_temp, 256,
            )))
        }
    }

    /// Read the last temperature measured
    pub async fn read_temp(&mut self) -> Result<ThermodynamicTemperature, I::Error> {
        use self::registers::{Configuration, Temperature};

        // Read current configuration to determine conversion ratio
        let reg_conf = self.read_register::<Configuration>().await?;

        // Read and return the temperature
        let raw_temp = self.read_register::<Temperature>().await?.read_raw_temperature() as i32;
        if reg_conf.read_extended() {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(
                raw_temp, 128,
            )))
        } else {
            Ok(ThermodynamicTemperature::new::<degree_celsius>(Rational32::new_raw(
                raw_temp, 256,
            )))
        }
    }

    pub async fn set_continuous(&mut self) -> Result<(), I::Error> {
        use self::registers::Configuration;

        // Read current configuration and update it for continuous mode
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_oneshot(false);
        reg_conf.write_extended(true);
        reg_conf.write_shutdown(false);

        // Initiate measurement
        self.write_register(&reg_conf).await?;

        Ok(())
    }
}
