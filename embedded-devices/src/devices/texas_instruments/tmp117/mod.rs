//! # TMP117
//!
//! The TMP117 is a high-precision digital temperature sensor. It is designed to meet ASTM E1112
//! and ISO 80601 requirements for electronic patient thermometers. The TMP117 provides a 16-bit
//! temperature result with a resolution of 0.0078 °C and an accuracy of up to ±0.1 °C across the
//! temperature range of –20 °C to 50 °C with no calibration. The TMP117 has in interface that is
//! I2C- and SMBus™-compatible, programmable alert functionality, and the device can support up to four
//! devices on a single bus. Integrated EEPROM is included for device programming with an additional
//! 48-bits memory available for general use.
//!
//! The low power consumption of the TMP117 minimizes the impact of self-heating on measurement accuracy.
//! The TMP117 operates from 1.7 V to 5.5 V and typically consumes 3.5 μA.
//!
//! For non-medical applications, the TMP117 can serve as a single chip digital alternative to a Platinum RTD.
//! The TMP117 has an accuracy comparable to a Class AA RTD, while only using a fraction of the power of the
//! power typically needed for a PT100 RTD. The TMP117 simplifies the design effort by removing many of the
//! complexities of RTDs such as precision references, matched traces, complicated algorithms, and calibration.
//!
//! The TMP117 units are 100% tested on a production setup that is NIST traceable and verified with
//! equipment that is calibrated to ISO/IEC 17025 accredited standards.
//!
//! ## Usage (sync)
//!
//! ```
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_registers::RegisterError<(), I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp117::{TMP117Sync, address::Address, registers::Temperature};
//! use embedded_devices::sensors::SensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp117 = TMP117Sync::new_i2c(i2c, Address::Gnd);
//! tmp117.init(&mut Delay).unwrap();
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp117.measure(&mut Delay)?
//!     .temperature.get::<degree_celsius>();
//! println!("Oneshot temperature: {}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_registers::RegisterError<(), I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp117::{TMP117Async, address::Address, registers::Temperature};
//! use embedded_devices::sensors::SensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp117 = TMP117Async::new_i2c(i2c, Address::Gnd);
//! tmp117.init(&mut Delay).await.unwrap();
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp117.measure(&mut Delay).await?
//!     .temperature.get::<degree_celsius>();
//! println!("Oneshot temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::{device, device_impl, sensor};
use embedded_registers::{RegisterError, WritableRegister};
use uom::si::f64::ThermodynamicTemperature;

use crate::utils::from_bus_error;

pub mod address;
pub mod registers;

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum InitError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(BusError),
    /// Invalid Device Id was encountered
    #[error("invalid device id {0:#04x}")]
    InvalidDeviceId(u16),
}

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum EepromError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(BusError),
    /// EEPROM is still busy after 13ms
    #[error("eeprom still busy")]
    EepromStillBusy,
}

from_bus_error!(InitError);
from_bus_error!(EepromError);

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Measured temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
}

/// The TMP117 is a high-precision digital temperature sensor. It provides a 16-bit
/// temperature result with a resolution of 0.0078 °C and an accuracy of up to ±0.1 °C across the
/// temperature range of –20 °C to 50 °C with no calibration.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct TMP117<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> TMP117<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(interface: I, address: self::address::Address) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> TMP117<I> {
    /// Initialize the sensor by waiting for the boot-up period and verifying its device id.
    /// Calling this function is not mandatory, but recommended to ensure proper operation.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), InitError<I::BusError>> {
        use self::registers::DeviceIdRevision;

        // Soft-reset device
        self.reset(delay).await?;

        // Verify device id
        let device_id = self.read_register::<DeviceIdRevision>().await?.read_device_id();
        if device_id != self::registers::DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId(device_id));
        }

        Ok(())
    }

    /// Performs a soft-reset of the device.
    /// The datasheet specifies a time to reset of 2ms which is
    /// automatically awaited before allowing further communication.
    pub async fn reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), RegisterError<(), I::BusError>> {
        self.write_register(self::registers::Configuration::default().with_soft_reset(true))
            .await?;
        delay.delay_ms(2).await;
        Ok(())
    }

    /// Write a register value into EEPROM. Usually this will persist the register
    /// value as the new power-on default. Not all registers / register-bits support this.
    pub async fn write_eeprom<R, D>(&mut self, delay: &mut D) -> Result<(), EepromError<I::BusError>>
    where
        R: WritableRegister,
        D: hal::delay::DelayNs,
    {
        use self::registers::{EepromLockMode, EepromUnlock};

        // Unlock EEPROM
        self.write_register(EepromUnlock::default().with_lock_mode(EepromLockMode::Unlocked))
            .await?;

        // Wait 7ms for EEPROM write to complete
        delay.delay_ms(7).await;

        // Wait up to 5ms for eeprom busy flag to be reset
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            if !self.read_register::<EepromUnlock>().await?.read_busy() {
                // EEPROM write complete, lock eeprom again
                self.write_register(EepromUnlock::default().with_lock_mode(EepromLockMode::Locked))
                    .await?;

                return Ok(());
            }

            // Wait another 1ms for EEPROM write to complete
            delay.delay_ms(1).await;
        }

        Err(EepromError::EepromStillBusy)
    }
}

#[sensor(Temperature)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface, Sensor),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> crate::sensors::Sensor for TMP117<I> {
    type Error = RegisterError<(), I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will set the conversion mode to
    /// [`self::registers::ConversionMode::Oneshot´] causing the device to perform a
    /// single conversion a return to sleep afterwards.
    async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Self::Measurement, Self::Error> {
        use self::registers::{Configuration, ConversionMode, Temperature};

        // Read current averaging mode to determine required measurement delay
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_conversion_mode(ConversionMode::Oneshot);

        // Initiate measurement
        self.write_register(reg_conf).await?;

        // Active conversion time is only linearly influenced by the averaging factor.
        // A single-conversion takes 15.5ms.
        let active_conversion_time = reg_conf.read_averaging_mode().factor() as u32 * 15500;
        delay.delay_us(active_conversion_time).await;

        // Read and return the temperature
        let temperature = self.read_register::<Temperature>().await?.read_temperature();
        Ok(Measurement { temperature })
    }
}
