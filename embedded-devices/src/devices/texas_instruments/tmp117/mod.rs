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
//! ## Usage
//!
//! ```
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::tmp117::{TMP117, address::Address, registers::Temperature};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device. Default conversion mode is continuous.
//! let mut tmp117 = TMP117::new_i2c(i2c, Address::Gnd);
//! tmp117.init(&mut Delay).await.unwrap();
//!
//! // Read the latest temperature conversion in °C and convert it to a float
//! let temp = tmp117
//!     .read_register::<Temperature>()
//!     .await?
//!     .read_temperature()
//!     .get::<degree_celsius>()
//!     .to_f32();
//! println!("Current temperature: {:?}°C", temp);
//!
//! // Perform a one-shot measurement now and return to sleep afterwards.
//! let temp = tmp117.oneshot(&mut Delay).await?.get::<degree_celsius>().to_f32();
//! println!("Oneshot temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{RegisterInterface, WritableRegister};
use uom::si::rational32::ThermodynamicTemperature;

pub mod address;
pub mod registers;

type TMP117I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid Device Id was encountered
    InvalidDeviceId,
}

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum EepromError<BusError> {
    /// Bus error
    Bus(BusError),
    /// EEPROM is still busy after 13ms
    EepromStillBusy,
}

/// The TMP117 is a high-precision digital temperature sensor. It provides a 16-bit
/// temperature result with a resolution of 0.0078 °C and an accuracy of up to ±0.1 °C across the
/// temperature range of –20 °C to 50 °C with no calibration.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct TMP117<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

crate::simple_device::i2c!(
    TMP117,
    self::address::Address,
    SevenBitAddress,
    TMP117I2cCodec,
    "init=wanted"
);

#[device_impl]
impl<I: RegisterInterface> TMP117<I> {
    /// Initialize the sensor by waiting for the boot-up period and verifying its device id.
    /// The datasheet specifies a power-on-reset time of 1.5ms.
    /// Calling this function is not mandatory, but recommended to ensure proper operation.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), InitError<I::Error>> {
        use self::registers::DeviceIdRevision;

        delay.delay_us(1500).await;

        let device_id = self.read_register::<DeviceIdRevision>().await.map_err(InitError::Bus)?;
        if device_id.read_device_id() != self::registers::DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId);
        }

        Ok(())
    }

    /// Performs a soft-reset of the device.
    /// The datasheet specifies a time to reset of 2ms which is
    /// automatically awaited before allowing further communication.
    pub async fn reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), I::Error> {
        self.write_register(&self::registers::Configuration::default().with_soft_reset(true))
            .await?;
        delay.delay_ms(2).await;
        Ok(())
    }

    /// Write a register value into EEPROM. Usually this will persist the register
    /// value as the new power-on default. Not all registers / register-bits support this.
    pub async fn write_eeprom<R, D>(&mut self, delay: &mut D) -> Result<(), EepromError<I::Error>>
    where
        R: WritableRegister,
        D: hal::delay::DelayNs,
    {
        use self::registers::{EepromLockMode, EepromUnlock};

        // Unlock EEPROM
        self.write_register(&EepromUnlock::default().with_lock_mode(EepromLockMode::Unlocked))
            .await
            .map_err(EepromError::Bus)?;

        // Wait 7ms for EEPROM write to complete
        delay.delay_ms(7).await;

        // Wait up to 5ms for eeprom busy flag to be reset
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            if !self
                .read_register::<EepromUnlock>()
                .await
                .map_err(EepromError::Bus)?
                .read_busy()
            {
                // EEPROM write complete, lock eeprom again
                self.write_register(&EepromUnlock::default().with_lock_mode(EepromLockMode::Locked))
                    .await
                    .map_err(EepromError::Bus)?;

                return Ok(());
            }

            // Wait another 1ms for EEPROM write to complete
            delay.delay_ms(1).await;
        }

        Err(EepromError::EepromStillBusy)
    }

    /// Performs a one-shot measurement. This will set the conversion mode to [`self::registers::ConversionMode::Oneshot´].
    /// which will cause the device to perform a single conversion a return to sleep mode afterwards.
    ///
    /// This function will initialize the measurement, wait until the data is acquired and return
    /// the temperature.
    pub async fn oneshot<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<ThermodynamicTemperature, I::Error> {
        use self::registers::{Configuration, ConversionMode, Temperature};

        // Read current averaging mode to determine required measurement delay
        let mut reg_conf = self.read_register::<Configuration>().await?;
        reg_conf.write_conversion_mode(ConversionMode::Oneshot);

        // Initiate measurement
        self.write_register(&reg_conf).await?;

        // Active conversion time is only linearly influenced by the averaging factor.
        // A single-conversion takes 15.5ms.
        let active_conversion_time = reg_conf.read_averaging_mode().factor() as u32 * 15500;
        delay.delay_us(active_conversion_time).await;

        // Read and return the temperature
        Ok(self.read_register::<Temperature>().await?.read_temperature())
    }
}
