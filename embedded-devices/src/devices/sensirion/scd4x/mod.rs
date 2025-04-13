//! # SCD4X(SCD40/SCD41/SCD43)
//! The SCD4x is Sensirion’s second generation optical CO2
//! sensor. This sensor builds on the photoacoustic NDIR
//! sensing principle and Sensirion’s patented PASens® and
//! CMOSens® technology to offer high accuracy at an
//! attractive price and small form factor. SMD assembly
//! allows cost- and space-effective integration of the sensor
//! combined with maximal freedom of design. On-chip
//! signal compensation is realized with the built-in SHT4x
//! humidity and temperature sensor.
//!
//! Product Variants
//! ▪ SCD40: Base accuracy, specified measurement
//! range 400 –2’000 ppm
//! ▪ SCD41: High accuracy, specified measurement
//! range 400 – 5’000 ppm, compatible with
//! California Title 241, RESET® 2 and WELL Building
//! Standard™ 3, features single shot operation mode
//! SCD43: High accuracy, specified measurement range
//! 400 – 5’000 ppm, additionally compatible with
//! ASHRAE Standard 62.1 4, features single-shot
//! operation mode
//! ## Usage (sync)
//!
//! ```rust, only_if(sync)
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::sensirion::scd4x::Error<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd4x::{SCD4xSync, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut scd = SCD4xSync::new_i2c(i2c, Address::Default);
//! scd.init(&mut Delay).unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = scd.measure(&mut Delay)?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::sensirion::scd4x::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd4x::{SCD4xAsync, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut scd = SCD4xAsync::new_i2c(i2c, Address::Default);
//! scd.init(&mut Delay).await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = scd.measure(&mut Delay).await?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!

/// TODO: forced field calibration.
use crc::{Algorithm, CRC_8_NRSC_5};
use embedded_devices_derive::{device, device_impl};
use embedded_registers::i2c::codecs::Crc8Algorithm;
use registers::{
    AmbientPressure, GetDataReadyStatus, GetSensorVariant, MeasureSingleShot, PowerDown, ReadMeasurement,
    SetSensorAltitude, StartLowPowerPeriodicMeasurement, StartPeriodicMeasurement, StopPeriodicMeasurement, WakeUp,
    DATA_READY_MASK,
};
use uom::num_rational::Rational32;
use uom::si::thermodynamic_temperature::degree_celsius;
use uom::si::{
    ratio::{part_per_million, percent},
    rational32::{Ratio, ThermodynamicTemperature},
    u16::{Length, Pressure},
};

pub mod address;
pub mod registers;

/// Enum describing the different modes the sensor can be in
#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum SensorState {
    #[default]
    Idle,
    PeriodicMeasurement,
    LowPowerPeriodicMeasurement,
    Sleep,
}

/// Enum desrcribing which sensor variant we are dealing with
#[derive(Default, PartialEq)]
pub enum SensorVariant {
    #[default]
    Unknown,
    SCD40,
    SCD41,
    SCD43,
}

#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct SCD4x<I: embedded_registers::RegisterInterface> {
    interface: I,
    pub variant: SensorVariant,
    /// The current state this sensor is in
    /// You should use the `change_state`
    /// for chaning states to prevent
    /// a mismatch with the actual state
    pub state: SensorState,
}

#[derive(Default)]
pub struct SCD4xCrcCodec {}

impl Crc8Algorithm for SCD4xCrcCodec {
    fn new() -> &'static Algorithm<u8> {
        const CUSTOM_ALG: crc::Algorithm<u8> = CRC_8_NRSC_5;
        &CUSTOM_ALG
    }
}
type SCD4xCodec = embedded_registers::i2c::codecs::Crc8Codec<2, 2, SCD4xCrcCodec>;

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> SCD4x<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, SCD4xCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    #[doc = "Initializes a new device with the given address on the specified bus."]
    #[doc = "This consumes the I2C bus `I`."]
    #[doc = ""]
    #[doc = "Before using this device, you should call the [`Self::init`] method which ensures that the device is working correctly."]
    #[inline]
    pub fn new_i2c(interface: I, address: self::address::Address) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
            variant: SensorVariant::default(),
            state: SensorState::default(),
        }
    }
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    pub co2: Ratio,
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current relative humidity
    pub humidity: Ratio,
}

/// All possible errors that may occur when using this device
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid chip id was encountered in `init`
    InvalidSensorVariant(u16),
    /// Trying to run a command which this sensor does not support
    /// e.g. measure_single_shot on SCD40
    SensorVariantNotSupported(),
    /// Sensor is in a status that does not allow the command you're trying to
    /// run or the command is not sensible for the current state
    InvalidSensorState(SensorState),
    /// The data is not ready yet.
    DataNotReady,
    /// NVM data copy is still in progress.
    NvmCopyInProgress,
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> SCD4x<I> {
    /// Initialize the sensor by waiting for the boot-up period and verifying its device id.
    /// The datasheet specifies a power-on-reset time of 30ms.
    /// Calling this function is not mandatory, but recommended to ensure proper operation.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        delay.delay_ms(30).await;
        let device_id = self.read_register::<GetSensorVariant>().await.map_err(Error::Bus)?;
        if (device_id.read_variant() & 0xF000) != self::registers::SCD40_ID {
            self.variant = SensorVariant::SCD40
        } else if (device_id.read_variant() & 0xF000) != self::registers::SCD41_ID {
            self.variant = SensorVariant::SCD41
        } else if (device_id.read_variant() & 0xF000) != self::registers::SCD43_ID {
            self.variant = SensorVariant::SCD43
        };
        if self.variant == SensorVariant::Unknown {
            return Err(Error::InvalidSensorVariant(device_id.read_variant()));
        }

        Ok(())
    }

    /// Set the current sensor altitude is meters above the sea level
    /// should be between 0 and 3000
    /// Can only be set while in idle mode
    pub async fn set_sensor_altitude(&mut self, height: &mut Length) -> Result<(), Error<I::Error>> {
        if self.state != SensorState::Idle {
            return Err(Error::InvalidSensorState(self.state.clone()));
        }
        let height = height.get::<uom::si::length::meter>();
        self.write_register(SetSensorAltitude::default().with_altitude(height))
            .await
            .map_err(Error::Bus)?;
        Ok(())
    }

    /// Set the sensor ambient pressure
    /// should be between 70000 to 120000 Pa
    /// Can only be set while in periodic measurement mode
    pub async fn set_sensor_ambient_pressure(&mut self, height: &mut Pressure) -> Result<(), Error<I::Error>> {
        if self.state != SensorState::PeriodicMeasurement || self.state != SensorState::LowPowerPeriodicMeasurement {
            return Err(Error::InvalidSensorState(self.state.clone()));
        }
        let pressure = height.get::<uom::si::pressure::hectopascal>();
        self.write_register(AmbientPressure::default().with_pressure(pressure))
            .await
            .map_err(Error::Bus)?;
        Ok(())
    }

    /// Unconditionally try and get a measurement, returning an error if no data is
    /// ready
    async fn get_measurement(&mut self) -> Result<Measurements, Error<I::Error>> {
        let measurements = self.read_register::<ReadMeasurement>().await.map_err(Error::Bus)?;
        let co2 = measurements.read_co2();
        let co2 = Rational32::new_raw(co2 as i32, 1);
        let co2 = Ratio::new::<part_per_million>(co2);
        let temperature = Rational32::new_raw(175 * measurements.read_temperature() as i32, (1 << 16) - 1);
        let temperature = temperature - 45;
        let temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature);
        let humidity = Rational32::new_raw(100 * measurements.read_humidity() as i32, (1 << 16) - 1);
        let humidity = Ratio::new::<percent>(humidity);
        Ok(Measurements {
            co2,
            temperature,
            humidity,
        })
    }

    pub async fn change_state<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
        state: SensorState,
    ) -> Result<(), Error<I::Error>> {
        match (&self.state, &state) {
            (SensorState::Idle, SensorState::PeriodicMeasurement) => {
                self.write_register(StartPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                self.state = state;
                Ok(())
            }
            (SensorState::Idle, SensorState::LowPowerPeriodicMeasurement) => {
                self.write_register(StartLowPowerPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                self.state = state;
                Ok(())
            }
            (SensorState::Idle, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(PowerDown::default()).await.map_err(Error::Bus)?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::PeriodicMeasurement, SensorState::Idle) => {
                self.write_register(StopPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                delay.delay_ms(500).await;
                self.state = state;
                Ok(())
            }
            (SensorState::PeriodicMeasurement, SensorState::LowPowerPeriodicMeasurement) => {
                self.write_register(StopPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                delay.delay_ms(500).await;
                self.write_register(StartLowPowerPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                self.state = state;
                Ok(())
            }
            (SensorState::PeriodicMeasurement, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(StopPeriodicMeasurement::default())
                        .await
                        .map_err(Error::Bus)?;
                    delay.delay_ms(500).await;
                    self.write_register(PowerDown::default()).await.map_err(Error::Bus)?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::Idle) => {
                self.write_register(StopPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                delay.delay_ms(500).await;
                self.state = state;
                Ok(())
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::PeriodicMeasurement) => {
                self.write_register(StopPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                delay.delay_ms(500).await;
                self.write_register(StartPeriodicMeasurement::default())
                    .await
                    .map_err(Error::Bus)?;
                self.state = state;
                Ok(())
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(StopPeriodicMeasurement::default())
                        .await
                        .map_err(Error::Bus)?;
                    delay.delay_ms(500).await;
                    self.write_register(PowerDown::default()).await.map_err(Error::Bus)?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::Idle) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(WakeUp::default()).await.map_err(Error::Bus)?;
                    delay.delay_ms(30).await;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::PeriodicMeasurement) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(WakeUp::default()).await.map_err(Error::Bus)?;
                    delay.delay_ms(30).await;
                    self.write_register(StartPeriodicMeasurement::default())
                        .await
                        .map_err(Error::Bus)?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::LowPowerPeriodicMeasurement) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported())
                } else {
                    self.write_register(WakeUp::default()).await.map_err(Error::Bus)?;
                    delay.delay_ms(30).await;
                    self.write_register(StartLowPowerPeriodicMeasurement::default())
                        .await
                        .map_err(Error::Bus)?;
                    self.state = state;
                    Ok(())
                }
            }
            _ => Ok(()),
        }
    }

    // Performs a one-shot measurement. This will set the conversion mode to [`self::registers::ConversionMode::Oneshot´].
    // which will cause the device to perform a single conversion a return to sleep mode afterwards.
    //
    // This function will initialize the measurement, wait until the data is acquired and return
    // the temperature.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        match self.state {
            SensorState::Idle => match self.variant {
                SensorVariant::SCD41 | SensorVariant::SCD43 => {
                    self.write_register(MeasureSingleShot::default())
                        .await
                        .map_err(Error::Bus)?;
                    delay.delay_ms(5000).await;
                    self.get_measurement().await
                }
                _ => {
                    self.change_state(delay, SensorState::PeriodicMeasurement).await?;
                    delay.delay_ms(5000).await;
                    let erg = self.get_measurement().await;
                    self.change_state(delay, SensorState::Idle).await?;
                    erg
                }
            },
            SensorState::PeriodicMeasurement => {
                let mut iteration = 0;
                loop {
                    let ready = self.read_register::<GetDataReadyStatus>().await.map_err(Error::Bus)?;
                    if (ready.read_status() & DATA_READY_MASK) != 0 {
                        return self.get_measurement().await;
                    } else if iteration > (5000 / 1000) * 2 {
                        return Err(Error::DataNotReady);
                    }
                    delay.delay_ms(1000).await;
                    iteration += 1;
                }
            }
            SensorState::LowPowerPeriodicMeasurement => {
                let mut iteration = 0;
                loop {
                    let ready = self.read_register::<GetDataReadyStatus>().await.map_err(Error::Bus)?;
                    if (ready.read_status() & DATA_READY_MASK) != 0 {
                        return self.get_measurement().await;
                    } else if iteration > (30000 / 2000) * 2 {
                        return Err(Error::DataNotReady);
                    }
                    delay.delay_ms(2000).await;
                    iteration += 1;
                }
            }
            SensorState::Sleep => {
                // Sleep is only supported on the SCD41 or SCD43
                // since both support single shot measurements we
                // start one without further guards
                self.change_state(delay, SensorState::Idle).await?;
                self.write_register(MeasureSingleShot::default())
                    .await
                    .map_err(Error::Bus)?;
                delay.delay_ms(5000).await;
                let erg = self.get_measurement().await;
                self.change_state(delay, SensorState::Idle).await?;
                self.change_state(delay, SensorState::Sleep).await?;
                erg
            }
        }
    }
}
