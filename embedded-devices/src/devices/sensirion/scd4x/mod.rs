//! # SCD4x (SCD40/SCD41/SCD43)
//!
//! The SCD4x is Sensirion's second generation series of optical CO2 sensors. The sensor series
//! builds on the photoacoustic NDIR sensing principle and Sensirion's patented PASens® and
//! CMOSens® technology to offer accuracy at an attractive price and small form factor. SMD
//! assembly allows cost- and space-effective integration of the sensor combined with maximal
//! freedom of design. On-chip signal compensation is realized with the built-in SHT4x humidity and
//! temperature sensor.
//!
//! Product Variants
//! - **SCD40:** Base accuracy, specified range 400 – 2,000 ppm
//! - **SCD41:** Improved accuracy, specified range 400 – 5,000 ppm, single shot operation feature
//! - **SCD43:** High accuracy, specified range 400 – 5,000 ppm, comprehensive building standard compatibility, single shot operation feature
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::scd4x::Error<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd4x::{SCD4xSync, address::Address};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::si::ratio::{part_per_million, percent};
//!
//! // Create and initialize the device
//! let mut scd4x = SCD4xSync::new_i2c(delay, i2c, Address::Default);
//! scd4x.init().unwrap();
//!
//! // Measure
//! let measurement = scd4x.measure()?;
//! let co2 = measurement.co2_concentration.get::<part_per_million>();
//! let temp = measurement.temperature.get::<degree_celsius>();
//! let humidity = measurement.relative_humidity.get::<percent>();
//! println!("Current measurement: {:?}ppm CO₂, {:?}°C, {:?}%RH", co2, temp, humidity);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), embedded_devices::devices::sensirion::scd4x::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::sensirion::scd4x::{SCD4xAsync, address::Address};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::si::ratio::{part_per_million, percent};
//!
//! // Create and initialize the device
//! let mut scd4x = SCD4xAsync::new_i2c(delay, i2c, Address::Default);
//! scd4x.init().await.unwrap();
//!
//! // Measure
//! let measurement = scd4x.measure().await?;
//! let co2 = measurement.co2_concentration.get::<part_per_million>();
//! let temp = measurement.temperature.get::<degree_celsius>();
//! let humidity = measurement.relative_humidity.get::<percent>();
//! println!("Current measurement: {:?}ppm CO₂, {:?}°C, {:?}%RH", co2, temp, humidity);
//! # Ok(())
//! # }
//! # }
//! ```

// TODO: forced field calibration.
// This is currently not supported as it consists of a write followed by a read 400 ms later
// The codec does not allow this as is.
use embedded_devices_derive::{device, device_impl, sensor};
use embedded_registers::i2c::codecs::crc8_codec::CrcError;
use embedded_registers::RegisterError;
use registers::{
    AmbientPressure, GetDataReadyStatus, GetSensorVariant, GetSerialNumber, MeasureSingleShot, PowerDown,
    ReadMeasurement, Reinit, SetSensorAltitude, StartLowPowerPeriodicMeasurement, StartPeriodicMeasurement,
    StopPeriodicMeasurement, WakeUp, DATA_READY_MASK,
};
use uom::si::thermodynamic_temperature::degree_celsius;
use uom::si::{
    f64::{Length, Pressure, Ratio, ThermodynamicTemperature},
    ratio::{part_per_million, percent},
};

pub mod address;
pub mod registers;

/// The different states the sensor can be in
#[derive(Debug, defmt::Format, Default, PartialEq, Clone)]
pub enum SensorState {
    #[default]
    Idle,
    PeriodicMeasurement,
    LowPowerPeriodicMeasurement,
    Sleep,
}

/// The sensor variant we are dealing with
#[derive(Default, PartialEq)]
pub enum SensorVariant {
    #[default]
    Unknown,
    SCD40,
    SCD41,
    SCD43,
}

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum Error<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Codec error
    #[error("crc codec error")]
    Crc(CrcError),
    /// Invalid chip id was encountered in `init`
    #[error("invalid sensor variant 0x{0:#04x}")]
    InvalidSensorVariant(u16),
    /// Trying to run a command which this sensor does not support
    /// e.g. measure_single_shot on SCD40
    #[error("sensor variant not supported")]
    SensorVariantNotSupported,
    /// Sensor is in a state that does not allow the command that is being executed or the command
    /// is nonsensical for the current state
    #[error("invalid sensor state {0:?}")]
    InvalidSensorState(SensorState),
    /// Sensor is in a status that does not allow the command you're trying to
    /// run or the command is not sensible for the current state
    #[error("self test failed 0x{0:#02}")]
    SelfTestFailed(u8),
    /// The data is not yet ready
    #[error("data not ready")]
    DataNotReady,
}

impl<BusError> From<RegisterError<CrcError, BusError>> for Error<BusError> {
    fn from(value: RegisterError<CrcError, BusError>) -> Self {
        match value {
            RegisterError::Codec(x) => Self::Crc(x),
            RegisterError::Bus(x) => Self::Bus(x),
        }
    }
}

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Current CO₂ concentration
    #[measurement(Co2Concentration)]
    pub co2_concentration: Ratio,
    /// Current temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
    /// Current relative humidity
    #[measurement(RelativeHumidity)]
    pub relative_humidity: Ratio,
}

/// The SCD4x is Sensirion's second generation series of optical CO2 sensors. Several product variants
/// exist with accuracy ranges from 400 – 2,000 ppm (SCD40) up to 400 – 5,000 ppm (SCD43).
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        I2cDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct SCD4x<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    pub variant: SensorVariant,
    /// The current state this sensor is in. Please use [`Self::change_state`] to enter a different
    /// state, so the sensor state is known at all times.
    pub state: SensorState,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> SCD4x<D, embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: self::address::Address) -> Self {
        Self {
            delay,
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
            variant: SensorVariant::default(),
            state: SensorState::default(),
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> SCD4x<D, I> {
    /// Initializes the sensor by first waking the sensor, then stopping any ongoing measurement,
    /// calling reinit and finally verifing the device id. Calling this function is mandatory to
    /// ensure subsequent function calls know the sensor variant.
    pub async fn init(&mut self) -> Result<(), Error<I::BusError>> {
        let _ = self.write_register(WakeUp::default()).await;
        self.delay.delay_ms(30).await;

        self.write_register(StopPeriodicMeasurement::default()).await?;
        self.delay.delay_ms(600).await;

        self.write_register(Reinit::default()).await?;
        self.delay.delay_ms(30).await;

        let device_id = self.read_register::<GetSensorVariant>().await?.read_variant();
        if (device_id & 0xF000) == self::registers::SENSOR_VARIANT_SCD40 {
            self.variant = SensorVariant::SCD40
        } else if (device_id & 0xF000) == self::registers::SENSOR_VARIANT_SCD41 {
            self.variant = SensorVariant::SCD41
        } else if (device_id & 0xF000) == self::registers::SENSOR_VARIANT_SCD43 {
            self.variant = SensorVariant::SCD43
        };
        if self.variant == SensorVariant::Unknown {
            return Err(Error::InvalidSensorVariant(device_id));
        }

        Ok(())
    }

    /// Sets the current sensor altitude in meters above sea level. This value is used to derive an
    /// expected air pressure value which will be used in signal compensation. Expects a value
    /// between 0 and 3000 meters. This function must be called while the sensor is in idle mode.
    pub async fn set_sensor_altitude(&mut self, height: Length) -> Result<(), Error<I::BusError>> {
        if self.state != SensorState::Idle {
            return Err(Error::InvalidSensorState(self.state.clone()));
        }
        let height = height.get::<uom::si::length::meter>();
        self.write_register(SetSensorAltitude::default().with_altitude(height as u16))
            .await?;
        Ok(())
    }

    /// Sets the ambient pressure which will be used for live signal compensation. A value between
    /// 70,000 to 120,000 Pa is expected. This function must be called while the sensor is in
    /// periodic measurement mode.
    pub async fn set_sensor_ambient_pressure(&mut self, height: Pressure) -> Result<(), Error<I::BusError>> {
        if self.state != SensorState::PeriodicMeasurement || self.state != SensorState::LowPowerPeriodicMeasurement {
            return Err(Error::InvalidSensorState(self.state.clone()));
        }
        let pressure = height.get::<uom::si::pressure::hectopascal>();
        self.write_register(AmbientPressure::default().with_pressure(pressure as u16))
            .await?;
        Ok(())
    }

    /// Waits for the next measurement result and returns it.
    async fn get_measurement(
        &mut self,

        wait_time: u32,
        max_iterations: u32,
    ) -> Result<Measurement, Error<I::BusError>> {
        let mut iteration = 0;
        loop {
            let ready = self.read_register::<GetDataReadyStatus>().await?;
            if (ready.read_status() & DATA_READY_MASK) != 0 {
                let measurement = self.read_register::<ReadMeasurement>().await?;
                let co2_concentration = Ratio::new::<part_per_million>(measurement.read_co2() as f64);
                let temperature = (175 * measurement.read_temperature() as i32) as f64 / ((1 << 16) - 1) as f64;
                let temperature = temperature - 45.0;
                let temperature = ThermodynamicTemperature::new::<degree_celsius>(temperature);
                let relative_humidity = (100 * measurement.read_humidity() as i32) as f64 / ((1 << 16) - 1) as f64;
                let relative_humidity = Ratio::new::<percent>(relative_humidity);
                return Ok(Measurement {
                    co2_concentration,
                    temperature,
                    relative_humidity,
                });
            } else if iteration > max_iterations {
                return Err(Error::DataNotReady);
            } else {
                self.delay.delay_ms(wait_time).await;
                iteration += 1;
            }
        }
    }

    // /// Performs a device self test and checks the result
    // pub async fn perform_self_test(&mut self) -> Result<(), Error<I::BusError>> {
    //     let crc = crc::Crc::<u8>::new(SCD4xCrcCodec::instance());
    //     let header = &PerformSelfTest::ADDRESS.to_be_bytes()[core::mem::size_of_val(&PerformSelfTest::ADDRESS) - 2..];
    //
    //     let mut array = [0u8; 3];
    //
    //     self.interface
    //         .bound_bus
    //         .interface
    //         .write(self.interface.bound_bus.address, header)
    //         .await
    //         ?;
    //     self.delay.delay_ms(10000).await;
    //
    //     self.interface
    //         .bound_bus
    //         .interface
    //         .read(self.interface.bound_bus.address, &mut array)
    //         .await
    //         ?;
    //     let value = &array[0..2];
    //     let crc_val = crc.checksum(value);
    //     let crc_real = array[2];
    //     if crc_real != crc_val {
    //         panic!("crc failed")
    //     }
    //     if array[0] != 0 {
    //         Err(Error::SelfTestFailed(array[0]))
    //     } else {
    //         Ok(())
    //     }
    // }

    /// Switch sensor state. This function will issue the correct command to switch from the
    /// current sensor state to the desired state. The SCD4x series has no readout for the current
    /// internal state, this function relies on knowing the current state. If the current state is
    /// not known, please reset the sensor by issuing wake-up and stop-periodic-measurement
    /// manually.
    pub async fn change_state(&mut self, state: SensorState) -> Result<(), Error<I::BusError>> {
        match (&self.state, &state) {
            (SensorState::Idle, SensorState::PeriodicMeasurement) => {
                self.write_register(StartPeriodicMeasurement::default()).await?;
                self.state = state;
                Ok(())
            }
            (SensorState::Idle, SensorState::LowPowerPeriodicMeasurement) => {
                self.write_register(StartLowPowerPeriodicMeasurement::default()).await?;
                self.state = state;
                Ok(())
            }
            (SensorState::Idle, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    self.write_register(PowerDown::default()).await?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::PeriodicMeasurement, SensorState::Idle) => {
                self.write_register(StopPeriodicMeasurement::default()).await?;
                self.delay.delay_ms(500).await;
                self.state = state;
                Ok(())
            }
            (SensorState::PeriodicMeasurement, SensorState::LowPowerPeriodicMeasurement) => {
                self.write_register(StopPeriodicMeasurement::default()).await?;
                self.delay.delay_ms(500).await;
                self.write_register(StartLowPowerPeriodicMeasurement::default()).await?;
                self.state = state;
                Ok(())
            }
            (SensorState::PeriodicMeasurement, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    self.write_register(StopPeriodicMeasurement::default()).await?;
                    self.delay.delay_ms(500).await;
                    self.write_register(PowerDown::default()).await?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::Idle) => {
                self.write_register(StopPeriodicMeasurement::default()).await?;
                self.delay.delay_ms(500).await;
                self.state = state;
                Ok(())
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::PeriodicMeasurement) => {
                self.write_register(StopPeriodicMeasurement::default()).await?;
                self.delay.delay_ms(500).await;
                self.write_register(StartPeriodicMeasurement::default()).await?;
                self.state = state;
                Ok(())
            }
            (SensorState::LowPowerPeriodicMeasurement, SensorState::Sleep) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    self.write_register(StopPeriodicMeasurement::default()).await?;
                    self.delay.delay_ms(500).await;
                    self.write_register(PowerDown::default()).await?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::Idle) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    // This does not acknowledge
                    let _ = self.write_register(WakeUp::default()).await;
                    self.delay.delay_ms(30).await;
                    self.read_register::<GetSerialNumber>().await?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::PeriodicMeasurement) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    // This does not acknowledge
                    let _ = self.write_register(WakeUp::default()).await;
                    self.delay.delay_ms(30).await;
                    self.write_register(StartPeriodicMeasurement::default()).await?;
                    self.state = state;
                    Ok(())
                }
            }
            (SensorState::Sleep, SensorState::LowPowerPeriodicMeasurement) => {
                if self.variant == SensorVariant::SCD40 {
                    Err(Error::SensorVariantNotSupported)
                } else {
                    // This does not acknowledge
                    let _ = self.write_register(WakeUp::default()).await;
                    self.delay.delay_ms(30).await;
                    self.write_register(StartLowPowerPeriodicMeasurement::default()).await?;
                    self.state = state;
                    Ok(())
                }
            }
            _ => Ok(()),
        }
    }
}

#[sensor(Co2Concentration, Temperature, RelativeHumidity)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        OneshotSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::OneshotSensor for SCD4x<D, I> {
    type Error = Error<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. If the sensor supports single shot measurement, we will
    /// utilize this, otherwise a single shot measurement is emulated by starting and stopping
    /// periodic measurement.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        match self.state {
            SensorState::Idle => match self.variant {
                SensorVariant::SCD41 | SensorVariant::SCD43 => {
                    self.write_register(MeasureSingleShot::default()).await?;
                    self.delay.delay_ms(4000).await;
                    self.get_measurement(1000, 6).await
                }
                _ => {
                    self.change_state(SensorState::PeriodicMeasurement).await?;
                    self.delay.delay_ms(4000).await;
                    let ret = self.get_measurement(1000, 6).await;
                    self.change_state(SensorState::Idle).await?;
                    ret
                }
            },
            SensorState::PeriodicMeasurement => self.get_measurement(1000, (5000 / 1000) * 2).await,
            SensorState::LowPowerPeriodicMeasurement => self.get_measurement(2000, (30000 / 2000) * 2).await,
            SensorState::Sleep => {
                // Sleep is only supported on the SCD41 or SCD43 since both support single shot
                // measurement we start without further guards
                self.change_state(SensorState::Idle).await?;
                self.write_register(MeasureSingleShot::default()).await?;
                self.delay.delay_ms(4000).await;
                let ret = self.get_measurement(1000, 6).await;
                self.change_state(SensorState::Idle).await?;
                self.change_state(SensorState::Sleep).await?;
                ret
            }
        }
    }
}
