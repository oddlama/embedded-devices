//! # BMP390
//!
//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
//! module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
//! mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
//! driven devices such as mobile phones, GPS modules or watches.
//!
//! ## Usage (sync)
//!
//! ```rust, only_if(sync)
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::bosch::bmp390::Error<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp390::{BMP390Sync, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bmp390 = BMP390Sync::new_i2c(i2c, Address::Primary);
//! bmp390.init(&mut Delay).unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = bmp390.measure(&mut Delay)?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), embedded_devices::devices::bosch::bmp390::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp390::{BMP390Async, address::Address};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! // Create and initialize the device
//! let mut bmp390 = BMP390Async::new_i2c(i2c, Address::Primary);
//! bmp390.init(&mut Delay).await.unwrap();
//!
//! // Read the current temperature in °C and convert it to a float
//! let measurements = bmp390.measure(&mut Delay).await?;
//! let temp = measurements.temperature.get::<degree_celsius>().to_f32();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::rational32::{Pressure, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::address::Address;
use self::registers::{
    BurstMeasurements, Chip, ChipId, Cmd, Config, IIRFilter, Oversampling, OversamplingControl, PowerControl,
    SensorMode, TrimmingCoefficients, TrimmingCoefficientsBitfield,
};

type BME390SpiCodec = embedded_registers::spi::codecs::SimpleCodec<1, 6, 0, 7, true, 1>;
type BME390I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur when using this device
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid ChipId was encountered in `init`
    InvalidChip(Chip),
    /// The calibration data was not yet read from the device, but a measurement was requested. Call `init` or `calibrate` first.
    NotCalibrated,
    /// NVM data copy is still in progress.
    ResetFailed,
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure
#[derive(Debug, Clone, Copy)]
pub(super) struct TFine(i32);

#[derive(Debug)]
pub(super) struct CalibrationData(TrimmingCoefficientsBitfield);

impl CalibrationData {
    pub(super) fn compensate_temperature(&self, uncompensated: u32) -> (ThermodynamicTemperature, TFine) {
        let v1: u64 = (uncompensated as u64) - ((self.0.par_t1 as u64) << 8);
        let v2: u64 = self.0.par_t2 as u64 * v1;
        let v3: u64 = v1 * v1;
        let v4: i64 = (v3 as i64) * (self.0.par_t3 as i64);
        let v5: i64 = ((v2 as i64) << 18) + v4;
        let t_fine: i32 = (v5 >> 32) as i32;
        let temperature = Rational32::new_raw(t_fine * 25, 100 << 14);

        (
            ThermodynamicTemperature::new::<degree_celsius>(temperature),
            TFine(t_fine),
        )
    }

    pub(super) fn compensate_pressure(&self, uncompensated: u32, t_fine: TFine) -> Pressure {
        let t_fine = t_fine.0;

        let v1 = t_fine as i64 * t_fine as i64;
        let v3 = ((v1 >> 6) * t_fine as i64) >> 8;
        let v4 = (self.0.par_p8 as i64 * v3) >> 5;
        let v5 = (self.0.par_p7 as i64 * v1) << 4;
        let v6 = (self.0.par_p6 as i64 * t_fine as i64) << 22;
        let offset = ((self.0.par_p5 as i64) << 47) + v4 + v5 + v6;
        let v2 = ((self.0.par_p4 as i64) * v3) >> 5;
        let v4 = (self.0.par_p3 as i64 * v1) << 2;
        let v5 = ((self.0.par_p2 as i64 - 16384) * t_fine as i64) << 21;
        let sensitivity = (((self.0.par_p1 as i64 - 16384) << 46) + v2 + v4 + v5) >> 24;
        let v1 = (sensitivity * uncompensated as i64) >> 13;
        let v2 = (self.0.par_p10 as i64) * (t_fine as i64);
        let v3 = v2 + ((self.0.par_p9 as i64) << 16);
        let v4 = (v3 * uncompensated as i64) >> 13;
        let v5 = (v4 * uncompensated as i64) >> 9;
        let v6 = uncompensated as i64 * uncompensated as i64;
        let v2 = ((self.0.par_p11 as i64) * v6) >> 16;
        let v3 = (v2 * uncompensated as i64) >> 7;
        let v4 = (offset / 4) + v1 + v5 + v3;
        let pressure = ((v4 >> 32) * 25) as i32;
        // TODO this is wrong ^------------
        // actually pressure in 0.01° = (v4 * 25) >> 40, but we save some extra precision
        // by only shifting by 32 and doing / 256 in the denominator
        let pressure = Rational32::new_raw(pressure, 100 << 8);
        Pressure::new::<pascal>(pressure)
    }
}

/// The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
/// module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
/// mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
/// driven devices such as mobile phones, GPS modules or watches.
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BMP390<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
    /// Calibration data
    pub(super) calibration_data: Option<CalibrationData>,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current pressure
    pub pressure: Pressure,
}

/// Common configuration values for the BME280 sensor.
/// The power-on-reset default is to set all oversampling settings to 1X
/// and use no IIR filter.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// The oversampling rate for temperature mesurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pub pressure_oversampling: Oversampling,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            temperature_oversampling: Oversampling::X_1,
            pressure_oversampling: Oversampling::X_1,
            iir_filter: IIRFilter::Disabled,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> BMP390<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, BME390I2cCodec>>
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
impl<I> BMP390<embedded_registers::spi::SpiDevice<I, BME390SpiCodec>>
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
impl<I: embedded_registers::RegisterInterface> BMP390<I> {
    /// Initialize the sensor by performing a soft-reset, verifying its chip id
    /// and reading calibration data.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        // Soft-reset device
        self.reset(delay).await?;

        // Verify chip id
        let chip = self.read_register::<ChipId>().await.map_err(Error::Bus)?.read_chip();
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
        let coefficients = self.read_register::<TrimmingCoefficients>().await?.read_all();
        self.calibration_data = Some(CalibrationData(coefficients));

        Ok(())
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 10ms, which is automatically awaited before allowing further communication.
    ///
    /// This will try resetting up to 5 times in case of an error.
    pub async fn reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            match self.try_reset(delay).await {
                Ok(()) => return Ok(()),
                Err(Error::<I::Error>::ResetFailed) => continue,
                Err(e) => return Err(e),
            }
        }

        Err(Error::<I::Error>::ResetFailed)
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 10ms, which is automatically awaited before allowing further communication.
    ///
    /// This will check the status register for success, returning an error otherwise.
    pub async fn try_reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        self.write_register(self::registers::Command::default().with_command(Cmd::Reset))
            .await
            .map_err(Error::Bus)?;
        delay.delay_ms(10).await;

        if self
            .read_register::<self::registers::Error>()
            .await
            .map_err(Error::Bus)?
            .read_cmd_err()
        {
            return Err(Error::ResetFailed);
        }
        Ok(())
    }

    /// Configures common sensor settings. Sensor must be in sleep mode for this to work.
    /// Check sensor mode beforehand and call [`Self::reset`] if necessary. To configure
    /// advanced settings, please directly update the respective registers.
    pub async fn configure<D: hal::delay::DelayNs>(&mut self, config: &Configuration) -> Result<(), Error<I::Error>> {
        self.write_register(
            OversamplingControl::default()
                .with_temperature_oversampling(config.temperature_oversampling)
                .with_pressure_oversampling(config.pressure_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        self.write_register(Config::default().with_iir_filter(config.iir_filter))
            .await
            .map_err(Error::Bus)?;

        Ok(())
    }

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        self.write_register(
            PowerControl::default()
                .with_sensor_mode(SensorMode::Forced)
                .with_temperature_enable(true)
                .with_pressure_enable(true),
        )
        .await
        .map_err(Error::Bus)?;

        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_m = self.read_register::<OversamplingControl>().await.map_err(Error::Bus)?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See section 3.9.2 of the datasheet for more information.
        let max_measurement_delay_us = 234 + (392 + 2020 * o_p.factor()) + (163 + o_t.factor() * 2020);
        delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self
            .read_register::<BurstMeasurements>()
            .await
            .map_err(Error::Bus)?
            .read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(Error::NotCalibrated);
        };

        let (temperature, t_fine) = cal.compensate_temperature(raw_data.temperature.temperature);
        let pressure = cal.compensate_pressure(raw_data.pressure.pressure, t_fine);

        Ok(Measurements { temperature, pressure })
    }
}
