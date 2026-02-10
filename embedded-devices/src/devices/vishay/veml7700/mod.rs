//! The VEML7700 is a very high sensitivity, high accuracy ambient light sensor.
//! It includes a highly sensitive photodiode, low-noise amplifier, 16-bit A/D converter,
//! and supports an easy-to-use I2C bus communication interface and additional interrupt feature.
//!
//! The ambient light read-out is available as a digital value, and the built-in photodiode
//! response is near that of the human eye. The 16-bit dynamic range for ambient light
//! detection is 0 lx to ~140 klx, with resolution down to 0.0042 lx/ct.
//!
//! Besides 100 Hz and 120 Hz flicker noise rejection and a low temperature coefficient,
//! the device consumes just 0.5 μA in shut down mode. In addition, another four Power saving
//! modes are available that allow operating current to be reduced down to just 2 μA.
//! The device operates within a temperature range of -25 °C to +85 °C.
//!
//! The VEML7700’s very high sensitivity of just 0.0042 lx allows the sensor to be placed
//! behind very dark cover glasses that will dramatically reduce the total light reaching it.
//! The sensor will also work behind clear cover glass, because even very high illumination
//! (such as direct sunlight) will not saturate the device and read-outs up to 140 klx are possible.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut i2c: I, delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::vishay::veml7700::{VEML7700Sync, address::Address};
//! use embedded_devices::sensor::OneshotSensorSync;
//!
//! // Create and initialize the device
//! let mut veml7700 = VEML7700Sync::new_i2c(delay, i2c, Address::Regular);
//! veml7700.init().unwrap();
//!
//! // One-shot read the ambient light level in lux
//! let measurement = veml7700.measure().unwrap();
//! println!("Ambient light measurement: {:?} lux", measurement.lux);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut i2c: I, delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::vishay::veml7700::{VEML7700Async, address::Address};
//! use embedded_devices::sensor::OneshotSensorAsync;
//!
//! // Create and initialize the device
//! let mut veml7700 = VEML7700Async::new_i2c(delay, i2c, Address::Regular);
//! veml7700.init().await.unwrap();
//!
//! // One-shot read the ambient light level in lux
//! let measurement = veml7700.measure().await.unwrap();
//! println!("Ambient light measurement: {:?} lux", measurement.lux);
//! # Ok(())
//! # }
//! # }
//! ```

use embedded_devices_derive::forward_register_fns;
use embedded_interfaces::TransportError;

use self::address::Address;

pub mod address;
pub mod registers;

use registers::{Configuration, PowerSaving};

//use uom::si::illuminance::lux; // requires uom 0.38.0

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, thiserror::Error)]
pub enum InitError<BusError> {
    /// Transport error
    #[error("transport error")]
    Transport(#[from] TransportError<(), BusError>),
    /// Invalid Device Id was encountered
    #[error("invalid device id {0:#02x}")]
    InvalidDeviceId(u8),
    /// Invalid Address Option was encountered
    #[error("invalid address option {0:#02x}")]
    InvalidAddressOption(u8),
}

#[derive(Debug, thiserror::Error)]
pub enum MeasurementError<BusError> {
    /// Transport error
    #[error("transport error")]
    Transport(#[from] TransportError<(), BusError>),
}

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    pub lux: f32, //uom::si::Illuminance,
}

/// The VEML7700 is a high accuracy digital ambient light sensor.
/// It has a wide dynamic rangeof 0 lx to 140 klx with a resolution ranging from
/// 0.0042 lx/ct to 2.1504 lx/ct depending on the used gain and integration time settings.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct VEML7700<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    delay: D,
    interface: I,
}

pub trait VEML7700Register {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> VEML7700<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    D: hal::delay::DelayNs,
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: Address) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}

#[forward_register_fns]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> VEML7700<D, I> {
    pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
        use registers::{AddressOption, DeviceID, DeviceIDCode};

        let id_reg = self.read_register::<DeviceID>().await?;

        let device_id = id_reg.read_device_id();
        if let DeviceIDCode::Invalid(val) = device_id {
            return Err(InitError::InvalidDeviceId(val));
        }

        let address_option = id_reg.read_address_option_code();
        if let AddressOption::Invalid(val) = address_option {
            return Err(InitError::InvalidAddressOption(val));
        }

        // reset to default configuration
        self.write_register(Configuration::default()).await?;
        self.write_register(PowerSaving::default()).await?;

        Ok(())
    }

    pub async fn configure(&mut self, config: &Configuration) -> Result<(), TransportError<(), I::BusError>> {
        // always shutdown before changing configuration
        if !config.read_shutdown() {
            self.write_register(config.with_shutdown(true)).await?;
        }

        self.write_register(config).await?;

        Ok(())
    }

    async fn measure_raw_lux_auto(
        &mut self,
        config: &mut Configuration,
        ps_sleep: u32,
    ) -> Result<f32, MeasurementError<I::BusError>> {
        const UP_THRESHOLD: u16 = 100;
        const DOWN_THRESHOLD: u16 = 10_000;

        if config.read_shutdown() {
            config.write_shutdown(false);
            self.write_register(*config).await?;
        }

        let mut gain = config.read_gain();
        let mut it = config.read_integration_time();
        let mut it_ms = it.ms();

        let mut raw_lux;

        loop {
            self.delay.delay_ms(10 + ps_sleep + it_ms).await;

            let raw_data = self.read_register::<registers::ALSData>().await?;
            raw_lux = raw_data.read_als_data();

            if raw_lux <= UP_THRESHOLD {
                if let Some(new_gain) = gain.increase_by_one() {
                    gain = new_gain;
                    config.write_gain(gain);
                } else if let Some(new_it) = it.increase_by_one() {
                    it = new_it;
                    it_ms = it.ms();
                    config.write_integration_time(it);
                } else {
                    break;
                }
            } else if raw_lux >= DOWN_THRESHOLD {
                if let Some(new_it) = it.decrease_by_one() {
                    it = new_it;
                    it_ms = it.ms();
                    config.write_integration_time(it);
                } else if let Some(new_gain) = gain.decrease_by_one() {
                    gain = new_gain;
                    config.write_gain(gain);
                } else {
                    break;
                }
            } else {
                break;
            }

            self.configure(config).await?;
        }

        Ok(raw_lux as f32 * config.resolution())
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        OneshotSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::sensor::OneshotSensor
    for VEML7700<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let mut config = self.read_register::<Configuration>().await?;
        let ps = self.read_register::<PowerSaving>().await?;

        config.write_shutdown(false);
        self.write_register(config).await?;

        let raw_lux = self.measure_raw_lux_auto(&mut config, ps.ms()).await?;

        Ok(Measurement {
            lux: lux_correction(raw_lux),
        })
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ContinuousSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::sensor::ContinuousSensor
    for VEML7700<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurements.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let mut config = self.read_register::<Configuration>().await?;
        config.write_shutdown(false);
        self.write_register(config).await?;

        Ok(())
    }

    /// Stops continuous measurements.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let mut config = self.read_register::<Configuration>().await?;
        config.write_shutdown(true);
        self.write_register(config).await?;

        Ok(())
    }

    /// Current measurement interval in microseconds.
    /// This is only a rough estimate since gain and integration time
    /// get ajdusted automatically depending on lighting conditions.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        let config = self.read_register::<Configuration>().await?;
        let ps = self.read_register::<PowerSaving>().await?;

        let interval_ms = 10 + ps.ms() + config.read_integration_time().ms();
        Ok(interval_ms * 1_000)
    }

    /// Returns the most recent measurement. Will never return [`None`].
    ///
    /// ## Warning
    ///
    /// Measurements may be invalid if the sensor has been reconfigured since the measurement has been taken.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let config = self.read_register::<Configuration>().await?;
        let raw_data = self.read_register::<registers::ALSData>().await?;
        let raw_lux = raw_data.read_als_data() as f32 * config.resolution();

        Ok(Some(Measurement {
            lux: lux_correction(raw_lux),
        }))
    }

    /// Checks if a new measurement is ready.
    /// Always returns `true` for VEML7700 since it has no data ready flag.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    /// Wait indefinitely until a new measurement is ready and returns it.
    /// Automatically adjusts gain and integration time.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        let mut config = self.read_register::<Configuration>().await?;
        let ps = self.read_register::<PowerSaving>().await?;

        let raw_lux = self.measure_raw_lux_auto(&mut config, ps.ms()).await?;
        Ok(Measurement {
            lux: lux_correction(raw_lux),
        })
    }
}

fn lux_correction(raw_lux: f32) -> f32 {
    if raw_lux <= 1000.0 {
        return raw_lux;
    }

    // 140klx is the maximum according to datasheet
    // 23_969.6 raw lux corresponds to 140klx with the correction formula
    if raw_lux >= 23_969.6 {
        return 140_000.0;
    }

    // coefficients from datasheet
    const A: f32 = 6.0135e-13;
    const B: f32 = -9.3924e-9;
    const C: f32 = 8.1488e-5;
    const D: f32 = 1.0023;

    let res_d = D * raw_lux;

    let raw_lux_pow = raw_lux * raw_lux;
    let res_c = C * raw_lux_pow;

    let res_b = B * raw_lux_pow * raw_lux;
    let res_a = A * raw_lux_pow * raw_lux_pow;

    let res = res_a + res_b + res_c + res_d;
    res.min(140_000.0)
}
