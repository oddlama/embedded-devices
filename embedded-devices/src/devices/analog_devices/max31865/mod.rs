//! # MAX31865
//!
//! The MAX31865 is an easy-to-use resistance-to-digital converter optimized for platinum
//! resistance temperature detectors (RTDs). An external resistor sets the sensitivity
//! for the RTD being used and a precision delta-sigma ADC converts the ratio of the RTD
//! resistance to the reference resistance into digital form. The MAX31865’s inputs are
//! protected against overvoltage faults as large as ±45V. Programmable detection of RTD
//! and cable open and short conditions is included.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut spi: I, mut Delay: D) -> Result<(), embedded_devices::devices::analog_devices::max31865::MeasurementError<I::Error>>
//! # where
//! #   I: embedded_hal::spi::SpiDevice,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::analog_devices::max31865::{MAX31865Sync, registers::{FilterMode, Resistance, WiringMode}};
//! use embedded_devices::sensors::SensorSync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut max31865 = MAX31865Sync::new_spi(spi, 4.3);
//! max31865.init(&mut Delay, WiringMode::ThreeWire, FilterMode::F_50Hz).unwrap();
//!
//! let temp = max31865.measure(&mut Delay)?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut spi: I, mut Delay: D) -> Result<(), embedded_devices::devices::analog_devices::max31865::MeasurementError<I::Error>>
//! # where
//! #   I: embedded_hal_async::spi::SpiDevice,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::analog_devices::max31865::{MAX31865Async, registers::{FilterMode, Resistance, WiringMode}};
//! use embedded_devices::sensors::SensorAsync;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut max31865 = MAX31865Async::new_spi(spi, 4.3);
//! max31865.init(&mut Delay, WiringMode::ThreeWire, FilterMode::F_50Hz).await.unwrap();
//!
//! let temp = max31865.measure(&mut Delay).await?
//!     .temperature.get::<degree_celsius>();
//! println!("Current temperature: {:?}°C", temp);
//! # Ok(())
//! # }
//! # }
//! ```

use self::registers::FilterMode;
use self::registers::WiringMode;

use embedded_devices_derive::sensor;
use embedded_devices_derive::{device, device_impl};
use registers::FaultDetectionCycle;
use uom::si::f64::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::utils::callendar_van_dusen;
use crate::utils::from_bus_error;

pub mod registers;

/// All possible errors that may occur in fault detection
#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum FaultDetectionError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(BusError),
    /// Timeout (the detection never finished in the allocated time frame)
    #[error("fault detection timeout")]
    Timeout,
    /// A fault was detected. Read the FaultStatus register for details.
    #[error("fault detected")]
    FaultDetected,
}

/// All possible errors that may occur in temperature reads
#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum MeasurementError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(BusError),
    /// A fault was detected. Read the FaultStatus register for details.
    #[error("fault detected")]
    FaultDetected,
}

from_bus_error!(FaultDetectionError);
from_bus_error!(MeasurementError);

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Current temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
}

/// The MAX31865 is an easy-to-use resistance-to-digital converter optimized for platinum
/// resistance temperature detectors (RTDs).
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct MAX31865<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
    /// The reference resistor value over to the nominal resistance of
    /// the temperature element at 0°C (100Ω for PT100, 1000Ω for PT1000).
    /// In many designs a resistor with a value of 4.3 times the nominal resistance is used,
    /// but your design may vary.
    reference_resistor_ratio: f64,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> MAX31865<embedded_registers::spi::SpiDevice<I>>
where
    I: hal::spi::r#SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The device supports SPI modes 1 and 3.
    ///
    /// The reference resistor ratio is defined as the reference resistor value over the nominal resistance of
    /// the temperature element at 0°C (100Ω for PT100, 1000Ω for PT1000).
    /// In many designs a resistor with a value of 4.3 times the nominal resistance is used,
    /// but your design may vary.
    #[inline]
    pub fn new_spi(interface: I, reference_resistor_ratio: f64) -> Self {
        Self {
            interface: embedded_registers::spi::SpiDevice::new(interface),
            reference_resistor_ratio,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> MAX31865<I> {
    /// Initializes the device by configuring important settings and
    /// running an initial fault detection cycle.
    pub async fn init<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
        wiring_mode: WiringMode,
        filter_mode: FilterMode,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        // Configure the wiring mode and filter mode
        self.write_register(
            self::registers::Configuration::default()
                .with_wiring_mode(wiring_mode)
                .with_filter_mode(filter_mode),
        )
        .await?;

        self.write_register(self::registers::FaultThresholdLow::default().with_resistance_ratio(0))
            .await?;

        self.write_register(self::registers::FaultThresholdHigh::default().with_resistance_ratio(0x7fff))
            .await?;

        self.detect_faults(delay).await
    }

    /// Runs the automatic fault detection.
    pub async fn detect_faults<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;

        // The automatic fault detection waits 100µs before checking for faults,
        // which should be plenty of time to charge the RC filter.
        // Typically a 100nF cap is used which should charge significantly faster.
        self.write_register(
            reg_conf
                .with_enable_bias_voltage(true)
                .with_conversion_mode(self::registers::ConversionMode::NormallyOff)
                .with_clear_fault_status(false)
                .with_fault_detection_cycle(self::registers::FaultDetectionCycle::Automatic),
        )
        .await?;

        // According to the flow diagram in the datasheet, automatic calibration waits
        // a total of 510µs. We will wait for a bit longer initially and then check
        // the status in the configuration register up to 5 times with some additional delay.
        delay.delay_us(550).await;
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let cycle = self
                .read_register::<self::registers::Configuration>()
                .await?
                .read_fault_detection_cycle();

            // Check if fault detection is done
            if cycle == FaultDetectionCycle::Finished {
                // Disable VBIAS
                self.write_register(reg_conf.with_enable_bias_voltage(false)).await?;

                let has_fault = self.read_register::<registers::Resistance>().await?.read_fault();

                if has_fault {
                    // Fault detected
                    return Err(FaultDetectionError::FaultDetected);
                } else {
                    // Everything's fine!
                    return Ok(());
                }
            }

            delay.delay_us(100).await;
        }

        // Disable VBIAS
        self.write_register(reg_conf.with_enable_bias_voltage(false)).await?;

        Err(FaultDetectionError::Timeout)
    }

    /// Converts a temperature into a resistance ratio as understood
    /// by the device by factoring in the reference resistor ratio
    /// and converting it to the required raw data format for this device.
    ///
    /// If the temperature results in a ratio >= 1.0, the resulting value will be clamped
    /// to the maximum representable ratio (1.0).
    pub fn temperature_to_raw_resistance_ratio(&mut self, temperature: ThermodynamicTemperature) -> u16 {
        let temperature = temperature.get::<degree_celsius>() as f32;
        let resistance = callendar_van_dusen::temperature_to_resistance_r100(temperature);
        let ratio = resistance / (100.0 * self.reference_resistor_ratio) as f32;
        if ratio >= 1.0 {
            (1 << 15) - 1
        } else {
            (ratio * (1 << 15) as f32) as u16
        }
    }

    /// Converts a raw resistance ratio into a temperature by
    /// utilizing a builtin inverse Callendar-Van Dusen lookup table.
    pub fn raw_resistance_ratio_to_temperature(&mut self, raw_resistance: u16) -> ThermodynamicTemperature {
        // We always calculate with a 100Ω lookup table, because the equation
        // linearly scales to other temperature ranges. Only the ratio between
        // reference resistor and PT element is important.
        let resistance = (100.0 * raw_resistance as f32 * self.reference_resistor_ratio as f32) / ((1 << 15) as f32);
        let temperature = callendar_van_dusen::resistance_to_temperature_r100(resistance);
        ThermodynamicTemperature::new::<degree_celsius>(temperature as f64)
    }

    /// Read the latest resistance measurement from the device register and convert
    /// it to a temperature by using the internal Callendar-Van Dusen lookup table.
    ///
    /// Checks for faults.
    pub async fn read_temperature(&mut self) -> Result<ThermodynamicTemperature, MeasurementError<I::BusError>> {
        let resistance = self.read_register::<registers::Resistance>().await?;

        if resistance.read_fault() {
            return Err(MeasurementError::FaultDetected);
        }

        Ok(self.raw_resistance_ratio_to_temperature(resistance.read_resistance_ratio()))
    }
}

#[sensor(Temperature)]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface, Sensor),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> crate::sensors::Sensor for MAX31865<I> {
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will enable bias voltage, transition the device into
    /// oneshot mode, which will cause it to take a measurement and return to sleep mode
    /// afterwards.
    async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Self::Measurement, Self::Error> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;

        // Enable VBIAS before initiating one-shot measurement,
        // 10.5 RC time constants are recommended plus 1ms extra.
        // So we just wait 2ms which should be plenty of time.
        self.write_register(reg_conf.with_enable_bias_voltage(true)).await?;
        delay.delay_us(2000).await;

        // Initiate measurement
        self.write_register(reg_conf.with_enable_bias_voltage(true).with_oneshot(true))
            .await?;

        // Wait until measurement is ready, plus 2ms extra
        let measurement_time_us = match reg_conf.read_filter_mode() {
            FilterMode::F_60Hz => 52000,
            FilterMode::F_50Hz => 62500,
        };
        delay.delay_us(2000 + measurement_time_us).await;

        // Revert config (disable VBIAS)
        self.write_register(reg_conf).await?;

        // Return conversion result
        Ok(Measurement {
            temperature: self.read_temperature().await?,
        })
    }
}
