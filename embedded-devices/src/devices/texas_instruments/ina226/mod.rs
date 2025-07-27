//! # INA226
//!
//! The INA226 is a current shunt and power monitor with an I2C- or SMBUS-compatible interface.
//! The device monitors both a shunt voltage drop and bus supply voltage.
//! Programmable calibration value, conversion times, and averaging,combined with an
//! internal multiplier, enable direct readouts of current in amperes and power in watts.
//!
//! The INA226 senses current on common-mode bus voltages that can vary from 0V to 36V,
//! independent of the supply voltage. The device operates from a single 2.7V to 5.5V supply,
//! drawing a typical of 330μA of supply current. The device is specified over the
//! operating temperature range between –40°C and 125°C and features
//! up to 16 programmable addresses on the I2C-compatible interface.
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
//! use embedded_devices::devices::texas_instruments::ina226::{INA226Sync, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina226 = INA226Sync::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina226.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).unwrap();
//!
//! // One-shot read all values
//! let measurement = ina226.measure().unwrap();
//! let bus_voltage = measurement.bus_voltage.get::<millivolt>();
//! let current = measurement.current.get::<milliampere>();
//! let power = measurement.power.get::<milliwatt>();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW", bus_voltage, current, power);
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
//! use embedded_devices::devices::texas_instruments::ina226::{INA226Async, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina226 = INA226Async::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina226.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).await.unwrap();
//!
//! // One-shot read all values
//! let measurement = ina226.measure().await.unwrap();
//! let bus_voltage = measurement.bus_voltage.get::<millivolt>();
//! let current = measurement.current.get::<milliampere>();
//! let power = measurement.power.get::<milliwatt>();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW", bus_voltage, current, power);
//! # Ok(())
//! # }
//! # }
//! ```

use crate::utils::from_bus_error;

use self::address::Address;

use embedded_devices_derive::{device, device_impl, sensor};
use embedded_interfaces::TransportError;
use uom::si::electric_current::ampere;
use uom::si::electrical_resistance::ohm;
use uom::si::f64::{ElectricCurrent, ElectricPotential, ElectricalResistance, Power};

pub mod address;
pub mod registers;

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum InitError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Invalid Die Id was encountered
    #[error("invalid die id {0:#04x}")]
    InvalidDieId(u16),
    /// Invalid Manufacturer Id was encountered
    #[error("invalid manufacturer id {0:#04x}")]
    InvalidManufacturerId(u16),
}

#[derive(Debug, thiserror::Error)]
pub enum MeasurementError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// The conversion ready flag was not set within the expected time frame.
    #[error("conversion timeout")]
    Timeout,
    /// Measurement was ready, but an overflow occurred. The power and
    /// current measurement may be incorrect.
    #[error("overflow in measurement")]
    Overflow(Measurement),
}

#[derive(Debug, thiserror::Error)]
pub enum ContinuousMeasurementError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Measurement was ready, but an overflow occurred. The power and
    /// current measurement may be incorrect.
    #[error("overflow in measurement")]
    Overflow(Measurement),
}

from_bus_error!(InitError);
from_bus_error!(MeasurementError);
from_bus_error!(ContinuousMeasurementError);

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Measured voltage across the shunt
    pub shunt_voltage: ElectricPotential,
    /// Measured voltage on the bus
    #[measurement(Voltage)]
    pub bus_voltage: ElectricPotential,
    /// Measured current
    #[measurement(Current)]
    pub current: ElectricCurrent,
    /// Measured power
    #[measurement(Power)]
    pub power: Power,
}

/// The INA226 is an ultra-precise digital power monitor with a 16-bit delta-sigma ADC
/// for high- or low-side current-sensing applications. The device can measure a full-scale
/// differential input of ±81.92 mV across a resistive shunt sense element with bus
/// voltage support from 0 V to +36 V.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct INA226<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// Shunt resistance
    shunt_resistance: ElectricalResistance,
    /// Maximum expected current
    max_expected_current: ElectricCurrent,
    /// Configured nA/LSB for current readings
    current_lsb_na: i64,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> INA226<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// saves the calibration values to enable current and power output.
    #[inline]
    pub fn new_i2c(delay: D, interface: I, address: Address) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::i2c::I2cDevice::new(interface, address.into()),
            shunt_resistance: Default::default(),
            max_expected_current: Default::default(),
            current_lsb_na: 1,
        }
    }
}

#[device_impl]
#[sensor(Voltage, Current, Power)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> INA226<D, I> {
    /// Soft-resets the device, calibrates it with the given shunt resistor
    /// value and maximum expected current.
    ///
    /// You can change the values later using [`Self::calibrate`].
    pub async fn init(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), InitError<I::BusError>> {
        use crate::device::ResettableDevice;
        use registers::{DieId, ManufacturerId};

        // Reset the device and wait for 0.5ms. The datasheet does not define
        // a specific startup time so we use a similar value as the INA228.
        self.reset().await?;
        self.delay.delay_us(500).await;

        // Verify we are talking to the correct device
        let manufacturer_id = self.read_register::<ManufacturerId>().await?.read_id();
        if manufacturer_id != ManufacturerId::default().read_id() {
            return Err(InitError::InvalidManufacturerId(manufacturer_id));
        }
        let die_id = self.read_register::<DieId>().await?.read_id();
        if die_id != DieId::default().read_id() {
            return Err(InitError::InvalidDieId(die_id));
        }

        self.calibrate(shunt_resistance, max_expected_current).await?;

        Ok(())
    }

    /// Writes the given shunt resistance and maximum expected current into
    /// the device configuration register. This enables power and current output.
    pub async fn calibrate(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), TransportError<(), I::BusError>> {
        self.shunt_resistance = shunt_resistance;
        self.max_expected_current = max_expected_current;

        let shunt_resistance = shunt_resistance.get::<ohm>();
        let max_expected_current = max_expected_current.get::<ampere>();

        self.current_lsb_na = ((1_000_000_000f64 / (1 << 15) as f64) * max_expected_current) as i64;
        let shunt_resistance_mohm = (1_000.0 * shunt_resistance) as i64;

        // Calibration Register = 0.00512 / (current_lsb (A) * shunt_resistance (Ω))
        let cal = 5_120_000_000 / (self.current_lsb_na * shunt_resistance_mohm);
        self.write_register(self::registers::Calibration::default().with_raw_value(cal as u16))
            .await?;

        Ok(())
    }

    /// Returns the currently stored measurement values without triggering a new measurement.
    /// A timeout error cannot occur here.
    pub async fn read_measurements(&mut self) -> Result<Measurement, MeasurementError<I::BusError>> {
        let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
        let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
        let current = self.read_register::<self::registers::Current>().await?;
        let power = self.read_register::<self::registers::Power>().await?;
        let flags = self.read_register::<self::registers::MaskEnable>().await?;

        let measurement = Measurement {
            shunt_voltage: shunt_voltage.read_value(),
            bus_voltage: bus_voltage.read_value(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        if flags.read_math_overflow_flag() {
            Err(MeasurementError::Overflow(measurement))
        } else {
            Ok(measurement)
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::device::ResettableDevice
    for INA226<D, I>
{
    type Error = TransportError<(), I::BusError>;

    /// Performs a soft-reset of the device, restoring internal registers to power-on reset values.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        self.write_register(self::registers::Configuration::default().with_reset(true))
            .await?;
        Ok(())
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
    for INA226<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will set the operating mode to
    /// [`self::registers::OperatingMode::ShuntAndBusTriggered´] and enable all conversion outputs causing the
    /// device to perform a single conversion a return to sleep afterwards.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;

        // Initiate measurement
        self.write_register(reg_conf.with_operating_mode(self::registers::OperatingMode::ShuntAndBusTriggered))
            .await?;

        // Wait until measurement is ready, plus 10% and 1ms extra
        let mut measurement_time_us = reg_conf.total_conversion_time_us();
        measurement_time_us += measurement_time_us / 10; // 10% extra (data sheet conversion times state ~10% higher maximum times)
        self.delay.delay_us(1000 + measurement_time_us).await;

        // Wait for the conversion ready flag to be set
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let flags = self.read_register::<self::registers::MaskEnable>().await?;
            if !flags.read_conversion_ready_flag() {
                self.delay.delay_us(1000).await;
                continue;
            }

            let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
            let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
            let current = self.read_register::<self::registers::Current>().await?;
            // Reading this register clears the conversion_ready flag
            let power = self.read_register::<self::registers::Power>().await?;

            let measurement = Measurement {
                shunt_voltage: shunt_voltage.read_value(),
                bus_voltage: bus_voltage.read_value(),
                current: current.read_current(self.current_lsb_na),
                power: power.read_power(self.current_lsb_na),
            };

            if flags.read_math_overflow_flag() {
                return Err(MeasurementError::Overflow(measurement));
            } else {
                return Ok(measurement);
            }
        }

        Err(MeasurementError::Timeout)
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
    for INA226<D, I>
{
    type Error = ContinuousMeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;
        self.write_register(reg_conf.with_operating_mode(self::registers::OperatingMode::ShuntAndBusContinuous))
            .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;
        self.write_register(reg_conf.with_operating_mode(self::registers::OperatingMode::PowerDown))
            .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;
        let mut measurement_time_us = reg_conf.total_conversion_time_us();
        measurement_time_us += measurement_time_us / 10; // 10% extra (data sheet conversion times state ~10% higher maximum times)
        Ok(measurement_time_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let flags = self.read_register::<self::registers::MaskEnable>().await?;
        let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
        let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
        let current = self.read_register::<self::registers::Current>().await?;
        // Reading this register clears the conversion_ready flag
        let power = self.read_register::<self::registers::Power>().await?;

        let measurement = Measurement {
            shunt_voltage: shunt_voltage.read_value(),
            bus_voltage: bus_voltage.read_value(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        if flags.read_math_overflow_flag() {
            Err(ContinuousMeasurementError::Overflow(measurement))
        } else {
            Ok(Some(measurement))
        }
    }

    /// Check if new measurements are available.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        let flags = self.read_register::<self::registers::MaskEnable>().await?;
        Ok(flags.read_conversion_ready_flag())
    }

    /// Wait indefinitely until new measurements are available and return them. Checks whether data
    /// is ready in intervals of 100us.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        loop {
            let flags = self.read_register::<self::registers::MaskEnable>().await?;
            if flags.read_conversion_ready_flag() {
                let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
                let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
                let current = self.read_register::<self::registers::Current>().await?;
                // Reading this register clears the conversion_ready flag
                let power = self.read_register::<self::registers::Power>().await?;

                let measurement = Measurement {
                    shunt_voltage: shunt_voltage.read_value(),
                    bus_voltage: bus_voltage.read_value(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                };

                if flags.read_math_overflow_flag() {
                    return Err(ContinuousMeasurementError::Overflow(measurement));
                } else {
                    return Ok(measurement);
                }
            }

            self.delay.delay_us(100).await;
        }
    }
}
