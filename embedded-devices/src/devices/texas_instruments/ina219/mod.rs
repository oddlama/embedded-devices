//! # INA219
//!
//! The INA219 is a current shunt and power monitor with an I2C- or SMBUS-compatible interface.
//! The device monitors both shunt voltage drop and bus supply voltage, with programmable
//! conversion times and filtering. A programmable calibration value, combined with
//! an internal multiplier, enables direct readouts of current in amperes. An additional
//! multiplying register calculates power in watts. The I2C- or SMBUS-compatible
//! interface features 16 programmable addresses.
//!
//! The INA219 is available in two grades: A and B. The B grade version has higher accuracy
//! and higher precision specifications.
//!
//! The INA219 senses across shunts on buses that can vary from 0 to 26 V. The device uses
//! a single 3- to 5.5-V supply, drawing a maximum of 1 mA of supply current. The INA219
//! operates from –40°C to 125°C.
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
//! use embedded_devices::devices::texas_instruments::ina219::{INA219Sync, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina219 = INA219Sync::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina219.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).unwrap();
//!
//! // One-shot read all values
//! let measurement = ina219.measure().unwrap();
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
//! use embedded_devices::devices::texas_instruments::ina219::{INA219Async, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina219 = INA219Async::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina219.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).await.unwrap();
//!
//! // One-shot read all values
//! let measurement = ina219.measure().await.unwrap();
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
use embedded_registers::TransportError;
use uom::si::electric_current::ampere;
use uom::si::electrical_resistance::ohm;
use uom::si::f64::{ElectricCurrent, ElectricPotential, ElectricalResistance, Power};

pub mod address;
pub mod registers;

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

/// The INA219 is a 12-bit current shunt and power monitor that can sense
/// on buses with 0-26V. It has a programmable gain amplifier to measure
/// full-scale shunt voltage ranges from 40mV to 320mV.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct INA219<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// Shunt resistance
    shunt_resistance: ElectricalResistance,
    /// Maximum expected current
    max_expected_current: ElectricCurrent,
    /// Configured nA/LSB for current readings
    current_lsb_na: u32,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> INA219<D, embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> INA219<D, I> {
    /// Soft-resets the device, calibrates it with the given shunt resistor
    /// value and maximum expected current.
    ///
    /// You can change the values later using [`Self::calibrate`].
    pub async fn init(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), TransportError<(), I::BusError>> {
        use crate::device::ResettableDevice;

        self.reset().await?;
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

        self.current_lsb_na = ((1_000_000_000f64 / (1 << 15) as f64) * max_expected_current) as u32;
        let shunt_resistance_mohm = (1_000.0 * shunt_resistance) as u32;

        // Calibration Register = 0.04096 / (current_lsb (A) * shunt_resistance (Ω)),
        // Juggling some numbers around to get a high precision calculation with u32.
        let cal = 4_096_000_000 / (self.current_lsb_na * shunt_resistance_mohm / 10);
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

        let measurement = Measurement {
            shunt_voltage: shunt_voltage.read_voltage(),
            bus_voltage: bus_voltage.read_voltage(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        if bus_voltage.read_overflow() {
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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::device::ResettableDevice
    for INA219<D, I>
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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::OneshotSensor for INA219<D, I> {
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

        // Wait until measurement is ready, plus 1ms extra
        let measurement_time_us = reg_conf.read_bus_adc_resolution().conversion_time_us()
            + reg_conf.read_shunt_adc_resolution().conversion_time_us();
        self.delay.delay_us(1000 + measurement_time_us).await;

        // Wait for the conversion ready flag to be set
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;

            if bus_voltage.read_conversion_ready() {
                let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
                let current = self.read_register::<self::registers::Current>().await?;
                // Reading this register clears the conversion_ready flag
                let power = self.read_register::<self::registers::Power>().await?;

                let measurement = Measurement {
                    shunt_voltage: shunt_voltage.read_voltage(),
                    bus_voltage: bus_voltage.read_voltage(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                };

                if bus_voltage.read_overflow() {
                    return Err(MeasurementError::Overflow(measurement));
                } else {
                    return Ok(measurement);
                }
            }

            self.delay.delay_us(1000).await;
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
impl<D: hal::delay::DelayNs, I: embedded_registers::RegisterInterface> crate::sensor::ContinuousSensor
    for INA219<D, I>
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
        let measurement_time_us = reg_conf.read_bus_adc_resolution().conversion_time_us()
            + reg_conf.read_shunt_adc_resolution().conversion_time_us();
        Ok(measurement_time_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
        let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
        let current = self.read_register::<self::registers::Current>().await?;
        // Reading this register clears the conversion_ready flag
        let power = self.read_register::<self::registers::Power>().await?;

        let measurement = Measurement {
            shunt_voltage: shunt_voltage.read_voltage(),
            bus_voltage: bus_voltage.read_voltage(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        if bus_voltage.read_overflow() {
            Err(ContinuousMeasurementError::Overflow(measurement))
        } else {
            Ok(Some(measurement))
        }
    }

    /// Check if new measurements are available.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
        Ok(bus_voltage.read_conversion_ready())
    }

    /// Wait indefinitely until new measurements are available and return them. Checks whether data
    /// is ready in intervals of 100us.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        loop {
            let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
            if bus_voltage.read_conversion_ready() {
                let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
                let current = self.read_register::<self::registers::Current>().await?;
                // Reading this register clears the conversion_ready flag
                let power = self.read_register::<self::registers::Power>().await?;

                let measurement = Measurement {
                    shunt_voltage: shunt_voltage.read_voltage(),
                    bus_voltage: bus_voltage.read_voltage(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                };

                if bus_voltage.read_overflow() {
                    return Err(ContinuousMeasurementError::Overflow(measurement));
                } else {
                    return Ok(measurement);
                }
            }

            self.delay.delay_us(100).await;
        }
    }
}
