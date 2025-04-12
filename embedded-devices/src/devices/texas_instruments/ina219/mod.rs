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
//! ```rust, only_if(sync)
//! # fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::ina219::{INA219Sync, address::Address, address::Pin};
//! use uom::num_rational::Rational32;
//! use uom::num_traits::ToPrimitive;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::rational32::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina219 = INA219Sync::new_i2c(i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina219.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(Rational32::new(1, 10)),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(Rational32::new(3, 1)),
//! ).unwrap();
//!
//! // One-shot read all values
//! let measurements = ina219.oneshot(&mut Delay).unwrap();
//! let bus_voltage = measurements.bus_voltage.get::<millivolt>().to_f32();
//! let current = measurements.current.get::<milliampere>().to_f32();
//! let power = measurements.power.get::<milliwatt>().to_f32();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW", bus_voltage, current, power);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::ina219::{INA219Async, address::Address, address::Pin};
//! use uom::num_rational::Rational32;
//! use uom::num_traits::ToPrimitive;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::rational32::{ElectricCurrent, ElectricalResistance};
//!
//! // Create and initialize the device
//! let mut ina219 = INA219Async::new_i2c(i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina219.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(Rational32::new(1, 10)),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(Rational32::new(3, 1)),
//! ).await.unwrap();
//!
//! // One-shot read all values
//! let measurements = ina219.oneshot(&mut Delay).await.unwrap();
//! let bus_voltage = measurements.bus_voltage.get::<millivolt>().to_f32();
//! let current = measurements.current.get::<milliampere>().to_f32();
//! let power = measurements.power.get::<milliwatt>().to_f32();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW", bus_voltage, current, power);
//! # Ok(())
//! # }
//! ```

use self::address::Address;

use embedded_devices_derive::{device, device_impl};
use uom::si::electric_current::ampere;
use uom::si::electrical_resistance::ohm;
use uom::si::rational32::{ElectricCurrent, ElectricPotential, ElectricalResistance, Power};

pub mod address;
pub mod registers;

type INA219I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Measured voltage across the shunt
    pub shunt_voltage: ElectricPotential,
    /// Measured voltage on the bus
    pub bus_voltage: ElectricPotential,
    /// Measured current
    pub current: ElectricCurrent,
    /// Measured power
    pub power: Power,
}

/// All possible errors that may occur during measurement
#[derive(Debug)]
pub enum MeasurementError<BusError> {
    /// Bus error
    Bus(BusError),
    /// The conversion ready flag was not set within the expected time frame.
    Timeout,
    /// Measurements were ready, but an overflow occurred. The power and
    /// current measurements may be incorrect.
    Overflow(Measurements),
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
pub struct INA219<I: embedded_registers::RegisterInterface> {
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
impl<I> INA219<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, INA219I2cCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you should call the [`Self::init`] method which
    /// saves the calibration values to enable current and power output.
    #[inline]
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
            shunt_resistance: Default::default(),
            max_expected_current: Default::default(),
            current_lsb_na: 1,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> INA219<I> {
    /// Soft-resets the device, calibrates it with the given shunt resistor
    /// value and maximum expected current.
    ///
    /// You can change the values later using [`Self::calibrate`].
    pub async fn init(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), I::Error> {
        self.reset().await?;
        self.calibrate(shunt_resistance, max_expected_current).await?;
        Ok(())
    }

    /// Performs a soft-reset of the device, restoring internal registers to power-on reset values.
    pub async fn reset(&mut self) -> Result<(), I::Error> {
        self.write_register(self::registers::Configuration::default().with_reset(true))
            .await?;

        Ok(())
    }

    /// Writes the given shunt resistance and maximum expected current into
    /// the device configuration register. This enables power and current output.
    pub async fn calibrate(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), I::Error> {
        self.shunt_resistance = shunt_resistance;
        self.max_expected_current = max_expected_current;

        let shunt_resistance = shunt_resistance.get::<ohm>();
        let max_expected_current = max_expected_current.get::<ampere>();

        self.current_lsb_na = ((1_000_000_000 * *max_expected_current.numer() as u64)
            / (*max_expected_current.denom() as u64 * (1 << 15))) as u32;
        let shunt_resistance_mohm = (1_000 * *shunt_resistance.numer() as u32) / (*shunt_resistance.denom() as u32);

        // Calibration Register = 0.04096 / (current_lsb (A) * shunt_resistance (Ω)),
        // Juggling some numbers around to get a high precision calculation with u32.
        let cal = 4_096_000_000 / (self.current_lsb_na * shunt_resistance_mohm / 10);
        self.write_register(self::registers::Calibration::default().with_raw_value(cal as u16))
            .await?;

        Ok(())
    }

    /// Returns the currently stored measurement values without triggering a new measurement.
    /// A timeout error cannot occur here.
    pub async fn read_measurements(&mut self) -> Result<Measurements, MeasurementError<I::Error>> {
        let bus_voltage = self
            .read_register::<self::registers::BusVoltage>()
            .await
            .map_err(MeasurementError::Bus)?;

        let shunt_voltage = self
            .read_register::<self::registers::ShuntVoltage>()
            .await
            .map_err(MeasurementError::Bus)?;

        let current = self
            .read_register::<self::registers::Current>()
            .await
            .map_err(MeasurementError::Bus)?;

        let power = self
            .read_register::<self::registers::Power>()
            .await
            .map_err(MeasurementError::Bus)?;

        let measurements = Measurements {
            shunt_voltage: shunt_voltage.read_voltage(),
            bus_voltage: bus_voltage.read_voltage(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        if bus_voltage.read_overflow() {
            Err(MeasurementError::Overflow(measurements))
        } else {
            Ok(measurements)
        }
    }

    /// Performs a one-shot measurement of all values.
    pub async fn oneshot<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurements, MeasurementError<I::Error>> {
        let reg_conf = self
            .read_register::<self::registers::Configuration>()
            .await
            .map_err(MeasurementError::Bus)?;

        // Initiate measurement
        self.write_register(reg_conf.with_operating_mode(self::registers::OperatingMode::ShuntAndBusTriggered))
            .await
            .map_err(MeasurementError::Bus)?;

        // Wait until measurement is ready, plus 1ms extra
        let measurement_time_us = reg_conf.read_bus_adc_resolution().conversion_time_us()
            + reg_conf.read_shunt_adc_resolution().conversion_time_us();
        delay.delay_us(1000 + measurement_time_us).await;

        // Wait for the conversion ready flag to be set
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let bus_voltage = self
                .read_register::<self::registers::BusVoltage>()
                .await
                .map_err(MeasurementError::Bus)?;

            if bus_voltage.read_conversion_ready() {
                let shunt_voltage = self
                    .read_register::<self::registers::ShuntVoltage>()
                    .await
                    .map_err(MeasurementError::Bus)?;

                let current = self
                    .read_register::<self::registers::Current>()
                    .await
                    .map_err(MeasurementError::Bus)?;

                // Reading this register clears the conversion_ready flag
                let power = self
                    .read_register::<self::registers::Power>()
                    .await
                    .map_err(MeasurementError::Bus)?;

                let measurements = Measurements {
                    shunt_voltage: shunt_voltage.read_voltage(),
                    bus_voltage: bus_voltage.read_voltage(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                };

                if bus_voltage.read_overflow() {
                    return Err(MeasurementError::Overflow(measurements));
                } else {
                    return Ok(measurements);
                }
            }

            delay.delay_us(1000).await;
        }

        Err(MeasurementError::Timeout)
    }
}
