//! # INA228
//!
//! The INA228 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC specifically
//! designed for current-sensing applications. The device can measure a full-scale differential
//! input of ±163.84 mV or ±40.96 mV across a resistive shunt sense element with common-mode
//! voltage support from –0.3 V to +85 V.
//!
//! The INA228 reports current, bus voltage, temperature, power, energy and charge accumulation
//! while employing a precision ±0.5% integrated oscillator, all while performing the needed
//! calculations in the background. The integrated temperature sensor is ±1°C accurate for die
//! temperature measurement and is useful in monitoring the system ambient temperature.
//!
//! The low offset and gain drift design of the INA228 allows the device to be used in precise
//! systems that do not undergo multi-temperature calibration during manufacturing. Further, the
//! very low offset voltage and noise allow for use in mA to kA sensing applications and provide a
//! wide dynamic range without significant power dissipation losses on the sensing shunt element.
//! The low input bias current of the device permits the use of larger current- sense resistors,
//! thus providing accurate current measurements in the micro-amp range.
//!
//! The device allows for selectable ADC conversion times from 50 μs to 4.12 ms as well as sample
//! averaging from 1x to 1024x, which further helps reduce the noise of the measured data.
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
//! use embedded_devices::devices::texas_instruments::ina228::{INA228Sync, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut ina228 = INA228Sync::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina228.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).unwrap();
//!
//! // One-shot read all values
//! let measurement = ina228.measure().unwrap();
//! let bus_voltage = measurement.bus_voltage.get::<millivolt>();
//! let temperature = measurement.temperature.get::<degree_celsius>();
//! let current = measurement.current.get::<milliampere>();
//! let power = measurement.power.get::<milliwatt>();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW, {:?}°C", bus_voltage, current, power, temperature);
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
//! use embedded_devices::devices::texas_instruments::ina228::{INA228Async, address::Address, address::Pin};
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut ina228 = INA228Async::new_i2c(delay, i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina228.init(
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(0.1),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(3.0),
//! ).await.unwrap();
//!
//! // One-shot read all values
//! let measurement = ina228.measure().await.unwrap();
//! let bus_voltage = measurement.bus_voltage.get::<millivolt>();
//! let temperature = measurement.temperature.get::<degree_celsius>();
//! let current = measurement.current.get::<milliampere>();
//! let power = measurement.power.get::<milliwatt>();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW, {:?}°C", bus_voltage, current, power, temperature);
//! # Ok(())
//! # }
//! # }
//! ```

use crate::utils::from_bus_error;

use self::address::Address;

use embedded_devices_derive::{forward_register_fns, sensor};
use embedded_interfaces::TransportError;
use registers::AdcRange;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::electrical_resistance::ohm;
use uom::si::f64::ThermodynamicTemperature;
use uom::si::f64::{ElectricCharge, ElectricCurrent, ElectricPotential, ElectricalResistance, Energy, Power};

pub mod address;
pub mod registers;

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum InitError<BusError> {
    /// Bus error
    #[error("bus error")]
    Bus(#[from] BusError),
    /// Invalid Device Id was encountered
    #[error("invalid device id {0:#04x}")]
    InvalidDeviceId(u16),
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
    /// Measured die temperature
    #[measurement(Temperature)]
    pub temperature: ThermodynamicTemperature,
    /// Measured current
    #[measurement(Current)]
    pub current: ElectricCurrent,
    /// Measured power
    #[measurement(Power)]
    pub power: Power,
    /// Measured energy
    #[measurement(Energy)]
    pub energy: Energy,
    /// Measured charge
    #[measurement(Charge)]
    pub charge: ElectricCharge,
}

/// The INA228 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC specifically
/// designed for current-sensing applications. The device can measure a full-scale differential
/// input of ±163.84 mV or ±40.96 mV across a resistive shunt sense element with common-mode
/// voltage support from –0.3 V to +85 V.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct INA228<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// Shunt resistance
    shunt_resistance: ElectricalResistance,
    /// Maximum expected current
    max_expected_current: ElectricCurrent,
    /// Configured nA/LSB for current readings
    pub current_lsb_na: i64,
    /// The configured adc range
    pub adc_range: self::registers::AdcRange,
}

pub trait INA228Register {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> INA228<D, embedded_interfaces::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
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
            adc_range: AdcRange::Div4,
        }
    }
}

#[forward_register_fns]
#[sensor(Voltage, Temperature, Current, Power, Energy, Charge)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> INA228<D, I> {
    /// Soft-resets the device, verifies its device id and manufacturer id and
    /// calibrates it with the given shunt resistor value and maximum expected current.
    ///
    /// You can change the values later using [`Self::calibrate`].
    pub async fn init(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), InitError<I::BusError>> {
        use crate::device::ResettableDevice;
        use registers::{DeviceId, ManufacturerId};

        // Reset the device and wait until it is ready. The datasheet specifies
        // 300µs startup time, so we round up to half a millisecond.
        self.reset().await?;
        self.delay.delay_us(500).await;

        // Verify we are talking to the correct device
        let manufacturer_id = self.read_register::<ManufacturerId>().await?.read_id();
        if manufacturer_id != ManufacturerId::default().read_id() {
            return Err(InitError::InvalidManufacturerId(manufacturer_id));
        }
        let device_id = self.read_register::<DeviceId>().await?.read_id();
        if device_id != DeviceId::default().read_id() {
            return Err(InitError::InvalidDeviceId(device_id));
        }

        // Calibrate the device
        self.calibrate(shunt_resistance, max_expected_current).await?;
        Ok(())
    }

    /// Writes the given shunt resistance and maximum expected current into the device
    /// configuration register. Automatically selects the adc range appropriately, leaving 10%
    /// headroom if possible. This required to enable power and current output.
    pub async fn calibrate(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), TransportError<(), I::BusError>> {
        self.shunt_resistance = shunt_resistance;
        self.max_expected_current = max_expected_current;
        let max_expected_shunt_voltage = shunt_resistance * max_expected_current;

        let shunt_resistance = shunt_resistance.get::<ohm>();
        let max_expected_current = max_expected_current.get::<ampere>();
        let max_expected_shunt_voltage = max_expected_shunt_voltage.get::<volt>();

        self.current_lsb_na = ((1_000_000_000f64 / (1 << 19) as f64) * max_expected_current) as i64;
        let shunt_resistance_mohm = (1_000.0 * shunt_resistance) as i64;

        // If the expected shunt voltage exceeds this threshold, we enable the input prescaler
        let div4_threshold = 0.036; // 36mV
        self.adc_range = if max_expected_shunt_voltage > div4_threshold {
            self::registers::AdcRange::Div4
        } else {
            self::registers::AdcRange::Div1
        };

        // Set adc range
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;
        self.write_register(reg_conf.with_adc_range(self.adc_range)).await?;

        // Calibration Register = 13107.2 x 10^6 x CURRENT_LSB (expected current / 1<<19) x R_SHUNT (in Ω) * 4 / adc_range,
        // we juggle some numbers around to get a higher precision calculation with u32.
        let cal = 524_288 * self.current_lsb_na * shunt_resistance_mohm / (10_000_000 * self.adc_range.factor() as i64);
        self.write_register(self::registers::ShuntCalibration::default().with_raw_value(cal as u16))
            .await?;

        Ok(())
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
    for INA228<D, I>
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
    for INA228<D, I>
{
    type Error = MeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement. This will set the operating mode to
    /// [`self::registers::OperatingMode::Triggered´] and enable all conversion outputs causing the
    /// device to perform a single conversion a return to sleep afterwards.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        let reg_adc_conf = self.read_register::<self::registers::AdcConfiguration>().await?;

        // Initiate measurement
        self.write_register(
            reg_adc_conf
                .with_operating_mode(self::registers::OperatingMode::Triggered)
                .with_enable_temperature(true)
                .with_enable_shunt(true)
                .with_enable_bus(true),
        )
        .await?;

        // Wait until measurement is ready, plus 100µs extra.
        let measurement_time_us = reg_adc_conf.read_bus_conversion_time().us()
            + reg_adc_conf.read_shunt_conversion_time().us()
            + reg_adc_conf.read_temperature_conversion_time().us();
        self.delay
            .delay_us(100 + measurement_time_us * reg_adc_conf.read_average_count().factor() as u32)
            .await;

        // Wait for the conversion ready flag to be set
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let diag = self.read_register::<self::registers::DiagnosticsAndAlert>().await?;
            if diag.read_conversion_ready() {
                let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
                let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
                let temperature = self.read_register::<self::registers::Temperature>().await?;
                let current = self.read_register::<self::registers::Current>().await?;
                let energy = self.read_register::<self::registers::Energy>().await?;
                let charge = self.read_register::<self::registers::Charge>().await?;
                // Reading this register clears the conversion_ready flag
                let power = self.read_register::<self::registers::Power>().await?;

                let measurement = Measurement {
                    shunt_voltage: shunt_voltage.read_voltage(self.adc_range),
                    bus_voltage: bus_voltage.read_value(),
                    temperature: temperature.read_value(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                    energy: energy.read_energy(self.current_lsb_na),
                    charge: charge.read_charge(self.current_lsb_na),
                };

                if diag.read_math_overflow() {
                    return Err(MeasurementError::Overflow(measurement));
                } else {
                    return Ok(measurement);
                }
            }

            self.delay.delay_us(100).await;
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
    for INA228<D, I>
{
    type Error = ContinuousMeasurementError<I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_adc_conf = self.read_register::<self::registers::AdcConfiguration>().await?;
        self.write_register(
            reg_adc_conf
                .with_operating_mode(self::registers::OperatingMode::Continuous)
                .with_enable_temperature(true)
                .with_enable_shunt(true)
                .with_enable_bus(true),
        )
        .await?;
        Ok(())
    }

    /// Stops continuous measurement.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        let reg_adc_conf = self.read_register::<self::registers::AdcConfiguration>().await?;
        self.write_register(
            reg_adc_conf
                .with_operating_mode(self::registers::OperatingMode::Continuous)
                .with_enable_temperature(false)
                .with_enable_shunt(false)
                .with_enable_bus(false),
        )
        .await?;
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        let reg_adc_conf = self.read_register::<self::registers::AdcConfiguration>().await?;
        let measurement_time_us = reg_adc_conf.read_bus_conversion_time().us()
            + reg_adc_conf.read_shunt_conversion_time().us()
            + reg_adc_conf.read_temperature_conversion_time().us();
        Ok(measurement_time_us)
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        let diag = self.read_register::<self::registers::DiagnosticsAndAlert>().await?;
        let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
        let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
        let temperature = self.read_register::<self::registers::Temperature>().await?;
        let current = self.read_register::<self::registers::Current>().await?;
        let energy = self.read_register::<self::registers::Energy>().await?;
        let charge = self.read_register::<self::registers::Charge>().await?;
        // Reading this register clears the conversion_ready flag
        let power = self.read_register::<self::registers::Power>().await?;

        let measurement = Measurement {
            shunt_voltage: shunt_voltage.read_voltage(self.adc_range),
            bus_voltage: bus_voltage.read_value(),
            temperature: temperature.read_value(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
            energy: energy.read_energy(self.current_lsb_na),
            charge: charge.read_charge(self.current_lsb_na),
        };

        if diag.read_math_overflow() {
            Err(ContinuousMeasurementError::Overflow(measurement))
        } else {
            Ok(Some(measurement))
        }
    }

    /// Check if new measurements are available.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        let diag = self.read_register::<self::registers::DiagnosticsAndAlert>().await?;
        Ok(diag.read_conversion_ready())
    }

    /// Wait indefinitely until new measurements are available and return them. Checks whether data
    /// is ready in intervals of 100us.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        loop {
            let diag = self.read_register::<self::registers::DiagnosticsAndAlert>().await?;
            if diag.read_conversion_ready() {
                let bus_voltage = self.read_register::<self::registers::BusVoltage>().await?;
                let shunt_voltage = self.read_register::<self::registers::ShuntVoltage>().await?;
                let temperature = self.read_register::<self::registers::Temperature>().await?;
                let current = self.read_register::<self::registers::Current>().await?;
                let energy = self.read_register::<self::registers::Energy>().await?;
                let charge = self.read_register::<self::registers::Charge>().await?;
                // Reading this register clears the conversion_ready flag
                let power = self.read_register::<self::registers::Power>().await?;

                let measurement = Measurement {
                    shunt_voltage: shunt_voltage.read_voltage(self.adc_range),
                    bus_voltage: bus_voltage.read_value(),
                    temperature: temperature.read_value(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                    energy: energy.read_energy(self.current_lsb_na),
                    charge: charge.read_charge(self.current_lsb_na),
                };

                if diag.read_math_overflow() {
                    return Err(ContinuousMeasurementError::Overflow(measurement));
                } else {
                    return Ok(measurement);
                }
            }

            self.delay.delay_us(100).await;
        }
    }
}
