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
//! ## Usage
//!
//! ```
//! # async fn test<I, D>(mut i2c: I, mut Delay: D) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::texas_instruments::ina228::{INA228, address::Address, address::Pin};
//! use uom::num_rational::Rational64;
//! use uom::num_traits::ToPrimitive;
//! use uom::si::electric_current::{ampere, milliampere};
//! use uom::si::electric_potential::millivolt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::power::milliwatt;
//! use uom::si::rational64::{ElectricCurrent, ElectricalResistance};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut ina228 = INA228::new_i2c(i2c, Address::A0A1(Pin::Gnd, Pin::Gnd));
//! ina228.init(
//!    &mut Delay,
//!   // Most units use a 100mΩ shunt resistor
//!   ElectricalResistance::new::<ohm>(Rational64::new(1, 10)),
//!   // Maximum expected current 3A
//!   ElectricCurrent::new::<ampere>(Rational64::new(3, 1)),
//! ).await.unwrap();
//!
//! // One-shot read all values
//! let measurements = ina228.oneshot(&mut Delay).await.unwrap();
//! let bus_voltage = measurements.bus_voltage.get::<millivolt>().to_f32();
//! let temperature = measurements.temperature.get::<degree_celsius>().to_f32();
//! let current = measurements.current.get::<milliampere>().to_f32();
//! let power = measurements.power.get::<milliwatt>().to_f32();
//! println!("Current measurement: {:?}mV, {:?}mA, {:?}mW, {:?}°C", bus_voltage, current, power, temperature);
//! # Ok(())
//! # }
//! ```

use self::address::Address;

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{i2c::I2cDevice, RegisterInterface};
use registers::AdcRange;
use uom::num_rational::Rational64;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::electrical_resistance::ohm;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::rational64::{ElectricCurrent, ElectricPotential, ElectricalResistance, Power};

pub mod address;
pub mod registers;

type INA228I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur in device initialization
#[derive(Debug, defmt::Format)]
pub enum InitError<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid Device Id was encountered
    InvalidDeviceId,
    /// Invalid Manufacturer Id was encountered
    InvalidManufacturerId,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Measured voltage across the shunt
    pub shunt_voltage: ElectricPotential,
    /// Measured voltage on the bus
    pub bus_voltage: ElectricPotential,
    /// Measured die temperature
    pub temperature: ThermodynamicTemperature,
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

/// The INA228 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC specifically
/// designed for current-sensing applications. The device can measure a full-scale differential
/// input of ±163.84 mV or ±40.96 mV across a resistive shunt sense element with common-mode
/// voltage support from –0.3 V to +85 V.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct INA228<I: RegisterInterface> {
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

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> INA228<I2cDevice<I, hal::i2c::SevenBitAddress, INA228I2cCodec>>
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
            interface: I2cDevice::new(interface, address.into(), INA228I2cCodec::default()),
            shunt_resistance: Default::default(),
            max_expected_current: Default::default(),
            current_lsb_na: 1,
            adc_range: AdcRange::Div4,
        }
    }
}

#[device_impl]
impl<I: RegisterInterface> INA228<I> {
    /// Soft-resets the device, verifies its device id and manufacturer id and
    /// calibrates it with the given shunt resistor value and maximum expected current.
    ///
    /// You can change the values later using [`Self::calibrate`].
    pub async fn init<D: hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), InitError<I::Error>> {
        use registers::{DeviceId, ManufacturerId};

        // Reset the device and wait until it is ready. The datasheet specifies
        // 300µs startup time, so we round up to half a millisecond.
        self.reset().await.map_err(InitError::Bus)?;
        delay.delay_us(500).await;

        // Verify we are talking to the correct device
        let manufacturer_id = self.read_register::<ManufacturerId>().await.map_err(InitError::Bus)?;
        if manufacturer_id.read_id() != ManufacturerId::default().read_id() {
            return Err(InitError::InvalidDeviceId);
        }
        let device_id = self.read_register::<DeviceId>().await.map_err(InitError::Bus)?;
        if device_id.read_id() != DeviceId::default().read_id() {
            return Err(InitError::InvalidManufacturerId);
        }

        // Calibrate the device
        self.calibrate(shunt_resistance, max_expected_current)
            .await
            .map_err(InitError::Bus)?;
        Ok(())
    }

    /// Performs a soft-reset of the device, restoring internal registers to power-on reset values.
    pub async fn reset(&mut self) -> Result<(), I::Error> {
        self.write_register(self::registers::Configuration::default().with_reset(true))
            .await?;

        Ok(())
    }

    /// Writes the given shunt resistance and maximum expected current into
    /// the device configuration register. Automatically selects the adc range appropriately,
    /// leaving 10% headroom if possible.
    /// This required to enable power and current output.
    pub async fn calibrate(
        &mut self,
        shunt_resistance: ElectricalResistance,
        max_expected_current: ElectricCurrent,
    ) -> Result<(), I::Error> {
        self.shunt_resistance = shunt_resistance;
        self.max_expected_current = max_expected_current;
        let max_expected_shunt_voltage = shunt_resistance * max_expected_current;

        let shunt_resistance = shunt_resistance.get::<ohm>();
        let max_expected_current = max_expected_current.get::<ampere>();
        let max_expected_shunt_voltage = max_expected_shunt_voltage.get::<volt>();

        self.current_lsb_na = ((1_000_000_000 * *max_expected_current.numer() as i64)
            / (*max_expected_current.denom() as i64 * (1 << 19))) as i64;
        let shunt_resistance_mohm = (1_000 * *shunt_resistance.numer() as i64) / (*shunt_resistance.denom() as i64);

        // If the expected shunt voltage exceeds this threshold, we enable the input prescaler
        let div4_threshold = Rational64::new(36, 1000); // 36mV
        self.adc_range = if max_expected_shunt_voltage > div4_threshold {
            self::registers::AdcRange::Div4
        } else {
            self::registers::AdcRange::Div1
        };

        // Set adc range
        let reg_conf = self.read_register::<self::registers::Configuration>().await?;
        self.write_register(&reg_conf.with_adc_range(self.adc_range)).await?;

        // Calibration Register = 13107.2 x 10^6 x CURRENT_LSB (expected current / 1<<19) x R_SHUNT (in Ω) * 4 / adc_range,
        // we juggle some numbers around to get a higher precision calculation with u32.
        let cal = 524_288 * self.current_lsb_na * shunt_resistance_mohm / (10_000_000 * self.adc_range.factor() as i64);
        self.write_register(self::registers::ShuntCalibration::default().with_raw_value(cal as u16))
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

        let temperature = self
            .read_register::<self::registers::Temperature>()
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
            shunt_voltage: shunt_voltage.read_voltage(self.adc_range),
            bus_voltage: bus_voltage.read_voltage(),
            temperature: temperature.read_temperature(),
            current: current.read_current(self.current_lsb_na),
            power: power.read_power(self.current_lsb_na),
        };

        let diag = self
            .read_register::<self::registers::DiagnosticsAndAlert>()
            .await
            .map_err(MeasurementError::Bus)?;

        if diag.read_math_overflow() {
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
        let reg_adc_conf = self
            .read_register::<self::registers::AdcConfiguration>()
            .await
            .map_err(MeasurementError::Bus)?;

        // Initiate measurement
        self.write_register(
            &reg_adc_conf
                .with_operating_mode(self::registers::OperatingMode::Triggered)
                .with_enable_temperature(true)
                .with_enable_shunt(true)
                .with_enable_bus(true),
        )
        .await
        .map_err(MeasurementError::Bus)?;

        // Wait until measurement is ready, plus 100µs extra.
        let measurement_time_us = reg_adc_conf.read_bus_conversion_time().us()
            + reg_adc_conf.read_shunt_conversion_time().us()
            + reg_adc_conf.read_temperature_conversion_time().us();
        delay.delay_us(100 + measurement_time_us).await;

        // Wait for the conversion ready flag to be set
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            let diag = self
                .read_register::<self::registers::DiagnosticsAndAlert>()
                .await
                .map_err(MeasurementError::Bus)?;

            if diag.read_conversion_ready() {
                let bus_voltage = self
                    .read_register::<self::registers::BusVoltage>()
                    .await
                    .map_err(MeasurementError::Bus)?;

                let shunt_voltage = self
                    .read_register::<self::registers::ShuntVoltage>()
                    .await
                    .map_err(MeasurementError::Bus)?;

                let temperature = self
                    .read_register::<self::registers::Temperature>()
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
                    shunt_voltage: shunt_voltage.read_voltage(self.adc_range),
                    bus_voltage: bus_voltage.read_voltage(),
                    temperature: temperature.read_temperature(),
                    current: current.read_current(self.current_lsb_na),
                    power: power.read_power(self.current_lsb_na),
                };

                if diag.read_math_overflow() {
                    return Err(MeasurementError::Overflow(measurements));
                } else {
                    return Ok(measurements);
                }
            }

            delay.delay_us(100).await;
        }

        Err(MeasurementError::Timeout)
    }
}
