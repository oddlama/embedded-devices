//! The BL0942 is a built-in clock calibration-free energy metering IC, which is suitable for
//! single-phase multi-function electricity meters, smart sockets, smart home appliances and other
//! applications.
//!
//! The BL0942 incorporates two sigma delta ADCs with a high accuracy energy measurement core. It measures
//! line voltage and current and calculate active energy as well as instantaneous voltage and current.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I, D>(mut spi: I, delay: D) -> Result<(), embedded_interfaces::TransportError<embedded_devices::devices::belling::ChecksumError, I::Error>>
//! # where
//! #   I: embedded_hal::spi::SpiDevice,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::belling::bl0942::BL0942Sync;
//! use embedded_devices::sensor::OneshotSensorSync;
//! use uom::si::electric_current::ampere;
//! use uom::si::electric_potential::volt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::energy::watt_hour;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//! use uom::si::frequency::hertz;
//! use uom::si::power::watt;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bl0942 = BL0942Sync::new_spi(delay, spi,
//!     ElectricalResistance::new::<ohm>(0.001),
//!     ElectricalResistance::new::<ohm>(510.0),
//!     ElectricalResistance::new::<ohm>(5.0 * 390_000.0),
//! );
//! bl0942.init().unwrap();
//! let measurement = bl0942.measure().unwrap();
//! let voltage = measurement.voltage.get::<volt>();
//! let current = measurement.current.get::<ampere>();
//! let power = measurement.power.get::<watt>();
//! let energy = measurement.energy.get::<watt_hour>();
//! let frequency = measurement.frequency.get::<hertz>();
//! println!("Current measurement: {:?}V, {:?}A, {:?}W, {:?}Wh, {:?}Hz", voltage, current, power, energy, frequency);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I, D>(mut spi: I, delay: D) -> Result<(), embedded_interfaces::TransportError<embedded_devices::devices::belling::ChecksumError, I::Error>>
//! # where
//! #   I: embedded_hal_async::spi::SpiDevice,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::belling::bl0942::BL0942Async;
//! use embedded_devices::sensor::OneshotSensorAsync;
//! use uom::si::electric_current::ampere;
//! use uom::si::electric_potential::volt;
//! use uom::si::electrical_resistance::ohm;
//! use uom::si::energy::watt_hour;
//! use uom::si::f64::{ElectricCurrent, ElectricalResistance};
//! use uom::si::frequency::hertz;
//! use uom::si::power::watt;
//! use uom::si::thermodynamic_temperature::degree_celsius;
//!
//! // Create and initialize the device
//! let mut bl0942 = BL0942Async::new_spi(delay, spi,
//!     ElectricalResistance::new::<ohm>(0.001),
//!     ElectricalResistance::new::<ohm>(510.0),
//!     ElectricalResistance::new::<ohm>(5.0 * 390_000.0),
//! );
//! bl0942.init().await.unwrap();
//! let measurement = bl0942.measure().await.unwrap();
//! let voltage = measurement.voltage.get::<volt>();
//! let current = measurement.current.get::<ampere>();
//! let power = measurement.power.get::<watt>();
//! let energy = measurement.energy.get::<watt_hour>();
//! let frequency = measurement.frequency.get::<hertz>();
//! println!("Current measurement: {:?}V, {:?}A, {:?}W, {:?}Wh, {:?}Hz", voltage, current, power, energy, frequency);
//! # Ok(())
//! # }
//! # }
//! ```

use super::ChecksumError;
use embedded_devices_derive::forward_register_fns;
use embedded_devices_derive::sensor;
use embedded_interfaces::TransportError;
use uom::si::electrical_resistance::ohm;
use uom::si::f64::{ElectricCurrent, ElectricPotential, ElectricalResistance, Energy, Frequency, Power};

pub mod registers;

pub const V_REF: f64 = 1.218;
pub const V_RMS_COEFF: f64 = V_REF / (73989.0 * 1000.0);
pub const I_RMS_COEFF: f64 = V_REF / (305978.0 * 1000.0);
pub const I_FAST_RMS_FACTOR: f64 = 0.363;
pub const P_COEFF: f64 = V_REF * V_REF / (3537.0 * 1_000_000.0);
pub const E_COEFF: f64 = 1638.4 * 256.0 / 3600.0;

/// Measurement data
#[derive(Debug, embedded_devices_derive::Measurement)]
pub struct Measurement {
    /// Measured voltage
    #[measurement(Voltage)]
    pub voltage: ElectricPotential,
    /// Measured current
    #[measurement(Current)]
    pub current: ElectricCurrent,
    /// Measured power
    #[measurement(Power)]
    pub power: Power,
    /// Measured energy
    #[measurement(Energy)]
    pub energy: Energy,
    /// Measured line frequency
    #[measurement(Frequency)]
    pub frequency: Frequency,
}

/// The BL0942 is a built-in clock calibration-free energy metering IC, which is suitable for
/// single-phase multi-function electricity meters, smart sockets, smart home appliances and other
/// applications.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BL0942<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
    /// The shunt resistor value in Ohms.
    pub shunt_resistance: ElectricalResistance,
    /// The value of the voltage divider calculated as (R1 + R2) / R1
    /// where R1 is the small value (e.g. 1k) and R2 is the large value (e.g. 6*390k).
    pub voltage_divider_coeff: f64,
    /// Precomputed LSB value of the current register
    pub i_lsb: f64,
    /// Precomputed LSB value of the voltage register
    pub v_lsb: f64,
    /// Precomputed LSB value of the power register
    pub p_lsb: f64,
    /// Precomputed LSB value of the energy register
    pub e_lsb: f64,
}

pub trait BL0942Register {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> BL0942<D, embedded_interfaces::spi::SpiDevice<I>>
where
    I: hal::spi::r#SpiDevice,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The value of the voltage divider calculated as (R1 + R2) / R1
    /// where R1 is the small value (e.g. 1k) and R2 is the large value (e.g. 6*390k).
    ///
    /// The device requires SPI mode 1 (CPOL=0 and CPHA=1).
    #[inline]
    pub fn new_spi(
        delay: D,
        interface: I,
        shunt_resistance: ElectricalResistance,
        voltage_divider_r1: ElectricalResistance,
        voltage_divider_r2: ElectricalResistance,
    ) -> Self {
        let r1 = voltage_divider_r1.get::<ohm>();
        let r2 = voltage_divider_r2.get::<ohm>();
        let voltage_divider_coeff = (r1 + r2) / r1;
        let p_lsb = P_COEFF * voltage_divider_coeff / shunt_resistance.get::<ohm>();
        Self {
            delay,
            interface: embedded_interfaces::spi::SpiDevice::new(interface),
            shunt_resistance,
            voltage_divider_coeff,
            i_lsb: I_RMS_COEFF / shunt_resistance.get::<ohm>(),
            v_lsb: V_RMS_COEFF * voltage_divider_coeff,
            p_lsb,
            e_lsb: E_COEFF * p_lsb,
        }
    }
}

#[forward_register_fns]
#[sensor(Voltage, Current, Power, Energy)]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        ResettableDevice
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> BL0942<D, I> {
    /// Initializes the sensor by resetting it.
    pub async fn init(&mut self) -> Result<(), TransportError<ChecksumError, I::BusError>> {
        use crate::device::ResettableDevice;

        // Reset the device and wait until it is ready. The datasheet specifies
        // no startup time, we use 1 millisecond for good measure.
        self.reset().await?;
        self.delay.delay_ms(1).await;

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
    for BL0942<D, I>
{
    type Error = TransportError<ChecksumError, I::BusError>;

    /// Performs a soft-reset of the device, restoring internal registers to power-on reset values.
    async fn reset(&mut self) -> Result<(), Self::Error> {
        self.write_register(self::registers::WriteProtection::default().with_key(registers::ProtectionKey::Unlocked))
            .await?;
        self.write_register(self::registers::SoftReset::default().with_magic(registers::ResetMagic::Reset))
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
    for BL0942<D, I>
{
    type Error = TransportError<ChecksumError, I::BusError>;
    type Measurement = Measurement;

    /// Performs a one-shot measurement.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error> {
        use uom::si::{electric_current::ampere, electric_potential::volt, energy::watt_hour, power::watt};

        let i_rms = self.read_register::<self::registers::CurrentRms>().await?.read_value() as f64 * self.i_lsb;
        let v_rms = self.read_register::<self::registers::VoltageRms>().await?.read_value() as f64 * self.v_lsb;
        let power = self.read_register::<self::registers::ActivePower>().await?.read_value() as f64 * self.p_lsb;
        let energy = self
            .read_register::<self::registers::EnergyCounter>()
            .await?
            .read_value() as f64
            * self.e_lsb;
        let frequency = self
            .read_register::<self::registers::Frequency>()
            .await?
            .read_frequency();
        let measurement = Measurement {
            voltage: ElectricPotential::new::<volt>(v_rms),
            current: ElectricCurrent::new::<ampere>(i_rms),
            power: Power::new::<watt>(power),
            energy: Energy::new::<watt_hour>(energy),
            frequency,
        };

        Ok(measurement)
    }
}

#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface,
        OneshotSensor,
        ContinuousSensor
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> crate::sensor::ContinuousSensor
    for BL0942<D, I>
{
    type Error = TransportError<ChecksumError, I::BusError>;
    type Measurement = Measurement;

    /// Starts continuous measurement. No-op, always enabled.
    async fn start_measuring(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Stops continuous measurement. No-op, cannot disable.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Expected amount of time between measurements in microseconds.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error> {
        Ok(self
            .read_register::<self::registers::UserMode>()
            .await?
            .read_rms_update_rate()
            .as_us())
    }

    /// Returns the most recent measurement. Will never return None.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error> {
        use crate::sensor::OneshotSensor;
        Ok(Some(self.measure().await?))
    }

    /// Not supported, always returns true.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    /// Opportunistically waits one conversion interval and returns the measurement.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error> {
        use crate::sensor::OneshotSensor;
        let interval = self.measurement_interval_us().await?;
        self.delay.delay_us(interval).await;
        self.measure().await
    }
}
