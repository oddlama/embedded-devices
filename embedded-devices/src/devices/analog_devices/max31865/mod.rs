//! # MAX31865
//!
//! The MAX31865 is an easy-to-use resistance-to-digital converter optimized for platinum
//! resistance temperature detectors (RTDs). An external resistor sets the sensitivity
//! for the RTD being used and a precision delta-sigma ADC converts the ratio of the RTD
//! resistance to the reference resistance into digital form. The MAX31865’s inputs are
//! protected against overvoltage faults as large as ±45V. Programmable detection of RTD
//! and cable open and short conditions is included.
//!
//! ## Usage
//!
//! ```
//! # async fn test<I>(mut spi: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::spi::SpiDevice
//! # {
//! use embedded_devices::devices::analog_devices::max31865::{MAX31865, registers::Resistance};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//! use uom::num_rational::Rational32;
//!
//! // Create and initialize the device
//! let mut max31865 = MAX31865::new_spi(spi, Rational32::new(43, 10));
//! max31865.init().await.unwrap();
//!
//! let ratio = max31865
//!     .read_temperature()
//!     .await?
//!     .get::<degree_celsius>();
//! println!("Current raw resistance ratio: {:?}", ratio);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{spi::SpiDevice, RegisterInterface};
use uom::num_rational::Rational32;
use uom::si::f32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

use crate::utils::callendar_van_dusen;

pub mod registers;

type MAX31865SpiCodec = embedded_registers::spi::codecs::SimpleCodec<1, 6, 0, 7, false, 0>;

/// The MAX31865 is an easy-to-use resistance-to-digital converter optimized for platinum
/// resistance temperature detectors (RTDs).
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[device]
pub struct MAX31865<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
    /// The ratio of the reference resistor compared to the nominal resistance of
    /// the temperature element at 0°C (100Ω for PT100, 1000Ω for PT1000).
    /// In many designs a resistor with a value of 4.3 times the nominal resistance is used,
    /// but your design may vary.
    reference_resistor_ratio: Rational32,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> MAX31865<SpiDevice<I, MAX31865SpiCodec>>
where
    I: hal::spi::SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    #[inline]
    pub fn new_spi(interface: I, reference_resistor_ratio: Rational32) -> Self {
        Self {
            interface: SpiDevice {
                interface,
                default_codec: MAX31865SpiCodec::default(),
            },
            reference_resistor_ratio,
        }
    }
}

#[device_impl]
impl<I: RegisterInterface> MAX31865<I> {
    /// TODO: run selftest
    pub async fn init(&mut self) -> Result<(), I::Error> {
        Ok(())
    }

    /// Converts a temperature into a resistance ratio as understood
    /// by the device by factoring in the reference resistor ratio
    /// and converting it to the required raw data format for this device.
    ///
    /// If the temperature results in a ratio >= 1.0, the resulting value will be clamped
    /// to the maximum representable ratio (1.0).
    pub fn temperature_to_raw_resistance_ratio(&mut self, temperature: ThermodynamicTemperature) -> u16 {
        let temperature: f32 = temperature.get::<degree_celsius>();
        let resistance = callendar_van_dusen::temperature_to_resistance_r100(temperature);
        let inv_reference_resistor_ratio =
            *self.reference_resistor_ratio.denom() as f32 / *self.reference_resistor_ratio.numer() as f32;
        let ratio = resistance / 100.0 * inv_reference_resistor_ratio;
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
        // can be scaled to other temperature ranges
        let resistance = (100.0 * raw_resistance as f32 * *self.reference_resistor_ratio.numer() as f32)
            / ((1 << 15) as f32 * *self.reference_resistor_ratio.denom() as f32);
        let temperature = callendar_van_dusen::resistance_to_temperature_r100(resistance);
        ThermodynamicTemperature::new::<degree_celsius>(temperature)
    }

    /// Read the current resistance from the device register and convert
    /// it to a temperature by using the internal Callendar-Van Dusen lookup table.
    pub async fn read_temperature(&mut self) -> Result<ThermodynamicTemperature, I::Error> {
        let raw_resistance_ratio = self
            .read_register::<registers::Resistance>()
            .await?
            .read_resistance_ratio();
        Ok(self.raw_resistance_ratio_to_temperature(raw_resistance_ratio))
    }
}
