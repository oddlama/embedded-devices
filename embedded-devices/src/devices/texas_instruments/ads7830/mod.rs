//! ADS7830 eight-bit, eight-channel ADC with an I²C interface.
//!
//! Supports eight single-ended inputs relative to COM and eight ordered pseudo-differential
//! selections. Differential results are unsigned: negative differences saturate at zero.
//! COM and negative inputs must satisfy the input limits in the
//! [datasheet](https://www.ti.com/lit/ds/symlink/ads7830.pdf).
//! Standard and fast I²C modes (100/400 kHz) are supported.
//!
//! The `maybe_async_cfg` macro generates `ADS7830Sync` with the `sync` feature and
//! `ADS7830Async` with the `async` feature; both can be enabled together.
//! Construction performs no I/O. Use `enable_internal_reference` to change the hardware
//! reference source. Reference changes and conversions return the underlying I²C error.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod example {
//! # fn read<I: embedded_hal::i2c::I2c>(i2c: I)
//! # -> Result<(), I::Error> {
//! use embedded_devices::devices::texas_instruments::ads7830::{
//!     ADS7830Sync, InputChannel, address::{Address, Pin},
//! };
//! use uom::si::{electric_potential::volt, f64::ElectricPotential};
//!
//! let mut adc = ADS7830Sync::new(
//!     i2c, Address::A0A1(Pin::Vcc, Pin::Vcc),
//! ).expect("valid address");
//! let voltage = adc.convert(InputChannel::Single0, Some(ElectricPotential::new::<volt>(3.3)))?;
//! let volts = voltage.get::<volt>();
//! let i2c = adc.destroy();
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async, internal reference)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod example {
//! # async fn read<I, D>(i2c: I, delay: &mut D)
//! # -> Result<(), I::Error>
//! # where I: embedded_hal_async::i2c::I2c, D: embedded_hal_async::delay::DelayNs {
//! use embedded_devices::devices::texas_instruments::ads7830::{
//!     ADS7830Async, InputChannel, address::{Address, Pin},
//! };
//!
//! let mut adc = ADS7830Async::new(
//!     i2c, Address::A0A1(Pin::Gnd, Pin::Gnd),
//! ).expect("valid address");
//! adc.enable_internal_reference(true).await?;
//! // Example duration; select and verify it for your reference circuit.
//! delay.delay_us(2_000).await;
//! let voltage = adc.convert(InputChannel::Diff01, None).await?;
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! Reference wiring must agree with the selected source. Do not externally drive REF when using
//! the internal reference. External references must be settled and no higher than actual VDD.
//! The internal reference uses nominal 2.5 V.
//! TI specifies typical settling times of 37 µs without a capacitor and 930 µs with 1 µF;
//! these are not guaranteed maxima. Wait for settling after enabling the internal reference,
//! including after a chip reset. Disable it before applying an external voltage to REF.
//! Pass `Some(reference_voltage)` to `convert` to use the reference voltage during that
//! conversion or a calibrated value. `None` uses nominal 2.5 V for either reference source;
//! this default only affects scaling and does not select the hardware reference.
//! Voltage conversion uses `sample * VREF / 256` and is not a calibrated measurement.

use uom::si::{electric_potential::volt, f64::ElectricPotential};

pub mod address;

/// Positive and negative analog inputs, named according to the chip pinout.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputChannel {
    /// CH0 relative to COM.
    Single0 = 0x80,
    /// CH1 relative to COM.
    Single1 = 0xc0,
    /// CH2 relative to COM.
    Single2 = 0x90,
    /// CH3 relative to COM.
    Single3 = 0xd0,
    /// CH4 relative to COM.
    Single4 = 0xa0,
    /// CH5 relative to COM.
    Single5 = 0xe0,
    /// CH6 relative to COM.
    Single6 = 0xb0,
    /// CH7 relative to COM.
    Single7 = 0xf0,
    /// CH0 positive, CH1 negative.
    Diff01 = 0x00,
    /// CH2 positive, CH3 negative.
    Diff23 = 0x10,
    /// CH4 positive, CH5 negative.
    Diff45 = 0x20,
    /// CH6 positive, CH7 negative.
    Diff67 = 0x30,
    /// CH1 positive, CH0 negative.
    Diff10 = 0x40,
    /// CH3 positive, CH2 negative.
    Diff32 = 0x50,
    /// CH5 positive, CH4 negative.
    Diff54 = 0x60,
    /// CH7 positive, CH6 negative.
    Diff76 = 0x70,
}

/// Invalid externally supplied constructor arguments.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ConfigError {
    /// Custom address exceeds the seven-bit range.
    #[error("invalid seven-bit address")]
    WrongAddress,
}

/// ADS7830 driver owning its I²C bus.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
pub struct ADS7830<I> {
    /// The interface to communicate with the device.
    interface: I,
    /// Seven-bit device address.
    address: u8,
    /// Whether conversion commands keep the internal reference enabled.
    internal_reference_enabled: bool,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: hal::i2c::I2c> ADS7830<I> {
    /// Creates a driver without accessing hardware, consuming the I²C bus.
    /// Conversions use an external reference until `enable_internal_reference(true)` succeeds.
    pub fn new(interface: I, address: impl Into<u8>) -> Result<Self, ConfigError> {
        let address = address.into();
        if address > 0x7f {
            return Err(ConfigError::WrongAddress);
        }
        Ok(Self {
            interface,
            address,
            internal_reference_enabled: false,
        })
    }

    /// Consumes the driver and recovers its bus without accessing hardware.
    pub fn destroy(self) -> I {
        self.interface
    }

    /// Enables or disables the internal 2.5 V reference, preserving the setting for conversions.
    /// When enabling, REF must not be externally driven; wait for settling before converting.
    /// After a failed or cancelled write, reapply the intended setting before converting.
    pub async fn enable_internal_reference(&mut self, enabled: bool) -> Result<(), I::Error> {
        let command = if enabled { 0x08 } else { 0x00 };
        self.interface.write(self.address, &[command]).await?;
        self.internal_reference_enabled = enabled;
        Ok(())
    }

    /// Performs a fresh conversion and returns an unsigned eight-bit sample.
    pub async fn convert_raw(&mut self, channel: InputChannel) -> Result<u8, I::Error> {
        let power = if self.internal_reference_enabled { 0x0c } else { 0x04 };
        let command = channel as u8 | power;
        let mut sample = [0];
        // Combined transaction preserves the datasheet's repeated START.
        self.interface.write_read(self.address, &[command], &mut sample).await?;
        Ok(sample[0])
    }

    /// Performs a fresh conversion and returns voltage relative to the negative input.
    /// `None` scales using nominal 2.5 V; it does not change the hardware reference source.
    pub async fn convert(
        &mut self,
        channel: InputChannel,
        reference_voltage: Option<ElectricPotential>,
    ) -> Result<ElectricPotential, I::Error> {
        let sample = self.convert_raw(channel).await?;
        let reference_voltage = reference_voltage.unwrap_or_else(|| ElectricPotential::new::<volt>(2.5));
        Ok(reference_voltage * (f64::from(sample) / 256.0))
    }
}
