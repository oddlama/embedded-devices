//! # MCP3204
//!
//! The Microchip Technology Inc. MCP3204 device is a successive approximation 12-bit Analog-
//! to-Digital (A/D) Converter with on-board sample and hold circuitry. The MCP3204 is programmable
//! to provide two pseudo-differential input pairs or four single-ended inputs.
//!
//! Differential Nonlinearity (DNL) is specified at ±1 LSB, while Integral Nonlinearity (INL) is
//! offered in ±1 LSB (MCP3204-B) and ±2 LSB (MCP3204-C) versions. Communication with the devices
//! is accomplished using a simple serial interface compatible with the SPI protocol. The devices
//! are capable of conversion rates of up to 100 ksps. The MCP3204 devices operate over a
//! broad voltage range (2.7V - 5.5V). Low current design permits operation with typical standby
//! and active currents of only 500 nA and 320 μA, respectively.
//!
//! ## Usage (sync)
//!
//! ```rust
//! # #[cfg(feature = "sync")] mod test {
//! # fn test<I>(mut spi: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal::spi::SpiDevice,
//! # {
//! use embedded_devices::devices::microchip::mcp3204::{MCP3204Sync, InputChannel};
//! use uom::si::electric_potential::{volt, millivolt};
//! use uom::si::f64::ElectricPotential;
//!
//! // 2.5V reference
//! let mut mcp3204 = MCP3204Sync::new_spi(spi, ElectricPotential::new::<volt>(2.5));
//!
//! let value = mcp3204.convert(InputChannel::Single0)?;
//! let voltage = value.get::<millivolt>();
//! println!("V_in at channel 0: {:?}mV", value);
//! # Ok(())
//! # }
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust
//! # #[cfg(feature = "async")] mod test {
//! # async fn test<I>(mut spi: I) -> Result<(), I::Error>
//! # where
//! #   I: embedded_hal_async::spi::SpiDevice,
//! # {
//! use embedded_devices::devices::microchip::mcp3204::{MCP3204Async, InputChannel};
//! use uom::si::electric_potential::{volt, millivolt};
//! use uom::si::f64::ElectricPotential;
//!
//! // 2.5V reference
//! let mut mcp3204 = MCP3204Async::new_spi(spi, ElectricPotential::new::<volt>(2.5));
//!
//! let value = mcp3204.convert(InputChannel::Single0).await?;
//! let voltage = value.get::<millivolt>();
//! println!("V_in at channel 0: {:?}mV", value);
//! # Ok(())
//! # }
//! # }
//! ```

use uom::si::{electric_potential::volt, f64::ElectricPotential};

/// The ADC input channel
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
pub enum InputChannel {
    /// Single channel 0
    Single0 = 0b1000,
    /// Single channel 1
    Single1 = 0b1001,
    /// Single channel 2
    Single2 = 0b1010,
    /// Single channel 3
    Single3 = 0b1011,
    /// Pseudo-differential channel (IN+ = CH0, IN- = CH1)
    Diff01 = 0b0000,
    /// Pseudo-differential channel (IN+ = CH1, IN- = CH0)
    Diff10 = 0b0001,
    /// Pseudo-differential channel (IN+ = CH2, IN- = CH3)
    Diff23 = 0b0010,
    /// Pseudo-differential channel (IN+ = CH3, IN- = CH2)
    Diff32 = 0b0011,
}

/// The MCP3204 is a 12-bit ADC with on-board sample and hold circuitry. It is programmable
/// to provide two pseudo-differential input pairs or four single-ended inputs.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
pub struct MCP3204<I> {
    /// The interface to communicate with the device
    interface: I,
    /// The reference voltage
    reference_voltage: ElectricPotential,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> MCP3204<I>
where
    I: hal::spi::r#SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The device supports SPI modes 0 and 3.
    #[inline]
    pub fn new_spi(interface: I, reference_voltage: ElectricPotential) -> Self {
        Self {
            interface,
            reference_voltage,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: hal::spi::r#SpiDevice> MCP3204<I> {
    /// Performs a conversion of the given channel and returns the raw value
    pub async fn convert_raw(&mut self, channel: InputChannel) -> Result<u16, I::Error> {
        // Do bitwise-or with the required start bit. We also pad one leading zero so the readout
        // has better alignment.
        let command: u8 = 0b01000000 | (channel as u8) << 2;
        let mut data = [command, 0, 0];
        self.interface.transfer_in_place(&mut data).await?;
        Ok((data[1] as u16) << 4 | (data[2] as u16) >> 4)
    }

    /// Performs a conversion of the given channel and returns the value in volts
    pub async fn convert(&mut self, channel: InputChannel) -> Result<ElectricPotential, I::Error> {
        let raw_value = self.convert_raw(channel).await?;
        let v_ref = self.reference_voltage.get::<volt>();
        let value = ElectricPotential::new::<volt>(raw_value as f64 * v_ref / 4096.0);
        Ok(value)
    }
}
