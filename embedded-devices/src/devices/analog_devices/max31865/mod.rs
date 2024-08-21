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
//!
//! // Create and initialize the device
//! let mut max31865 = MAX31865::new_spi(spi);
//! max31865.init().await.unwrap();
//!
//! // FIXME: TODO get temperature
//! // Read the current resistance ratio
//! let ratio = max31865
//!     .read_register::<Resistance>()
//!     .await?
//!     .read_resistance_ratio();
//! println!("Current raw resistance ratio: {:?}", ratio);
//! # Ok(())
//! # }
//! ```

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{spi::SpiDevice, RegisterInterface};

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
    pub fn new_spi(interface: I) -> Self {
        Self {
            interface: SpiDevice {
                interface,
                default_codec: MAX31865SpiCodec::default(),
            },
        }
    }
}

#[device_impl]
impl<I: RegisterInterface> MAX31865<I> {
    pub async fn init(&mut self) -> Result<(), I::Error> {
        Ok(())
    }
}
