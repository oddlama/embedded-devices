use crate::TransportError;

pub mod i2c;
pub mod spi;

/// A base trait for all register codecs
pub trait RegisterCodec {
    /// An error type for codec specific errors
    type Error;
}

/// The basis trait for all registers. A register is a type that maps to a specific register on an
/// embedded device and should own the raw data required for this register.
///
/// Additionally, a register knows the virtual address ossociated to the embedded device, and a
/// bitfield representation of the data content.
pub trait Register: Default + Clone + bytemuck::Pod {
    /// The size of the register in bytes
    const REGISTER_SIZE: usize;
    /// The virtual address of this register
    const ADDRESS: u64;

    /// The associated unpacked type
    type Unpacked;
    /// A common error type which can represent all associated codec errors
    type CodecError;
    /// The SPI codec that should be used for this register. If the device doesn't support SPI
    /// communication, this can be ignored.
    #[cfg(all(feature = "sync", not(feature = "async")))]
    type SpiCodec: spi::CodecSync<Error = Self::CodecError>;
    #[cfg(all(not(feature = "sync"), feature = "async"))]
    type SpiCodec: spi::CodecAsync<Error = Self::CodecError>;
    #[cfg(all(feature = "sync", feature = "async"))]
    type SpiCodec: spi::CodecSync<Error = Self::CodecError> + spi::CodecAsync<Error = Self::CodecError>;
    /// The I2C codec that should be used for this register. If the device doesn't support I2C
    /// communication, this can be ignored.
    #[cfg(all(feature = "sync", not(feature = "async")))]
    type I2cCodec: i2c::CodecSync<Error = Self::CodecError>;
    #[cfg(all(not(feature = "sync"), feature = "async"))]
    type I2cCodec: i2c::CodecAsync<Error = Self::CodecError>;
    #[cfg(all(feature = "sync", feature = "async"))]
    type I2cCodec: i2c::CodecSync<Error = Self::CodecError> + i2c::CodecAsync<Error = Self::CodecError>;
}

/// This trait is a marker trait implemented by any register that can be read from a device.
pub trait ReadableRegister: Register {}

/// This trait is a marker trait implemented by any register that can be written to a device.
pub trait WritableRegister: Register {}

/// A trait that is implemented by any bus interface and allows devices with registers to share
/// register read/write implementations independent of the actual interface in use.
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
#[allow(async_fn_in_trait)]
pub trait RegisterInterface {
    /// A type representing errors on the underlying bus
    type BusError;

    /// Reads the given register through this interface
    async fn read_register<R>(&mut self) -> Result<R, TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: ReadableRegister;

    /// Writes the given register through this interface
    async fn write_register<R>(
        &mut self,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: WritableRegister;
}
