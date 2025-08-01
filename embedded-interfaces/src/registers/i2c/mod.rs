pub mod codecs;

use crate::TransportError;
use crate::registers::{ReadableRegister, Register, RegisterCodec, WritableRegister};

/// Represents a trait for I2C codecs. These are responsible to perform
/// writes and reads to registers, given the register address and
/// the raw data. Different devices can have different ways to encode
/// the desired address, address size, continuous-read mode and more.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
pub trait Codec: RegisterCodec {
    /// Read this register through the given I2C interface.
    async fn read_register<R, I, A>(
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
    ) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy;

    /// Write this register through the given I2C interface.
    async fn write_register<R, I, A>(
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, A> crate::registers::RegisterInterface for crate::i2c::I2cDevice<I, A>
where
    I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
    A: hal::i2c::AddressMode + Copy,
{
    type BusError = I::Error;

    /// Read this register from this spi device using the codec specified by the register.
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: ReadableRegister,
    {
        <R::I2cCodec as Codec>::read_register::<R, _, A>(&mut self.bound_bus).await
    }

    /// Write this register to this i2c device using the codec specified by the register.
    #[inline]
    async fn write_register<R>(
        &mut self,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: WritableRegister,
    {
        <R::I2cCodec as Codec>::write_register(&mut self.bound_bus, register).await
    }
}
