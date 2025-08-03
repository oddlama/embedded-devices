use core::marker::PhantomData;

use crate::TransportError;
use crate::registers::{ReadableRegister, Register, RegisterCodec, WritableRegister};

/// A codec that represents absense of a codec. This is only used as a placeholder in register
/// definitions to specify that the associated interface is not supported.
pub struct UnsupportedCodec<E> {
    _marker: PhantomData<E>,
}

impl<E: core::fmt::Debug> RegisterCodec for UnsupportedCodec<E> {
    type Error = E;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<E: core::fmt::Debug> crate::registers::i2c::Codec for UnsupportedCodec<E> {
    #[inline]
    async fn read_register<R, I, A>(
        _bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
    ) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        Err(TransportError::Unexpected("unsupported interface"))
    }

    #[inline]
    async fn write_register<R, I, A>(
        _bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        _register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        Err(TransportError::Unexpected("unsupported interface"))
    }
}
