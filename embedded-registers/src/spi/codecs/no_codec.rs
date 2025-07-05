use core::marker::PhantomData;

use crate::{ReadableRegister, Register, RegisterCodec, TransportError, WritableRegister};

/// A codec that represents absense of a codec. This is only used as a placeholder in register
/// definitions to specify that the associated interface is not supported.
pub struct NoCodec<E: 'static> {
    _marker: PhantomData<E>,
}

impl<E: 'static> RegisterCodec for NoCodec<E> {
    type Error = E;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<E: 'static> crate::spi::Codec for NoCodec<E> {
    #[inline]
    async fn read_register<R, I>(_interface: &mut I) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::spi::r#SpiDevice,
    {
        panic!("spi::codecs::NoCodec cannot be used at runtime! Please specify a real codec to access this register.");
    }

    #[inline]
    async fn write_register<R, I>(
        _interface: &mut I,
        _register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::spi::r#SpiDevice,
    {
        panic!("spi::codecs::NoCodec cannot be used at runtime! Please specify a real codec to access this register.");
    }
}
