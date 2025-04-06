use crate::{ReadableRegister, WritableRegister};

/// A codec that represents absense of a codec. This has two main usecases:
///
/// Firstly, if this is used as the default codec for a device, it essentially
/// requires any associated register to explicitly specify a codec. Otherwise
/// accessing that register via the [`RegisterInterfaceSync`](crate::RegisterInterfaceSync)
/// or [`RegisterInterfaceAsync`](crate::RegisterInterfaceAsync) trait will cause a panic.
///
/// Secondly, specifying this codec as the default for a register will cause
/// any reads or writes to that register via the [`RegisterInterfaceSync`](crate::RegisterInterfaceSync)
/// or [`RegisterInterfaceAsync`](crate::RegisterInterfaceAsync) traits to be performed
/// through the default codec of the device.
#[derive(Default)]
pub struct NoCodec {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl crate::i2c::Codec for NoCodec {
    #[inline]
    async fn read_register<R, I, A>(_bound_bus: &mut crate::i2c::I2cBoundBus<I, A>) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        panic!("i2c::codecs::NoCodec cannot be used at runtime! Please specify a real codec to access this register.");
    }

    #[inline]
    async fn write_register<R, I, A>(
        _bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        _register: impl AsRef<R>,
    ) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        panic!("i2c::codecs::NoCodec cannot be used at runtime! Please specify a real codec to access this register.");
    }
}
