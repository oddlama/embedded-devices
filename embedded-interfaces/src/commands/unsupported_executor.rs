use crate::TransportError;

use super::{Command, Executor, i2c, spi};

/// This dummy executor accepts any command and does nothing when used. If this executor is used,
/// it means that the associated interface is not supported.
pub struct UnsupportedExecutor<E, C: Command + ?Sized> {
    _marker: core::marker::PhantomData<(E, C)>,
}

impl<E, C: Command> Executor for UnsupportedExecutor<E, C> {
    type Error = E;
    type Command = C;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<E, C: Command> i2c::Executor for UnsupportedExecutor<E, C> {
    async fn execute<D, I, A>(
        _delay: &mut D,
        _bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        _input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        Err(TransportError::Unexpected("unsupported interface"))
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<E, C: Command> spi::Executor for UnsupportedExecutor<E, C> {
    async fn execute<D, I>(
        _delay: &mut D,
        _interface: &mut I,
        _input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::spi::r#SpiDevice,
    {
        Err(TransportError::Unexpected("unsupported interface"))
    }
}
