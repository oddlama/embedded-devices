use crate::{TransportError, commands::Command};

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
#[allow(clippy::type_complexity)]
pub trait Executor: crate::commands::r#Executor {
    /// Execute the given command through the given I2C interface
    async fn execute<D, I, A>(
        delay: &mut D,
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        input: <Self::Command as Command>::In,
    ) -> Result<<Self::Command as Command>::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy + core::fmt::Debug;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, CommandInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, A> crate::commands::CommandInterface for crate::i2c::I2cDevice<I, A>
where
    I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
    A: hal::i2c::AddressMode + Copy + core::fmt::Debug,
{
    type BusError = I::Error;

    /// Executes the given command through this interface
    #[inline]
    async fn execute<C, D>(
        &mut self,
        delay: &mut D,
        input: C::In,
    ) -> Result<C::Out, TransportError<C::ExecutorError, Self::BusError>>
    where
        D: hal::delay::DelayNs,
        C: Command,
    {
        <C::I2cExecutor as Executor>::execute::<D, I, A>(delay, &mut self.bound_bus, input).await
    }
}
