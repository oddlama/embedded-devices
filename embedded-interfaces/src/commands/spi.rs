use crate::{TransportError, commands::Command};

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
pub trait Executor: crate::commands::r#Executor {
    /// Execute the given command through the given SPI interface
    async fn execute<D, I>(
        delay: &mut D,
        interface: &mut I,
        input: <Self::Command as Command>::In,
    ) -> Result<<Self::Command as Command>::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::spi::r#SpiDevice;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, CommandInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> crate::commands::CommandInterface for crate::spi::SpiDevice<I>
where
    I: hal::spi::r#SpiDevice,
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
        <C::SpiExecutor as Executor>::execute::<D, I>(delay, &mut self.interface, input).await
    }
}
