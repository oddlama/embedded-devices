use crate::TransportError;

pub mod i2c;
pub mod spi;
pub mod unsupported_executor;

/// A base trait for all executors
pub trait Executor {
    /// The command type used by this executor
    type Command: Command;
    /// An error type for executor specific errors
    type Error: core::fmt::Debug;
}

/// The basis trait for all commands. A command represents one or more transactions that belong
/// together and have to occur in a specific order and/or with a specific timing.
///
/// Commands can both have input and output data.
pub trait Command {
    /// The input data type
    type In: bytemuck::Pod;
    /// The output data type
    type Out: bytemuck::Pod + bytemuck::Zeroable;

    /// A common error type which can represent all associated executor errors
    type ExecutorError: core::fmt::Debug;
    /// The SPI executor that should be used for this register. If the device doesn't support SPI
    /// communication, this can be ignored.
    #[cfg(all(feature = "sync", not(feature = "async")))]
    type SpiExecutor: spi::ExecutorSync<Command = Self, Error = Self::ExecutorError>;
    #[cfg(all(not(feature = "sync"), feature = "async"))]
    type SpiExecutor: spi::ExecutorAsync<Command = Self, Error = Self::ExecutorError>;
    #[cfg(all(feature = "sync", feature = "async"))]
    type SpiExecutor: spi::ExecutorSync<Command = Self, Error = Self::ExecutorError>
        + spi::ExecutorAsync<Command = Self, Error = Self::ExecutorError>;
    /// The I2C executor that should be used for this register. If the device doesn't support I2C
    /// communication, this can be ignored.
    #[cfg(all(feature = "sync", not(feature = "async")))]
    type I2cExecutor: i2c::ExecutorSync<Command = Self, Error = Self::ExecutorError>;
    #[cfg(all(not(feature = "sync"), feature = "async"))]
    type I2cExecutor: i2c::ExecutorAsync<Command = Self, Error = Self::ExecutorError>;
    #[cfg(all(feature = "sync", feature = "async"))]
    type I2cExecutor: i2c::ExecutorSync<Command = Self, Error = Self::ExecutorError>
        + i2c::ExecutorAsync<Command = Self, Error = Self::ExecutorError>;
}

/// A trait that is implemented by any bus interface and allows devices with commands to share
/// commands executor implementations independent of the actual interface in use.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
pub trait CommandInterface {
    /// A type representing errors on the underlying bus
    type BusError: core::fmt::Debug;

    /// Executes the given command through this interface
    async fn execute<C, D>(
        &mut self,
        delay: &mut D,
        input: C::In,
    ) -> Result<C::Out, TransportError<C::ExecutorError, Self::BusError>>
    where
        D: hal::delay::DelayNs,
        C: Command;
}

#[macro_export]
macro_rules! define_executor {
    ($executor:ident, $command_trait:ident, $error_type:ty) => {
        #[doc = "The executor for any [`"]
        #[doc = stringify!($command_trait)]
        #[doc = "`]"]
        pub struct $executor<C: $command_trait + ?Sized> {
            _marker: PhantomData<C>,
        }

        impl<C: $command_trait> embedded_interfaces::commands::Executor for $executor<C> {
            type Error = $error_type;
            type Command = C;
        }
    };
}
