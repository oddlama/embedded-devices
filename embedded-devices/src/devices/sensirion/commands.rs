use core::marker::PhantomData;

use embedded_interfaces::{TransportError, commands::Command, define_executor};
use heapless::Vec;

/// An error representing CRC errors.
#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum Crc8Error {
    #[error("the calculated crc checksum {calculated:#02x} did not match the expected value {expected:#02x}")]
    CrcMismatch { calculated: u8, expected: u8 },
}

/// Metadata associated to any sensirion specific command
pub trait SensirionCommand {
    /// Length of the command id in bytes
    const COMMAND_ID_LEN: usize;
    /// The command id.
    const COMMAND_ID: u64;
    /// This reflects the execution time for this command in milliseconds, if specified in the
    /// datasheet.
    const EXECUTION_TIME_MS: u32;
}

/// This trait represents a send-command as specified by Sensirion. It consists of sending a
/// command id followed by waiting for a pre-defined amount of time.
pub trait SensirionSendCommand:
    SensirionCommand + Command<I2cExecutor = SensirionSendCommandExecutor<Self>, In = (), Out = ()>
{
}

/// This trait represents a write-command as specified by Sensirion. It consists of sending a
/// command id plus some data followed by waiting for a pre-defined amount of time.
pub trait SensirionWriteCommand:
    SensirionCommand + Command<I2cExecutor = SensirionWriteCommandExecutor<Self>, Out = ()>
{
}

/// This trait represents a read-command as specified by Sensirion. It consists of sending a
/// command id followed by a wait period and a final read.
pub trait SensirionReadCommand:
    SensirionCommand + Command<I2cExecutor = SensirionReadCommandExecutor<Self>, In = ()>
{
}

/// This trait represents a write-read-command (also called fetch-command) as specified by
/// Sensirion. It consists of sending a command id plus some data followed by a wait period and a
/// final read.
pub trait SensirionWriteReadCommand:
    SensirionCommand + Command<I2cExecutor = SensirionWriteReadCommandExecutor<Self>>
{
}

// Crc8Error is technically not needed for send and write, but we use it anyway to make
// the interface more usable. Otherwise we'd need a ton of fallible error conversions.
// FIXME: try to rework this once the never type is stable and we can express that this can't error, ever.
define_executor!(SensirionSendCommandExecutor, SensirionSendCommand, Crc8Error);
define_executor!(SensirionWriteCommandExecutor, SensirionWriteCommand, Crc8Error);
define_executor!(SensirionReadCommandExecutor, SensirionReadCommand, Crc8Error);
define_executor!(SensirionWriteReadCommandExecutor, SensirionWriteReadCommand, Crc8Error);

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<C: SensirionSendCommand> embedded_interfaces::commands::i2c::Executor for SensirionSendCommandExecutor<C> {
    async fn execute<D, I, A>(
        delay: &mut D,
        bound_bus: &mut embedded_interfaces::i2c::I2cBoundBus<I, A>,
        _input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_ID_LEN..];
        bound_bus.interface.write(bound_bus.address, header).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;
        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<C: SensirionWriteCommand> embedded_interfaces::commands::i2c::Executor for SensirionWriteCommandExecutor<C> {
    async fn execute<D, I, A>(
        delay: &mut D,
        bound_bus: &mut embedded_interfaces::i2c::I2cBoundBus<I, A>,
        input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_ID_LEN..];
        let mut array = Vec::<u8, 64>::new();
        array
            .extend_from_slice(header)
            .map_err(|_| TransportError::Unexpected("command id: too large for command buffer"))?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let data = bytemuck::bytes_of(&input);
        for x in data.chunks(CHUNK_SIZE) {
            array
                .extend_from_slice(x)
                .map_err(|_| TransportError::Unexpected("command data: too large for buffer"))?;
            array
                .push(crc.checksum(x))
                .map_err(|_| TransportError::Unexpected("command data: too large for buffer"))?;
        }

        bound_bus.interface.write(bound_bus.address, &array).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;
        Ok(())
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<C: SensirionReadCommand> embedded_interfaces::commands::i2c::Executor for SensirionReadCommandExecutor<C> {
    async fn execute<D, I, A>(
        delay: &mut D,
        bound_bus: &mut embedded_interfaces::i2c::I2cBoundBus<I, A>,
        _input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        use bytemuck::Zeroable;
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_ID_LEN..];
        bound_bus.interface.write(bound_bus.address, header).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;

        let mut array = Vec::<u8, 64>::new();
        let mut out = C::Out::zeroed();
        let data = bytemuck::bytes_of_mut(&mut out);
        array
            .resize_default(data.len() + data.len() / CHUNK_SIZE)
            .map_err(|_| TransportError::Unexpected("command response data: too large for buffer"))?;

        bound_bus.interface.read(bound_bus.address, &mut array).await?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[(i * CHUNK_SIZE)..((i + 1) * CHUNK_SIZE)].copy_from_slice(value);

            let calculated = crc.checksum(value);
            let expected = x[CHUNK_SIZE];
            if expected != calculated {
                return Err(TransportError::Codec(Crc8Error::CrcMismatch { calculated, expected }));
            }
        }

        Ok(out)
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Executor, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<C: SensirionWriteReadCommand> embedded_interfaces::commands::i2c::Executor
    for SensirionWriteReadCommandExecutor<C>
{
    async fn execute<D, I, A>(
        delay: &mut D,
        bound_bus: &mut embedded_interfaces::i2c::I2cBoundBus<I, A>,
        input: C::In,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        use bytemuck::Zeroable;
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_ID_LEN..];
        let mut array = Vec::<u8, 64>::new();
        array
            .extend_from_slice(header)
            .map_err(|_| TransportError::Unexpected("command id: too large for command buffer"))?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let data = bytemuck::bytes_of(&input);
        for x in data.chunks(CHUNK_SIZE) {
            array
                .extend_from_slice(x)
                .map_err(|_| TransportError::Unexpected("command data: too large for buffer"))?;
            array
                .push(crc.checksum(x))
                .map_err(|_| TransportError::Unexpected("command data: too large for buffer"))?;
        }

        bound_bus.interface.write(bound_bus.address, &array).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;

        let mut array = Vec::<u8, 64>::new();
        let mut out = C::Out::zeroed();
        let data = bytemuck::bytes_of_mut(&mut out);
        array
            .resize_default(data.len() + data.len() / CHUNK_SIZE)
            .map_err(|_| TransportError::Unexpected("command response data: too large for buffer"))?;

        bound_bus.interface.read(bound_bus.address, &mut array).await?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[(i * CHUNK_SIZE)..((i + 1) * CHUNK_SIZE)].copy_from_slice(value);

            let calculated = crc.checksum(value);
            let expected = x[CHUNK_SIZE];
            if expected != calculated {
                return Err(TransportError::Codec(Crc8Error::CrcMismatch { calculated, expected }));
            }
        }

        Ok(out)
    }
}

#[allow(unused)]
macro_rules! define_send_command {
    ($name:ident, id_len=$id_len:literal, id=$id:expr, time=$time_ms:expr) => {
        impl embedded_interfaces::commands::Command for $name {
            type In = ();
            type Out = ();
            type ExecutorError = crate::devices::sensirion::commands::Crc8Error;
            type SpiExecutor = embedded_interfaces::commands::unsupported_executor::UnsupportedExecutor<
                crate::devices::sensirion::commands::Crc8Error,
                Self,
            >;
            type I2cExecutor = crate::devices::sensirion::commands::SensirionSendCommandExecutor<Self>;
        }
        impl crate::devices::sensirion::commands::SensirionCommand for $name {
            const COMMAND_ID_LEN: usize = $id_len;
            const COMMAND_ID: u64 = $id;
            const EXECUTION_TIME_MS: u32 = $time_ms;
        }
        impl crate::devices::sensirion::commands::SensirionSendCommand for $name {}
    };
}
#[allow(unused)]
pub(crate) use define_send_command;

#[allow(unused)]
macro_rules! define_write_command {
    ($name:ident, id_len=$id_len:literal, id=$id:expr, time=$time_ms:expr, in=$in:ty) => {
        impl embedded_interfaces::commands::Command for $name {
            type In = $in;
            type Out = ();
            type ExecutorError = crate::devices::sensirion::commands::Crc8Error;
            type SpiExecutor = embedded_interfaces::commands::unsupported_executor::UnsupportedExecutor<
                crate::devices::sensirion::commands::Crc8Error,
                Self,
            >;
            type I2cExecutor = crate::devices::sensirion::commands::SensirionWriteCommandExecutor<Self>;
        }
        impl crate::devices::sensirion::commands::SensirionCommand for $name {
            const COMMAND_ID_LEN: usize = $id_len;
            const COMMAND_ID: u64 = $id;
            const EXECUTION_TIME_MS: u32 = $time_ms;
        }
        impl crate::devices::sensirion::commands::SensirionWriteCommand for $name {}
    };
}
#[allow(unused)]
pub(crate) use define_write_command;

#[allow(unused)]
macro_rules! define_read_command {
    ($name:ident, id_len=$id_len:literal, id=$id:expr, time=$time_ms:expr, out=$out:ty) => {
        impl embedded_interfaces::commands::Command for $name {
            type In = ();
            type Out = $out;
            type ExecutorError = crate::devices::sensirion::commands::Crc8Error;
            type SpiExecutor = embedded_interfaces::commands::unsupported_executor::UnsupportedExecutor<
                crate::devices::sensirion::commands::Crc8Error,
                Self,
            >;
            type I2cExecutor = crate::devices::sensirion::commands::SensirionReadCommandExecutor<Self>;
        }
        impl crate::devices::sensirion::commands::SensirionCommand for $name {
            const COMMAND_ID_LEN: usize = $id_len;
            const COMMAND_ID: u64 = $id;
            const EXECUTION_TIME_MS: u32 = $time_ms;
        }
        impl crate::devices::sensirion::commands::SensirionReadCommand for $name {}
    };
}
#[allow(unused)]
pub(crate) use define_read_command;

#[allow(unused)]
macro_rules! define_write_read_command {
    ($name:ident, id_len=$id_len:literal, id=$id:expr, time=$time_ms:expr, in=$in:ty, out=$out:ty) => {
        impl embedded_interfaces::commands::Command for $name {
            type In = $in;
            type Out = $out;
            type ExecutorError = crate::devices::sensirion::commands::Crc8Error;
            type SpiExecutor = embedded_interfaces::commands::unsupported_executor::UnsupportedExecutor<
                crate::devices::sensirion::commands::Crc8Error,
                Self,
            >;
            type I2cExecutor = crate::devices::sensirion::commands::SensirionWriteReadCommandExecutor<Self>;
        }
        impl crate::devices::sensirion::commands::SensirionCommand for $name {
            const COMMAND_ID_LEN: usize = $id_len;
            const COMMAND_ID: u64 = $id;
            const EXECUTION_TIME_MS: u32 = $time_ms;
        }
        impl crate::devices::sensirion::commands::SensirionWriteReadCommand for $name {}
    };
}
#[allow(unused)]
pub(crate) use define_write_read_command;

#[allow(unused)]
macro_rules! define_sensirion_commands {
    // Main entry point - parse the marker section first
    (
        id_len $id_len:literal;
        marker [
            $(($feature:literal, $trait_path:path)),* $(,)?
        ];
        $($rest:tt)*
    ) => {
        define_sensirion_commands! {
            @id_len $id_len;
            @markers [$(($feature, $trait_path)),*];
            @parse [$($rest)*]
        }
    };

    // Parse individual command definitions
    (
        @id_len $id_len:literal;
        @markers [$($markers:tt)*];
        @parse [
            $(#[doc = $doc:literal])*
            send $id:literal time_ms=$time:literal $name:ident();
            $($rest:tt)*
        ]
    ) => {
        define_sensirion_commands! { @emit_send { [$( #[doc = $doc] )*], $name, $id_len, $id, $time } }
        define_sensirion_commands! { @emit_impls { $name, [$($markers)*] } }
        define_sensirion_commands! {
            @id_len $id_len;
            @markers [$($markers)*];
            @parse [$($rest)*]
        }
    };

    (
        @id_len $id_len:literal;
        @markers [$($markers:tt)*];
        @parse [
            $(#[doc = $doc:literal])*
            read $id:literal time_ms=$time:literal $name:ident() -> $out:ty;
            $($rest:tt)*
        ]
    ) => {
        define_sensirion_commands! { @emit_read { [$( #[doc = $doc] )*], $name, $id_len, $id, $time, $out } }
        define_sensirion_commands! { @emit_impls { $name, [$($markers)*] } }
        define_sensirion_commands! {
            @id_len $id_len;
            @markers [$($markers)*];
            @parse [$($rest)*]
        }
    };

    (
        @id_len $id_len:literal;
        @markers [$($markers:tt)*];
        @parse [
            $(#[doc = $doc:literal])*
            write $id:literal time_ms=$time:literal $name:ident($in:ty);
            $($rest:tt)*
        ]
    ) => {
        define_sensirion_commands! { @emit_write { [$( #[doc = $doc] )*], $name, $id_len, $id, $time, $in } }
        define_sensirion_commands! { @emit_impls { $name, [$($markers)*] } }
        define_sensirion_commands! {
            @id_len $id_len;
            @markers [$($markers)*];
            @parse [$($rest)*]
        }
    };

    (
        @id_len $id_len:literal;
        @markers [$($markers:tt)*];
        @parse [
            $(#[doc = $doc:literal])*
            write_read $id:literal time_ms=$time:literal $name:ident($in:ty) -> $out:ty;
            $($rest:tt)*
        ]
    ) => {
        define_sensirion_commands! { @emit_write_read { [$( #[doc = $doc] )*], $name, $id_len, $id, $time, $in, $out } }
        define_sensirion_commands! { @emit_impls { $name, [$($markers)*] } }
        define_sensirion_commands! {
            @id_len $id_len;
            @markers [$($markers)*];
            @parse [$($rest)*]
        }
    };

    // Base case - no more commands to parse
    (
        @id_len $id_len:literal;
        @markers [$($markers:tt)*];
        @parse []
    ) => {};

    // Emit the actual command definitions
    (@emit_send { [$( $doc:tt )*], $name:ident, $id_len:literal, $id:literal, $time:literal }) => {
        $( $doc )*
        #[doc = ""]
        #[doc = concat!("Execution time (ms): `", stringify!($time), "`")]
        pub struct $name {}

        crate::devices::sensirion::commands::define_send_command!($name, id_len=$id_len, id=$id, time=$time);
    };

    (@emit_read { [$( $doc:tt )*], $name:ident, $id_len:literal, $id:literal, $time:literal, $out:ty }) => {
        $( $doc )*
        #[doc = ""]
        #[doc = concat!("Out: [`", stringify!($out), "`]")]
        #[doc = ""]
        #[doc = concat!("Execution time (ms): `", stringify!($time), "`")]
        pub struct $name {}

        crate::devices::sensirion::commands::define_read_command!($name, id_len=$id_len, id=$id, time=$time, out=$out);
    };

    (@emit_write { [$( $doc:tt )*], $name:ident, $id_len:literal, $id:literal, $time:literal, $in:ty }) => {
        $( $doc )*
        #[doc = ""]
        #[doc = concat!("In: [`", stringify!($in), "`]")]
        #[doc = ""]
        #[doc = concat!("Execution time (ms): `", stringify!($time), "`")]
        pub struct $name {}

        crate::devices::sensirion::commands::define_write_command!($name, id_len=$id_len, id=$id, time=$time, in=$in);
    };

    (@emit_write_read { [$( $doc:tt )*], $name:ident, $id_len:literal, $id:literal, $time:literal, $in:ty, $out:ty }) => {
        $( $doc )*
        #[doc = ""]
        #[doc = concat!("In: [`", stringify!($in), "`]")]
        #[doc = ""]
        #[doc = concat!("Out: [`", stringify!($out), "`]")]
        #[doc = ""]
        #[doc = concat!("Execution time (ms): `", stringify!($time), "`")]
        pub struct $name {}

        crate::devices::sensirion::commands::define_write_read_command!($name, id_len=$id_len, id=$id, time=$time, in=$in, out=$out);
    };

    // Emit trait implementations for all markers
    (
        @emit_impls { $name:ident, [$(($feature:literal, $trait_path:path)),*] }
    ) => {
        $(
            #[cfg(feature = $feature)]
            impl $trait_path for $name {}
        )*
    };
}
#[allow(unused)]
pub(crate) use define_sensirion_commands;
