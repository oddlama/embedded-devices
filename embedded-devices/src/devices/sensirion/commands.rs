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
    const COMMAND_SIZE: usize;
    /// The command id.
    const COMMAND_ID: u64;
    /// This reflects the execution time for this command in milliseconds, if specified in the
    /// datasheet.
    const EXECUTION_TIME_MS: u32;
    /// Whether this command can be executed during measurement
    const DURING_MEASUREMENT: bool;
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

define_executor!(SensirionSendCommandExecutor, SensirionSendCommand, ());
define_executor!(SensirionWriteCommandExecutor, SensirionWriteCommand, ());
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
        _input: impl AsRef<C::In>,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_SIZE..];
        bound_bus.interface.write(bound_bus.address, &header).await?;
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
        input: impl AsRef<C::In>,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_SIZE..];
        let mut array = Vec::<u8, 64>::new();
        array
            .extend_from_slice(header)
            .expect("Command id is too large for command buffer");

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let data = bytemuck::bytes_of(input.as_ref());
        for x in data.chunks(CHUNK_SIZE) {
            array
                .extend_from_slice(x)
                .expect("Command data too large for buffer. Raise an issue in embedded_interfaces.");
            array
                .push(crc.checksum(x))
                .expect("Command data too large for buffer. Raise an issue in embedded_interfaces.");
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
        _input: impl AsRef<C::In>,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        use bytemuck::Zeroable;
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_SIZE..];
        bound_bus.interface.write(bound_bus.address, &header).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;

        let mut array = Vec::<u8, 64>::new();
        array
            .resize_default(C::COMMAND_SIZE + C::COMMAND_SIZE / CHUNK_SIZE)
            .expect("Command response data is too large for buffer");

        bound_bus.interface.read(bound_bus.address, &mut array).await?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let mut out = C::Out::zeroed();
        let data = bytemuck::bytes_of_mut(&mut out);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[i..i + CHUNK_SIZE].copy_from_slice(value);

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
        input: impl AsRef<C::In>,
    ) -> Result<C::Out, TransportError<Self::Error, I::Error>>
    where
        D: hal::delay::DelayNs,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        use bytemuck::Zeroable;
        const CHUNK_SIZE: usize = 2;

        let header = &C::COMMAND_ID.to_be_bytes()[core::mem::size_of_val(&C::COMMAND_ID) - C::COMMAND_SIZE..];
        let mut array = Vec::<u8, 64>::new();
        array
            .extend_from_slice(header)
            .expect("Command id is too large for command buffer");

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let data = bytemuck::bytes_of(input.as_ref());
        for x in data.chunks(CHUNK_SIZE) {
            array
                .extend_from_slice(x)
                .expect("Command data too large for buffer. Raise an issue in embedded_interfaces.");
            array
                .push(crc.checksum(x))
                .expect("Command data too large for buffer. Raise an issue in embedded_interfaces.");
        }

        bound_bus.interface.write(bound_bus.address, &array).await?;
        delay.delay_ms(C::EXECUTION_TIME_MS).await;

        let mut array = Vec::<u8, 64>::new();
        array
            .resize_default(C::COMMAND_SIZE + C::COMMAND_SIZE / CHUNK_SIZE)
            .expect("Command response data is too large for buffer");

        bound_bus.interface.read(bound_bus.address, &mut array).await?;

        let crc = crc::Crc::<u8>::new(&crc::CRC_8_NRSC_5);
        let mut out = C::Out::zeroed();
        let data = bytemuck::bytes_of_mut(&mut out);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[i..i + CHUNK_SIZE].copy_from_slice(value);

            let calculated = crc.checksum(value);
            let expected = x[CHUNK_SIZE];
            if expected != calculated {
                return Err(TransportError::Codec(Crc8Error::CrcMismatch { calculated, expected }));
            }
        }

        Ok(out)
    }
}
