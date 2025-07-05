pub mod codecs;

use crate::{ReadableRegister, Register, RegisterCodec, TransportError, WritableRegister};

/// Represents a trait for SPI codecs. These are responsible to perform
/// writes and reads to registers, given the register address and
/// the raw data. Different devices can have different ways to encode
/// the desired address, R/W bit location, continuous-read mode and more.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
pub trait Codec: RegisterCodec {
    /// Read this register from the given SPI interface/device.
    async fn read_register<R, I>(interface: &mut I) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::spi::r#SpiDevice;

    /// Write this register to the given SPI interface/device.
    async fn write_register<R, I>(
        interface: &mut I,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::spi::r#SpiDevice;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async")
)]
/// This represents an SPI device on an SPI bus.
pub struct SpiDevice<I>
where
    I: hal::spi::r#SpiDevice,
{
    /// Spi interface
    pub interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> SpiDevice<I>
where
    I: hal::spi::r#SpiDevice,
{
    /// Create a new I2cDevice from an interface
    pub fn new(interface: I) -> Self {
        Self { interface }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> crate::RegisterInterface for SpiDevice<I>
where
    I: hal::spi::r#SpiDevice,
{
    type BusError = I::Error;

    /// Read this register from this spi device using the codec specified by the register.
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: ReadableRegister,
    {
        <R::SpiCodec as Codec>::read_register::<R, _>(&mut self.interface).await
    }

    /// Write this register to this spi device using the codec specified by the register.
    #[inline]
    async fn write_register<R>(
        &mut self,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<<R as Register>::CodecError, Self::BusError>>
    where
        R: WritableRegister,
    {
        <R::SpiCodec as Codec>::write_register(&mut self.interface, register).await
    }
}
