pub mod codecs;

use core::{any::TypeId, marker::PhantomData};

use codecs::NoCodec;

use crate::{ReadableRegister, WritableRegister};

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
pub trait Codec: 'static {
    /// Read this register from the given SPI interface/device.
    async fn read_register<R, I>(interface: &mut I) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::spi::r#SpiDevice;

    /// Write this register to the given SPI interface/device.
    async fn write_register<R, I>(interface: &mut I, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::spi::r#SpiDevice;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async")
)]
/// This represents an SPI device on an SPI bus.
pub struct SpiDevice<I, C>
where
    I: hal::spi::r#SpiDevice,
    C: Codec,
{
    /// Spi interface
    pub interface: I,
    /// The default codec used to interface with registers that don't explicitly specify a codec
    /// themselves. Usually this is a simple codec designating a bit for R/W and a bit-range
    /// for the register address. See implementors of the Codec trait for more information on
    /// available codecs.
    default_codec: PhantomData<C>,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, C> SpiDevice<I, C>
where
    I: hal::spi::r#SpiDevice,
    C: Codec,
{
    /// Create a new I2cDevice from an interface while specifying the default codec.
    pub fn new(interface: I) -> Self {
        Self {
            interface,
            default_codec: Default::default(),
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, C> crate::RegisterInterface for SpiDevice<I, C>
where
    I: hal::spi::r#SpiDevice,
    C: Codec,
{
    type Error = I::Error;

    /// Read this register from this spi device using the codec
    /// specified by the register (if any) or otherwise the
    /// default codec of the device.
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, I::Error>
    where
        R: ReadableRegister,
    {
        if TypeId::of::<R::SpiCodec>() == TypeId::of::<NoCodec>() {
            C::read_register::<R, _>(&mut self.interface).await
        } else {
            <R::SpiCodec as Codec>::read_register::<R, _>(&mut self.interface).await
        }
    }

    /// Write this register to this spi device using the codec
    /// specified by the register (if any) or otherwise the
    /// default codec of the device.
    #[inline]
    async fn write_register<R>(&mut self, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        if TypeId::of::<R::SpiCodec>() == TypeId::of::<NoCodec>() {
            C::write_register(&mut self.interface, register).await
        } else {
            <R::SpiCodec as Codec>::write_register(&mut self.interface, register).await
        }
    }
}
