pub mod codecs;

use core::any::TypeId;

use codecs::NoCodec;

use crate::{ReadableRegister, RegisterInterface, WritableRegister};

/// Represents a trait for SPI codecs. These are the function that is
/// responsible to write and read data to registers given the register
/// address and the raw data. Different devices can have different
/// ways to encode the desired address, R/W bit location, continuous-read
/// mode and more.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
#[allow(async_fn_in_trait)]
pub trait Codec: Default + 'static {
    /// Read this register from the given SPI interface/device.
    async fn read_register<R, I>(&mut self, interface: &mut I) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::spi::SpiDevice;

    /// Write this register to the given SPI interface/device.
    async fn write_register<R, I>(&mut self, interface: &mut I, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::spi::SpiDevice;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
/// This represents an spi device on an spi bus.
pub struct SpiDevice<I, C>
where
    I: hal::spi::SpiDevice,
    C: Codec,
{
    /// Spi interface
    pub interface: I,
    /// The default codec used to interface with registers
    /// that don't explicitly specify a codec themselves.
    /// Usually this is a simple codec designating a bit for R/W
    /// and a bit-range for the register address. See implementors of
    /// [`Codec`](Codec) for more information on available codecs.
    pub default_codec: C,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I, C> RegisterInterface for SpiDevice<I, C>
where
    I: hal::spi::SpiDevice,
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
            self.default_codec.read_register::<R, _>(&mut self.interface).await
        } else {
            let mut codec = R::SpiCodec::default();
            codec.read_register::<R, _>(&mut self.interface).await
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
            self.default_codec.write_register(&mut self.interface, register).await
        } else {
            let mut codec = R::SpiCodec::default();
            codec.write_register(&mut self.interface, register).await
        }
    }
}
