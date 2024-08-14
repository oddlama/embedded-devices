pub mod codecs;

use core::any::TypeId;

use codecs::NoCodec;

use crate::{ReadableRegister, RegisterInterface, WritableRegister};

/// Represents a trait for I2C codecs. These are responsible to perform
/// writes and reads to registers, given the register address and
/// the raw data. Different devices can have different ways to encode
/// the desired address, address size, continuous-read mode and more.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
#[allow(async_fn_in_trait)]
pub trait Codec: Default + 'static {
    /// Read this register from the given I2C interface/device.
    async fn read_register<R, I>(&mut self, bound_bus: &mut I2cBoundBus<I>) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::i2c::I2c + hal::i2c::ErrorType;

    /// Write this register to the given I2C interface/device.
    async fn write_register<R, I>(
        &mut self,
        bound_bus: &mut I2cBoundBus<I>,
        register: impl AsRef<R>,
    ) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::i2c::I2c + hal::i2c::ErrorType;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
/// This represents a specific device bound to an I2C bus.
pub struct I2cBoundBus<I>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
{
    /// I2c interface
    pub interface: I,
    /// Device address
    pub address: u8,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
/// This represents an I2C device on an I2C bus, including
/// a default codec.
pub struct I2cDevice<I, C>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
    C: Codec,
{
    /// I2c interface and device address
    pub bound_bus: I2cBoundBus<I>,
    /// The default codec used to interface with registers
    /// that don't explicitly specify a codec themselves.
    /// Usually this is a simple codec specifying address size and some metadata.
    /// See implementors of [`Codec`](Codec) for more information on available codecs.
    pub default_codec: C,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I, C> I2cDevice<I, C>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
    C: Codec,
{
    /// Create a new I2cDevice from an interface, device address and default codec.
    pub fn new(interface: I, address: u8, default_codec: C) -> Self {
        Self {
            bound_bus: I2cBoundBus { interface, address },
            default_codec,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I, C> RegisterInterface for I2cDevice<I, C>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
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
        if TypeId::of::<R::I2cCodec>() == TypeId::of::<NoCodec>() {
            self.default_codec.read_register::<R, _>(&mut self.bound_bus).await
        } else {
            let mut codec = R::I2cCodec::default();
            codec.read_register::<R, _>(&mut self.bound_bus).await
        }
    }

    /// Write this register to this i2c device using the codec
    /// specified by the register (if any) or otherwise the
    /// default codec of the device.
    #[inline]
    async fn write_register<R>(&mut self, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        if TypeId::of::<R::I2cCodec>() == TypeId::of::<NoCodec>() {
            self.default_codec.write_register(&mut self.bound_bus, register).await
        } else {
            let mut codec = R::I2cCodec::default();
            codec.write_register(&mut self.bound_bus, register).await
        }
    }
}
