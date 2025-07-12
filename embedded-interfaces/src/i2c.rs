#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
/// This represents a specific device bound to an I2C bus.
pub struct I2cBoundBus<I, A>
where
    I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
    A: hal::i2c::AddressMode + Copy,
{
    /// I2c interface
    pub interface: I,
    /// Device address
    pub address: A,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async")
)]
/// This represents an I2C device on an I2C bus.
pub struct I2cDevice<I, A>
where
    I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
    A: hal::i2c::AddressMode + Copy,
{
    /// I2c interface and device address
    pub bound_bus: I2cBoundBus<I, A>,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, A> I2cDevice<I, A>
where
    I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
    A: hal::i2c::AddressMode + Copy,
{
    /// Create a new I2cDevice from an interface and device address
    pub fn new(interface: I, address: A) -> Self {
        Self {
            bound_bus: I2cBoundBus { interface, address },
        }
    }
}
