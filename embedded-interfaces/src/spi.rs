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
