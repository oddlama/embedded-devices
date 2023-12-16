use arrayvec::ArrayVec;

use crate::{ReadableRegister, RegisterInterface, WritableRegister};

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
/// This represents an spi device on an async spi bus
pub struct SpiDevice<I>
where
    I: hal::spi::SpiDevice,
{
    /// Spi interface
    pub interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> RegisterInterface for SpiDevice<I>
where
    I: hal::spi::SpiDevice,
{
    type Error = I::Error;

    /// Read this register from this spi device.
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, I::Error>
    where
        R: ReadableRegister,
    {
        let mut register = R::default();
        self.interface.transfer(register.data_mut(), R::ADDRESS).await?;
        Ok(register)
    }

    /// Write this register to this spi device.
    #[inline]
    async fn write_register<R>(&mut self, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        let mut data: ArrayVec<_, 64> = ArrayVec::new();
        data.try_extend_from_slice(R::ADDRESS).unwrap();
        data.try_extend_from_slice(register.as_ref().data()).unwrap();
        self.interface.transfer(&mut [], &data).await
    }
}
