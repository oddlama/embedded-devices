use arrayvec::ArrayVec;

use crate::{ReadableRegister, RegisterInterface, WritableRegister};

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
/// This represents an i2c device on an async i2c bus
pub struct I2cDevice<I>
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
impl<I> RegisterInterface for I2cDevice<I>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
{
    type Error = I::Error;

    /// Read this register from this i2c device.
    #[inline]
    async fn read_register<R>(&mut self) -> Result<R, I::Error>
    where
        R: ReadableRegister,
    {
        let mut register = R::default();
        self.interface
            .write_read(self.address, R::ADDRESS, register.data_mut())
            .await?;
        Ok(register)
    }

    /// Write this register to this i2c device.
    #[inline]
    async fn write_register<R>(&mut self, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        // FIXME: this is preferrable, but currently disabled because the embassy
        // embedded-hal implementation doesn't support this function yet.
        // (uses todo!() in https://github.com/embassy-rs/embassy/blob/915423fc63f209059a2242fbc3ad3b88d796514f/embassy-nrf/src/twim.rs#L871-L880)
        //self.interface
        //    .transaction(
        //        self.address,
        //        &mut [Operation::Write(R::ADDRESS), Operation::Write(register.as_ref().data())],
        //    )
        //    .await

        let mut data: ArrayVec<_, 64> = ArrayVec::new();
        let addr_len = R::ADDRESS.len();
        let data_len = register.as_ref().data().len();
        let len = addr_len + data_len;
        assert!(len <= data.capacity());

        data[0..addr_len].copy_from_slice(R::ADDRESS);
        data[addr_len..len].copy_from_slice(register.as_ref().data());
        self.interface.write(self.address, &data[..len]).await
    }
}
