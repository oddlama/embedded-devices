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
            .write_read(self.address, &[R::ADDRESS], register.data_mut())
            .await?;
        Ok(register)
    }

    /// Write this register to this i2c device.
    #[inline]
    async fn write_register<R>(&mut self, register: &R) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        // FIXME: transaction is currently not implemented in embedded_hal_async,
        // so we need to construct an array...
        //self.interface.transaction(
        //    self.address,
        //    &mut [Operation::Write(&[R::ADDRESS]), Operation::Write(self.data())],
        //)
        //.await

        let mut data = [0u8; 8];
        let len = register.data().len();
        data[0] = R::ADDRESS;
        data[1..len + 1].copy_from_slice(register.data());
        self.interface.write(self.address, &data[..len + 1]).await
    }
}
