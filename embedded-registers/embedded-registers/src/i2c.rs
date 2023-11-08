use crate::{ReadableRegister, RegisterInterfaceAsync, RegisterInterfaceSync, WritableRegister};

/// This represents an i2c device on an async i2c bus
pub struct I2cDeviceAsync<I>
where
    I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
{
    /// I2c interface
    pub interface: I,
    /// Device address
    pub address: u8,
}

impl<I> RegisterInterfaceAsync for I2cDeviceAsync<I>
where
    I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
{
    type Error = I::Error;

    /// Read this register from the given i2c bus and device address.
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

    /// Write this register to the given i2c bus and device address.
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
        self.interface.write(self.address, &data[0..len]).await
    }
}

/// This represents an i2c device on a i2c bus
pub struct I2cDeviceSync<I>
where
    I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
{
    /// I2c interface
    pub interface: I,
    /// Device address
    pub address: u8,
}

impl<I> RegisterInterfaceSync for I2cDeviceSync<I>
where
    I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
{
    type Error = I::Error;

    /// Read this register from the given i2c bus and device address.
    #[inline]
    fn read_register<R>(&mut self) -> Result<R, I::Error>
    where
        R: ReadableRegister,
    {
        let mut register = R::default();
        self.interface
            .write_read(self.address, &[R::ADDRESS], register.data_mut())?;
        Ok(register)
    }

    /// Write this register to the given i2c bus and device address.
    #[inline]
    fn write_register<R>(&mut self, register: &R) -> Result<(), I::Error>
    where
        R: WritableRegister,
    {
        // FIXME: transaction is currently not implemented in embedded_hal_async,
        // so we need to construct an array...
        //self.interface.transaction(
        //    self.address,
        //    &mut [Operation::Write(&[R::ADDRESS]), Operation::Write(register.data())],
        //)

        let mut data = [0u8; 8];
        let len = register.data().len();
        data[0] = R::ADDRESS;
        data[1..len + 1].copy_from_slice(register.data());
        self.interface.write(self.address, &data[0..len])
    }
}
