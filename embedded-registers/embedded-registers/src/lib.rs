#![cfg_attr(not(feature = "std"), no_std)]

use embedded_hal::i2c::Operation;

pub trait Register: Default + Clone {
    const REGISTER_SIZE: usize;
    const ADDRESS: u8;

    type Bitfield;
    type Data;

    fn data(&self) -> &[u8];
    fn data_mut(&mut self) -> &mut [u8];
}

#[allow(async_fn_in_trait)]
pub trait RegisterRead: Register {
    #[inline]
    async fn read_i2c<I>(i2c: &mut I, address: u8) -> Result<Self, I::Error>
    where
        I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
    {
        let mut register = Self::default();
        i2c.write_read(address, &[Self::ADDRESS], register.data_mut())
            .await?;
        Ok(register)
    }

    #[inline]
    fn read_i2c_blocking<I>(i2c: &mut I, address: u8) -> Result<Self, I::Error>
    where
        I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
    {
        let mut register = Self::default();
        i2c.write_read(address, &[Self::ADDRESS], register.data_mut())?;
        Ok(register)
    }
}

#[allow(async_fn_in_trait)]
pub trait RegisterWrite: Register {
    #[inline]
    async fn write_i2c<I>(&self, i2c: &mut I, address: u8) -> Result<(), I::Error>
    where
        I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
    {
        i2c.transaction(
            address,
            &mut [
                Operation::Write(&[Self::ADDRESS]),
                Operation::Write(self.data()),
            ],
        )
        .await
    }

    #[inline]
    fn write_i2c_blocking<I>(&self, i2c: &mut I, address: u8) -> Result<(), I::Error>
    where
        I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
    {
        i2c.transaction(
            address,
            &mut [
                Operation::Write(&[Self::ADDRESS]),
                Operation::Write(self.data()),
            ],
        )
    }
}

// re-export the derive stuff
#[cfg(feature = "derive")]
#[doc(hidden)]
pub use embedded_registers_derive::*;
