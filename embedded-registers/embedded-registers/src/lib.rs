//! Defined Traits for embedded-registers-derive.
//! For Derive Docs see [embedded-registers-derive](https://docs.rs/embedded-registers-derive/latest/embedded_registers_derive/)

#![cfg_attr(not(feature = "std"), no_std)]

/// The basis trait for all registers. A register is a type
/// that maps to a specific register on an embedded device and
/// should own the raw data required for this register.
///
/// Additionally, a register knows the virtual address ossociated
/// to the embedded device, and a bitfield representation of the
/// data content.
pub trait Register: Default + Clone {
    /// The size of the register in bytes
    const REGISTER_SIZE: usize;
    /// The virtual address of this register
    const ADDRESS: u8;

    /// The associated bondrewd bitfield type
    type Bitfield;

    /// Provides immutable access to the raw data.
    fn data(&self) -> &[u8];
    /// Provides mutable access to the raw data.
    fn data_mut(&mut self) -> &mut [u8];
}

/// This trait is implemented by any register that can be read via a specific bus interface.
#[allow(async_fn_in_trait)]
pub trait RegisterRead: Register {
    /// Asynchronously read this register from the given i2c bus and device address.
    #[inline]
    async fn read_i2c<I>(i2c: &mut I, address: u8) -> Result<Self, I::Error>
    where
        I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
    {
        let mut register = Self::default();
        i2c.write_read(address, &[Self::ADDRESS], register.data_mut()).await?;
        Ok(register)
    }

    /// Synchronously read this register from the given i2c bus and device address.
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

/// This trait is implemented by any register that can be written via a specific bus interface.
#[allow(async_fn_in_trait)]
pub trait RegisterWrite: Register {
    /// Asynchronously write this register to the given i2c bus and device address.
    #[inline]
    async fn write_i2c<I>(&self, i2c: &mut I, address: u8) -> Result<(), I::Error>
    where
        I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
    {
        // FIXME: transaction is currently not implemented in embedded_hal_async,
        // so we need to construct an array...
        //i2c.transaction(
        //    address,
        //    &mut [Operation::Write(&[Self::ADDRESS]), Operation::Write(self.data())],
        //)
        //.await

        let mut data = [0u8; 8];
        let len = 1 + self.data().len();
        data[0] = Self::ADDRESS;
        data[1..len + 1].copy_from_slice(self.data());
        i2c.write(address, &data).await
    }

    /// Synchronously write this register to the given i2c bus and device address.
    #[inline]
    fn write_i2c_blocking<I>(&self, i2c: &mut I, address: u8) -> Result<(), I::Error>
    where
        I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
    {
        let mut data = [0u8; 8];
        let len = 1 + self.data().len();
        data[0] = Self::ADDRESS;
        data[1..len + 1].copy_from_slice(self.data());
        i2c.write(address, &data)

        // FIXME: transaction is currently not implemented in embedded_hal_async,
        // so we need to construct an array...
        //i2c.transaction(
        //    address,
        //    &mut [Operation::Write(&[Self::ADDRESS]), Operation::Write(self.data())],
        //)
    }
}

// re-export the derive stuff
#[cfg(feature = "derive")]
#[doc(hidden)]
pub use embedded_registers_derive::*;
