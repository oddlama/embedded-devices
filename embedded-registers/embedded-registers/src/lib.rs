//! Defined Traits for embedded-registers-derive.
//! For Derive Docs see [embedded-registers-derive](https://docs.rs/embedded-registers-derive/latest/embedded_registers_derive/)

#![cfg_attr(not(feature = "std"), no_std)]

pub mod i2c;
pub mod spi;

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

/// This trait is a marker trait implemented by any register that can be read via a specific bus interface.
pub trait ReadableRegister: Register {}

/// This trait is a marker trait implemented by any register that can be written via a specific bus interface.
pub trait WritableRegister: Register {}

/// A trait that is implemented by any async bus interface and allows
/// devices with registers to share read implementations independent
/// of the actual interface.
#[allow(async_fn_in_trait)]
pub trait RegisterInterfaceAsync {
    type Error;

    /// Reads the given register via this interface
    async fn read_register<R>(&mut self) -> Result<R, Self::Error>
    where
        R: ReadableRegister;

    /// Writes the given register via this interface
    async fn write_register<R>(&mut self, register: &R) -> Result<(), Self::Error>
    where
        R: WritableRegister;
}

/// A trait that is implemented by any sync bus interface and allows
/// devices with registers to share read implementations independent
/// of the actual interface.
pub trait RegisterInterfaceSync {
    type Error;

    /// Reads the given register via this interface
    fn read_register<R>(&mut self) -> Result<R, Self::Error>
    where
        R: ReadableRegister;

    /// Writes the given register via this interface
    fn write_register<R>(&mut self, register: &R) -> Result<(), Self::Error>
    where
        R: WritableRegister;
}

/// Represents an object that owns a specific async register interface.
pub trait RegisterInterfaceOwnerAsync<I>
where
    I: RegisterInterfaceAsync,
{
    fn interface(&mut self) -> &mut I;
}

/// Represents an object that owns a specific sync register interface.
pub trait RegisterInterfaceOwnerSync<I>
where
    I: RegisterInterfaceSync,
{
    fn interface(&mut self) -> &mut I;
}

// re-export the derive stuff
#[cfg(feature = "derive")]
#[doc(hidden)]
pub use embedded_registers_derive::*;
