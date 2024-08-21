//! Defined Traits for embedded-registers-derive.
//! For Derive Docs see [embedded-registers-derive](https://docs.rs/embedded-registers-derive/latest/embedded_registers_derive/).
//!
//! This crate provides a procedural macro for effortless definitions of registers
//! in embedded device drivers. This is automatically generates functions to read/write
//! the register over I2C and SPI, although it isn't limited to those buses. The
//! resulting struct may trivially be extended to work with any other similar communication bus.
//!
//! - Allows defintion of read-only, write-only and read-write registers
//! - Generates I2C and SPI read/write functions
//! - Registers are defined as bitfields via [bondrewd](https://github.com/Devlyn-Nelson/Bondrewd).
//! - Only the accessed bitfield members are decoded, conserving memory and saving on CPU time.
//! - Supports both async and blocking operation modes
//!
//! This crate was made primarily for [embedded-devices](https://github.com/oddlama/embedded-devices),
//! which is a collection of drivers for a variety of different embedded sensors and devices.
//!
//! ## Defining a register
//!
//! Registers are defined simply by annotating a bondrewd struct with `#[register(address = 0x42, mode = "rw")]`.
//! The necessary derive attribute is added automatically.
//! Take for example this 2-byte read-write register at device address `0x42,0x43`, which contains two `u8` values:
//!
//! ```
//! #![feature(generic_arg_infer)]
//! use embedded_registers::register;
//!
//! #[register(address = 0x42, mode = "rw")]
//! #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
//! pub struct ValueRegister {
//!     pub width: u8,
//!     pub height: u8,
//! }
//! ```
//!
//! This will create two structs called `ValueRegister` and `ValueRegisterBitfield`.
//! The first will only contain a byte array `[u8; 2]` to store the packed register contents,
//! and the latter will contain the unpacked actual members as defined above.
//! You will always interface with a device using the packed data, which can be transferred over the bus as-is.
//!
//! The packed data contains methods to directly read/write the underlying storage array, which
//! means you can only unpack what you need, saving resources.
//!
//! > I find it a bit misleading that the members written in `ValueRegister` end up in `ValueRegisterBitfield`.
//! > So this might change in the future, but I currently cannot think of another design that is as simple
//! > to use as this one right now. The issue is that we need a struct for the packed data and one for
//! > the unpacked data. Since we usually deal with the packed data, and want to allow direct read/write
//! > operations on the packed data for performance, the naming gets confusing quite quickly.
//!
//! ### Accessing a register
//!
//! To access such a register, you need to obtain an interface that implements the `RegisterInterface` trait.
//! This crate already comes with an implementation of that trait for I2C and SPI devices called [`i2c::I2cDevice`] and [`spi::SpiDevice`] respectively.
//! You may then read the register simply by calling [`read_register`](RegisterInterface::read_register) or [`write_register`](RegisterInterface::write_register) on that interface.
//!
//! ```
//! # #![feature(generic_arg_infer)]
//! # use embedded_registers::register;
//!
//! # #[register(address = 0x42, mode = "rw")]
//! # #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
//! # pub struct ValueRegister {
//! #     pub width: u8,
//! #     pub height: u8,
//! # }
//!
//! use embedded_registers::{i2c::{I2cDevice, codecs::OneByteRegAddrCodec}, RegisterInterface};
//!
//! async fn init<I>(mut i2c_bus: I) -> Result<(), I::Error>
//! where
//!     I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! {
//!     // Imagine we already have constructed a device using
//!     // the i2c bus from your controller, a device address and default codec:
//!     let mut dev = I2cDevice::new(i2c_bus, 0x12, OneByteRegAddrCodec::default());
//!     // We can now retrieve the register
//!     let mut reg = dev.read_register::<ValueRegister>().await?;
//!
//!     // Unpack a specific field from the register and print it
//!     println!("{}", reg.read_width());
//!     // If you need all fields (or are not bound to tight resource constraints),
//!     // you can also unpack all fields and access them more conveniently
//!     let data = reg.read_all();
//!     // All bitfields implement Debug and defmt::Format, so you can conveniently
//!     // print the contents
//!     println!("{:?}", data);
//!
//!     // We can also change a single value
//!     reg.write_height(190);
//!     // Or pack a bitfield and replace everything
//!     reg.write_all(data); // same as reg = ValueRegister::new(data);
//!
//!     // Which we can now write back to the device, given that the register is writable.
//!     dev.write_register(reg).await?;
//!     Ok(())
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod i2c;
pub mod spi;

// Re-exports for derive macro
pub use bondrewd;
pub use bytemuck;

/// The basis trait for all registers. A register is a type
/// that maps to a specific register on an embedded device and
/// should own the raw data required for this register.
///
/// Additionally, a register knows the virtual address ossociated
/// to the embedded device, and a bitfield representation of the
/// data content.
pub trait Register: Default + Clone + bytemuck::Pod {
    /// The size of the register in bytes
    const REGISTER_SIZE: usize;
    /// The virtual address of this register
    const ADDRESS: u64;

    /// The associated bondrewd bitfield type
    type Bitfield;
    /// The SPI codec that should be used for this register when no
    /// codec is specified by the user. Setting this to `spi::codecs::NoCodec`
    /// will automatically cause accesses to use the device's default codec.
    /// If the device doesn't support SPI communication, this can be ignored.
    type SpiCodec: spi::Codec;
    /// The I2C codec that should be used for this register when no
    /// codec is specified by the user. Setting this to `i2c::codecs::NoCodec`
    /// will automatically cause accesses to use the device's default codec.
    /// If the device doesn't support I2C communication, this can be ignored.
    type I2cCodec: i2c::Codec;

    /// Provides immutable access to the raw data.
    fn data(&self) -> &[u8];
    /// Provides mutable access to the raw data.
    fn data_mut(&mut self) -> &mut [u8];
}

/// This trait is a marker trait implemented by any register that can be read via a specific bus interface.
pub trait ReadableRegister: Register {}

/// This trait is a marker trait implemented by any register that can be written via a specific bus interface.
pub trait WritableRegister: Register {}

/// A trait that is implemented by any bus interface and allows
/// devices with registers to share register read/write implementations
/// independent of the actual interface in use.
#[allow(async_fn_in_trait)]
#[maybe_async_cfg::maybe(sync(not(feature = "async")), async(feature = "async"), keep_self)]
pub trait RegisterInterface {
    type Error;

    /// Reads the given register via this interface
    async fn read_register<R>(&mut self) -> Result<R, Self::Error>
    where
        R: ReadableRegister;

    /// Writes the given register via this interface
    async fn write_register<R>(&mut self, register: impl AsRef<R>) -> Result<(), Self::Error>
    where
        R: WritableRegister;
}

// re-export the derive stuff
#[cfg(feature = "derive")]
#[doc(hidden)]
pub use embedded_registers_derive::*;
