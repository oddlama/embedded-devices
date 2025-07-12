//! Defines traits for embedded-registers-derive.
//! For Derive Docs see [embedded-registers-derive](https://docs.rs/embedded-registers-derive/latest/embedded_registers_derive/).
//!
//! This crate provides a procedural macro for effortless definitions of registers
//! in embedded device drivers. This is automatically generates functions to read/write
//! the register over I2C and SPI, although it isn't limited to those buses. The
//! resulting struct may be trivially extended to work with any other similar communication bus.
//!
//! - Allows defintion of read-only, write-only and read-write registers
//! - Generates I2C and SPI read/write functions
//! - Registers are defined as bitfields via [bondrewd](https://github.com/Devlyn-Nelson/Bondrewd).
//! - Only the accessed bitfield members are decoded, conserving memory and saving on CPU time.
//! - Supports both async and blocking operation modes simultaneously by generating two versions of
//!   each driver using [maybe_async_cfg](https://docs.rs/maybe-async-cfg/latest/maybe_async_cfg)
//!
//! This crate was made primarily for [embedded-devices](https://github.com/oddlama/embedded-devices),
//! which is a collection of drivers for a variety of different embedded sensors and devices.
//!
//! ## Defining a register
//!
//! Registers are defined simply by annotating a bondrewd struct with `#[register(address = 0x42, mode = "rw", codec = "OneByteRegAddrCodec")]`.
//! The necessary derive attribute for Bondrewd is added automatically.
//! Take for example this 2-byte read-write register at device addresses `0x42,0x43`, which contains two `u8` values:
//!
//! ```
//! #![feature(generic_arg_infer)]
//! use embedded_registers::register;
//! use embedded_registers::i2c::codecs::OneByteRegAddrCodec;
//!
//! #[register(address = 0x42, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
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
//! ### Accessing a register (async)
//!
//! To access such a register, you need to obtain an interface that implements the `RegisterInterfaceAsync` trait.
//! This crate already comes with an implementation of that trait for I2C and SPI devices called [`i2c::I2cDeviceAsync`] and [`spi::SpiDeviceAsync`] respectively.
//! You may then read the register simply by calling [`read_register`](RegisterInterfaceAsync::read_register) or [`write_register`](RegisterInterfaceAsync::write_register) on that interface.
//!
//! ```rust, only_if(async)
//! # #![feature(generic_arg_infer)]
//! # use embedded_registers::register;
//! # use embedded_registers::i2c::codecs::OneByteRegAddrCodec;
//! # #[register(address = 0x42, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
//! # #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
//! # pub struct ValueRegister {
//! #     pub width: u8,
//! #     pub height: u8,
//! # }
//! use embedded_registers::{i2c::I2cDeviceAsync, TransportError, RegisterInterfaceAsync};
//!
//! async fn init<I>(mut i2c_bus: I) -> Result<(), TransportError<(), I::Error>>
//! where
//!     I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! {
//!     // Imagine we already have constructed a device using
//!     // the i2c bus from your controller and a device address:
//!     let mut dev = I2cDeviceAsync::new(i2c_bus, 0x12);
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
#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You must enable at least one of the create features `sync` or `async`");

// Re-exports for codegen macro
pub use bytemuck;

pub mod commands;
pub mod i2c;
pub mod registers;
pub mod spi;

/// A combined error type for Codec or Bus errors
#[derive(Debug, thiserror::Error)]
pub enum TransportError<CodecError, BusError> {
    /// The codec failed to encore or decode this register
    #[error("codec error")]
    Codec(CodecError),
    /// An error ocurred on the underlying interface
    #[error("bus error")]
    Bus(#[from] BusError),
}

pub mod codegen {
    // re-export the derive stuff
    pub use embedded_interfaces_codegen::*;
}
