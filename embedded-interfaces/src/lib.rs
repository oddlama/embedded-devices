//! A comprehensive framework for building type-safe and ergonomic embedded device drivers.
//!
#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]
#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You must enable at least one of the create features `sync` or `async`");

// Re-exports for codegen macro
pub use bitvec;
pub use bytemuck;
pub use const_format;

pub mod commands;
pub mod i2c;
pub mod packable;
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
