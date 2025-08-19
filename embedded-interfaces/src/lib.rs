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

pub mod bitdump;
pub mod commands;
pub mod i2c;
pub mod packable;
pub mod registers;
pub mod spi;

/// A combined error type for Codec or Bus errors
#[derive(Debug, thiserror::Error)]
pub enum TransportError<CodecError, BusError> {
    /// The codec failed to encode or decode something
    #[error("codec error")]
    Codec(CodecError),
    /// An unexpected error
    #[error("unexpected: {0}")]
    Unexpected(&'static str),
    /// An error ocurred on the underlying interface
    #[error("bus error")]
    Bus(#[from] BusError),
}

/// Objects that implement this trait can produce a pretty-printed bit-dump of it's internal
/// structure.
pub trait BitdumpFormattable {
    #[cfg(feature = "std")]
    /// Returns a bitdump formatter for the current object.
    fn bitdump(&self) -> bitdump::BitdumpFormatter;
}

pub mod codegen {
    // re-export the derive stuff
    pub use embedded_interfaces_codegen::*;
}
