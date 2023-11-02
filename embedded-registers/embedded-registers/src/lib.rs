#![cfg_attr(not(feature = "std"), no_std)]

pub trait Register: Default + Clone {
    const REGISTER_SIZE: usize;
    const ADDRESS: u8;

    type Bitfield;
    type Data;

    fn data(&self) -> &[u8];
    fn data_mut(&mut self) -> &mut [u8];
}

pub trait RegisterRead {}
pub trait RegisterWrite {}

// re-export the derive stuff
#[cfg(feature = "derive")]
#[doc(hidden)]
pub use embedded_registers_derive::*;
