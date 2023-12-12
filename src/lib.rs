//! Device driver implementations for many embedded sensors and devices
//!
#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(generic_arg_infer)]

pub mod devices;
pub(crate) mod simple_device;
