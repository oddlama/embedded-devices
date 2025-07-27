//! This crate provides procedural helper macro for embedded-devices

use proc_macro::TokenStream;
use syn::DeriveInput;

mod derive_measurement;
mod device;
mod device_impl;
mod sensor;

#[proc_macro_attribute]
pub fn device(args: TokenStream, input: TokenStream) -> TokenStream {
    match device::device(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

#[proc_macro_attribute]
pub fn device_impl(args: TokenStream, input: TokenStream) -> TokenStream {
    match device_impl::device_impl(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

#[proc_macro_derive(Measurement, attributes(measurement))]
pub fn derive_measurement(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as DeriveInput);
    match derive_measurement::generate(&input) {
        Ok(result) => result.into(),
        Err(e) => e.to_compile_error().into(),
    }
}

#[proc_macro_attribute]
pub fn sensor(args: TokenStream, input: TokenStream) -> TokenStream {
    match sensor::sensor(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}
