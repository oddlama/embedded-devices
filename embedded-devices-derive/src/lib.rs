//! This crate provides procedural helper macro for embedded-devices

use proc_macro::TokenStream;
use syn::DeriveInput;

mod derive_measurement;
mod forward_command_fns;
mod forward_register_fns;
mod sensor;

#[proc_macro_attribute]
pub fn forward_command_fns(args: TokenStream, input: TokenStream) -> TokenStream {
    match forward_command_fns::forward_command_fns(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

#[proc_macro_attribute]
pub fn forward_register_fns(args: TokenStream, input: TokenStream) -> TokenStream {
    match forward_register_fns::forward_register_fns(args.into(), input.into()) {
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
