//! This crate provides procedural helper macro for embedded-devices

use proc_macro as pc;

mod device;
mod device_impl;
mod device_register;

#[proc_macro_attribute]
pub fn device(args: pc::TokenStream, input: pc::TokenStream) -> pc::TokenStream {
    match device::device(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

#[proc_macro_attribute]
pub fn device_impl(args: pc::TokenStream, input: pc::TokenStream) -> pc::TokenStream {
    match device_impl::device_impl(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

#[proc_macro_attribute]
pub fn device_register(args: pc::TokenStream, input: pc::TokenStream) -> pc::TokenStream {
    match device_register::device_register(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}
