//! Register-specific trait implementations
//!
//! This module contains the register-specific logic for generating trait
//! implementations like Register, ReadableRegister, and WritableRegister.

use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::Ident;

use crate::parser::InterfaceObjectsDefinition;

/// Generate register-specific trait implementations
#[allow(clippy::too_many_arguments)]
pub fn generate_register_trait_implementations(
    interface_def: &InterfaceObjectsDefinition,
    register_name: &Ident,
    unpacked_name: &Ident,
    addr: &TokenStream2,
    mode: &str,
    size: usize,
    codec_error: &TokenStream2,
    i2c_codec: &TokenStream2,
    spi_codec: &TokenStream2,
) -> syn::Result<TokenStream2> {
    let register_traits = generate_register_traits(
        interface_def,
        register_name,
        unpacked_name,
        addr,
        size,
        codec_error,
        i2c_codec,
        spi_codec,
    )?;

    let mode_traits = generate_mode_traits(register_name, mode)?;

    Ok(quote! {
        #register_traits
        #mode_traits
    })
}

/// Generate the main Register trait implementation
#[allow(clippy::too_many_arguments)]
fn generate_register_traits(
    interface_def: &InterfaceObjectsDefinition,
    register_name: &Ident,
    unpacked_name: &Ident,
    addr: &TokenStream2,
    size: usize,
    codec_error: &TokenStream2,
    i2c_codec: &TokenStream2,
    spi_codec: &TokenStream2,
) -> syn::Result<TokenStream2> {
    let register_marker_impls: Vec<_> = interface_def
        .register_devices
        .as_ref()
        .map(|x| x.devices.clone())
        .unwrap_or_default()
        .iter()
        .map(|device| {
            let device_register_marker = format_ident!("{}Register", device);
            quote! {
                impl #device_register_marker for #register_name {}
            }
        })
        .collect();

    Ok(quote! {
        #(#register_marker_impls)*

        impl embedded_interfaces::registers::Register for #register_name {
            type Unpacked = #unpacked_name;
            type CodecError = #codec_error;
            type SpiCodec = #spi_codec;
            type I2cCodec = #i2c_codec;
            const REGISTER_SIZE: usize = #size;
            const ADDRESS: u64 = #addr;
        }
    })
}

/// Generate readable/writable traits based on mode
fn generate_mode_traits(register_name: &Ident, mode: &str) -> syn::Result<TokenStream2> {
    match mode {
        "r" => Ok(quote! { impl embedded_interfaces::registers::ReadableRegister for #register_name {} }),
        "w" => Ok(quote! { impl embedded_interfaces::registers::WritableRegister for #register_name {} }),
        "rw" => Ok(quote! {
            impl embedded_interfaces::registers::ReadableRegister for #register_name {}
            impl embedded_interfaces::registers::WritableRegister for #register_name {}
        }),
        _ => Err(syn::Error::new_spanned(
            register_name,
            format!("Invalid mode: {}", mode),
        )),
    }
}
