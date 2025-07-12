//! Code generation for register definitions

use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::Ident;

use crate::parser::*;

mod packed_struct;
mod register_traits;

pub use packed_struct::*;
pub use register_traits::*;

pub fn generate_registers(registers_def: &RegistersDefinition) -> syn::Result<TokenStream2> {
    let mut generated = TokenStream2::new();

    // Generate code for each register
    for register in &registers_def.registers {
        let register_code = generate_register(registers_def, register)?;
        generated.extend(register_code);
    }

    Ok(generated)
}

fn generate_register(registers_def: &RegistersDefinition, register: &RegisterDefinition) -> syn::Result<TokenStream2> {
    let register_name = &register.name;
    let unpacked_name = format_ident!("{}Unpacked", register_name);

    // Get effective attributes (defaults + register-specific)
    let attrs = registers_def.get_effective_attrs(register);

    // Extract register attributes
    let (addr, mode, size) = extract_register_attrs(&attrs)?;
    let (codec_error, i2c_codec, spi_codec) = extract_codec_attrs(&attrs)?;

    // Generate the packed/unpacked struct pair (reusable part)
    let packed_structs = generate_packed_struct_pair(
        register_name,
        &unpacked_name,
        &register.fields,
        &register.attributes,
        size,
    )?;

    // Generate register-specific trait implementations
    let trait_impls = generate_register_trait_implementations(
        register_name,
        &unpacked_name,
        &addr,
        &mode,
        size,
        &codec_error,
        &i2c_codec,
        &spi_codec,
    )?;

    Ok(quote! {
        #packed_structs
        #trait_impls
    })
}

fn extract_register_attrs(attrs: &[RegisterAttr]) -> syn::Result<(TokenStream2, String, usize)> {
    let mut addr = None;
    let mut mode = None;
    let mut size = None;

    for attr in attrs {
        match attr.name.to_string().as_str() {
            "addr" => {
                if let RegisterAttrValue::Int(lit_int) = &attr.value {
                    addr = Some(quote! { #lit_int });
                }
            }
            "mode" => {
                if let RegisterAttrValue::Ident(ident) = &attr.value {
                    mode = Some(ident.to_string());
                }
            }
            "size" => {
                if let RegisterAttrValue::Int(lit_int) = &attr.value {
                    size = Some(lit_int.base10_parse::<usize>()?);
                }
            }
            _ => {}
        }
    }

    let addr = addr.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing addr attribute"))?;
    let mode = mode.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing mode attribute"))?;
    let size = size.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing size attribute"))?;

    Ok((addr, mode, size))
}

fn extract_codec_attrs(attrs: &[RegisterAttr]) -> syn::Result<(TokenStream2, TokenStream2, TokenStream2)> {
    let mut codec_error = None;
    let mut i2c_codec = None;
    let mut spi_codec = None;

    for attr in attrs {
        match attr.name.to_string().as_str() {
            "codec_error" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    codec_error = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    codec_error = Some(quote! { #ident });
                }
                _ => {}
            },
            "i2c_codec" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    i2c_codec = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    i2c_codec = Some(quote! { #ident });
                }
                _ => {}
            },
            "spi_codec" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    spi_codec = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    spi_codec = Some(quote! { #ident });
                }
                _ => {}
            },
            _ => {}
        }
    }

    let codec_error = codec_error.unwrap_or_else(|| quote! { () });
    let i2c_codec = i2c_codec.unwrap_or_else(|| quote! { () });
    let spi_codec = spi_codec.unwrap_or_else(|| quote! { () });

    Ok((codec_error, i2c_codec, spi_codec))
}
