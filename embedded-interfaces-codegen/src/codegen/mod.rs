//! Code generation for register definitions

use proc_macro2::Ident;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::Expr;

use crate::parser::*;

mod bit_manipulation;
mod bit_pattern;
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
    let attrs = registers_def.get_effective_attrs(register)?;

    // Extract register attributes
    let (addr, mode, size) = extract_register_attrs(&register.name, &attrs)?;
    let (codec_error, i2c_codec, spi_codec) = extract_codec_attrs(&register.name, &attrs)?;

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

fn extract_register_attrs(register_name: &Ident, attrs: &[Attr]) -> syn::Result<(TokenStream2, String, usize)> {
    let mut addr = None;
    let mut mode = None;
    let mut size = None;

    for attr in attrs {
        match attr.name.to_string().as_str() {
            "addr" => {
                let attr_value = &attr.value;
                addr = Some(quote! { #attr_value });
            }
            "mode" => {
                // Extract mode string from expression
                mode = Some(extract_mode_string(&attr.value)?);
            }
            "size" => {
                // Extract size integer from expression
                size = Some(extract_size_integer(&attr.value)?);
            }
            _ => {}
        }
    }

    let addr = addr.ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'addr' attribute"))?;
    let mode = mode.ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'mode' attribute"))?;
    let size = size.ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'size' attribute"))?;

    Ok((addr, mode, size))
}

fn extract_codec_attrs(
    register_name: &Ident,
    attrs: &[Attr],
) -> syn::Result<(TokenStream2, TokenStream2, TokenStream2)> {
    let mut codec_error = None;
    let mut i2c_codec = None;
    let mut spi_codec = None;

    for attr in attrs {
        let attr_value = &attr.value;
        match attr.name.to_string().as_str() {
            "codec_error" => {
                codec_error = Some(quote! { #attr_value });
            }
            "i2c_codec" => {
                i2c_codec = Some(quote! { #attr_value });
            }
            "spi_codec" => {
                spi_codec = Some(quote! { #attr_value });
            }
            _ => {}
        }
    }

    let codec_error = codec_error
        .ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'codec_error' attribute"))?;
    let i2c_codec =
        i2c_codec.ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'i2c_codec' attribute"))?;
    let spi_codec =
        spi_codec.ok_or_else(|| syn::Error::new_spanned(register_name, "Missing required 'spi_codec' attribute"))?;

    Ok((codec_error, i2c_codec, spi_codec))
}

/// Extract a mode string from an expression
fn extract_mode_string(expr: &Expr) -> syn::Result<String> {
    match expr {
        Expr::Lit(syn::ExprLit {
            lit: syn::Lit::Str(lit_str),
            ..
        }) => Ok(lit_str.value()),
        Expr::Path(path) => {
            if let Some(ident) = path.path.get_ident() {
                Ok(ident.to_string())
            } else {
                Err(syn::Error::new_spanned(
                    expr,
                    "Mode must be a string literal or identifier (e.g., \"rw\", r, rw)",
                ))
            }
        }
        _ => Err(syn::Error::new_spanned(
            expr,
            "Mode must be a string literal or identifier (e.g., \"rw\", r, rw)",
        )),
    }
}

/// Extract a size integer from an expression
fn extract_size_integer(expr: &Expr) -> syn::Result<usize> {
    match expr {
        Expr::Lit(syn::ExprLit {
            lit: syn::Lit::Int(lit_int),
            ..
        }) => lit_int.base10_parse::<usize>(),
        _ => Err(syn::Error::new_spanned(expr, "Size must be an integer literal")),
    }
}
