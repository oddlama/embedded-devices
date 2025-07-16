//! Code generation for interface objects

use proc_macro2::Ident;
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::Expr;

use crate::parser::*;

mod bit_helpers;
mod bit_pattern;
mod enum_gen;
mod pack;
mod packed_struct;
mod register_traits;
mod unpack;

pub use packed_struct::*;
pub use register_traits::*;

pub fn generate_interface_objects(interface_def: &InterfaceObjectsDefinition) -> syn::Result<TokenStream2> {
    let mut generated = TokenStream2::new();

    // Generate code for each definition
    for definition in &interface_def.definitions {
        match definition {
            Definition::Register(register) => {
                let register_code = generate_register(interface_def, register)?;
                generated.extend(register_code);
            }
            Definition::Struct(struct_def) => {
                let struct_code = generate_struct(interface_def, struct_def)?;
                generated.extend(struct_code);
            }
            Definition::Enum(enum_def) => {
                let enum_code = enum_gen::generate_enum(enum_def)?;
                generated.extend(enum_code);
            }
        }
    }

    Ok(generated)
}

fn generate_register(
    interface_def: &InterfaceObjectsDefinition,
    register: &RegisterDefinition,
) -> syn::Result<TokenStream2> {
    let register_name = &register.name;
    let unpacked_name = format_ident!("{}Unpacked", register_name);

    // Get effective attributes (defaults + register-specific)
    let attrs = interface_def.get_effective_register_attrs(register)?;

    // Extract register attributes
    let (addr, mode, size) = extract_register_attrs(&register.name, &attrs)?;
    let (codec_error, i2c_codec, spi_codec) = extract_codec_attrs(&register.name, &attrs)?;

    // Generate the packed/unpacked struct pair (reusable part)
    let packed_structs = generate_packed_struct_pair(
        interface_def,
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

fn generate_struct(
    interface_def: &InterfaceObjectsDefinition,
    struct_def: &StructDefinition,
) -> syn::Result<TokenStream2> {
    let struct_name = &struct_def.name;
    let unpacked_name = format_ident!("{}Unpacked", struct_name);

    // Get effective attributes (only size is allowed for structs)
    let attrs = interface_def.get_effective_struct_attrs(struct_def)?;

    // Extract size attribute
    let size = extract_struct_size(&struct_def.name, &attrs)?;

    // Generate the packed/unpacked struct pair (reusable part)
    let packed_structs = generate_packed_struct_pair(
        interface_def,
        struct_name,
        &unpacked_name,
        &struct_def.fields,
        &struct_def.attributes,
        size,
    )?;

    Ok(packed_structs)
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

fn extract_struct_size(struct_name: &Ident, attrs: &[Attr]) -> syn::Result<usize> {
    for attr in attrs {
        if attr.name.to_string().as_str() == "size" {
            return extract_size_integer(&attr.value);
        }
    }
    Err(syn::Error::new_spanned(
        struct_name,
        "Missing required 'size' attribute",
    ))
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
