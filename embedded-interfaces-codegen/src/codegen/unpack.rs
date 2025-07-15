//! Code generation for unpacking bit-packed structs
//!
//! This module generates the unpack method that converts from packed bytes
//! to unpacked field structs.

use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::{Ident, Type};

use super::{
    bit_helpers::{generate_copy_from_normalized_ranges, get_element_bits, get_type_bits},
    bit_pattern::{IterChunksExactBits, NormalizedRange, ProcessedField},
};

/// Generate the body of the unpack method
pub fn generate_unpack_body(unpacked_name: &Ident, processed_fields: &[ProcessedField]) -> syn::Result<TokenStream2> {
    let mut field_extractions: Vec<TokenStream2> = Vec::new();

    for processed in processed_fields {
        let field_name = &processed.field.name;
        let field_type = &processed.field.field_type;
        let ranges = &processed.normalized_ranges;

        // Skip reserved fields
        if processed.field.is_reserved() {
            continue;
        }

        let unpack_expr =
            generate_unpack_expression(field_type, ranges).map_err(|e| syn::Error::new_spanned(field_name, e))?;
        field_extractions.push(quote! {
            #field_name: #unpack_expr
        });
    }

    Ok(quote! {
        use embedded_interfaces::bitvec::{order::Msb0, view::BitView};

        let src = self.0;
        let src_bits = src.view_bits::<Msb0>();

        #unpacked_name {
            #(#field_extractions,)*
        }
    })
}

/// Generate an expression to unpack a field from bytes
fn generate_unpack_expression(field_type: &Type, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                match type_name.as_str() {
                    "bool" => generate_bool_unpack(ranges),
                    "u8" | "u16" | "u32" | "u64" | "u128" => generate_unsigned_unpack(&type_name, ranges),
                    "i8" | "i16" | "i32" | "i64" | "i128" => generate_signed_unpack(&type_name, ranges),
                    "f32" | "f64" => generate_float_unpack(&type_name, ranges),
                    "usize" | "isize" | "char" => {
                        Err(format!("Type '{}' is not supported for bit manipulation", type_name))
                    }
                    _ => generate_custom_type_unpack(field_type, ranges),
                }
            } else {
                generate_custom_type_unpack(field_type, ranges)
            }
        }
        Type::Array(array_type) => generate_array_unpack(array_type, ranges),
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not supported for bit manipulation".to_string()),
    }
}

/// Generate bool unpack expression
fn generate_bool_unpack(ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit = ranges[0].start;
    Ok(quote! { src_bits[#bit] })
}

/// Generate unsigned integer unpack expression
fn generate_unsigned_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_bytes = type_bits / 8;
    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        {
            let mut dst = [0u8; #type_bytes];
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #copy_ranges
            #type_ident::from_be_bytes(dst)
        }
    })
}

/// Generate signed integer unpack expression
fn generate_signed_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_bytes = type_bits / 8;
    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    let sign_bit = ranges[0].start;

    // To do sign extension we initialize the array with the sign bit
    Ok(quote! {
        {
            let mut dst;
            if src_bits[#sign_bit] {
                dst = [0xffu8; #type_bytes];
            }  else {
                dst = [0u8; #type_bytes];
            }
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #copy_ranges
            #type_ident::from_be_bytes(dst)
        }
    })
}

/// Generate float unpack expression
fn generate_float_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(format!(
            "Float type {} requires exactly {} bits, but {} bits were provided",
            type_name, type_bits, total_bits
        ));
    }

    let type_bytes = type_bits / 8;

    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        {
            let mut dst = [0u8; #type_bytes];
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #copy_ranges
            #type_ident::from_be_bytes(dst)
        }
    })
}

/// Generate array unpack expression
fn generate_array_unpack(array_type: &syn::TypeArray, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let element_type = &array_type.elem;
    let array_len = if let syn::Expr::Lit(syn::ExprLit {
        lit: syn::Lit::Int(lit_int),
        ..
    }) = &array_type.len
    {
        lit_int.base10_parse::<usize>().map_err(|e| e.to_string())?
    } else {
        return Err("Array length must be a literal integer".to_string());
    };

    let element_bits = get_element_bits(element_type)?;
    let total_expected_bits = element_bits * array_len;
    let total_actual_bits: usize = ranges.iter().map(NormalizedRange::size).sum();

    if total_actual_bits != total_expected_bits {
        return Err(format!(
            "This array requires {} bits but {} bits were provided",
            total_expected_bits, total_actual_bits
        ));
    }

    // Generate unpack statements for each array element
    let element_unpacks = ranges
        .iter()
        .chunks_exact_bits(element_bits)
        .map(|element_ranges| generate_unpack_expression(element_type, &element_ranges))
        .collect::<Result<Vec<_>, _>>()?;

    Ok(quote! {
        [#(#element_unpacks,)*]
    })
}

/// Generate array unpack expression
fn generate_custom_type_unpack(custom_type: &Type, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let size_error = format!(
        "The type {} requires exactly {{BITS}} bits, but {} were provided",
        custom_type.to_token_stream(),
        total_bits
    );

    let type_name = match total_bits {
        1..=8 => "u8",
        9..=16 => "u16",
        17..=32 => "u32",
        33..=64 => "u64",
        65..=128 => "u128",
        _ => {
            return Err(format!(
                "Custom types occupying {} bits are not supported. It needs to have at most 128 bits.",
                total_bits
            ));
        }
    };

    let type_ident = format_ident!("{}", type_name);
    let type_bits = get_type_bits(type_name);
    let type_bytes = type_bits / 8;
    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        {
            const BITS: usize = <#custom_type as embedded_interfaces::packable::UnsignedPackable>::BITS;
            embedded_interfaces::const_format::assertcp_eq!(BITS, #total_bits, #size_error);

            let mut dst = [0u8; #type_bytes];
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #copy_ranges
            let value = #type_ident::from_be_bytes(dst);
            <#custom_type as embedded_interfaces::packable::UnsignedPackable>::from_unsigned(value.into())
        }
    })
}
