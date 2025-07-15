//! Code generation for packing structs into bit-packed format
//!
//! This module generates the pack method that converts from unpacked field structs
//! to packed bytes.

use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::{Ident, Type};

use super::{
    bit_helpers::{generate_copy_to_normalized_ranges, get_element_bits, get_type_bits},
    bit_pattern::{IterChunksExactBits, NormalizedRange, ProcessedField},
};

/// Generate the body of the pack method
pub fn generate_pack_body(
    packed_name: &Ident,
    processed_fields: &[ProcessedField],
    size: usize,
) -> syn::Result<TokenStream2> {
    let mut field_packings: Vec<TokenStream2> = Vec::new();

    for processed in processed_fields {
        let field_name = &processed.field.name;
        let field_type = &processed.field.field_type;
        let ranges = &processed.normalized_ranges;

        // Skip reserved fields
        let value_expr = if processed.field.is_reserved() {
            if let Some(default) = processed.field.default_value.as_ref() {
                quote! { { let default: #field_type = #default; default } }
            } else {
                // Reserved values without defaults will never be altered.
                continue;
            }
        } else {
            quote! { self.#field_name }
        };

        let pack_statement = generate_pack_statement(value_expr, field_type, ranges)
            .map_err(|e| syn::Error::new_spanned(field_name, e))?;
        field_packings.push(pack_statement);
    }

    Ok(quote! {
        use embedded_interfaces::bitvec::{order::Msb0, view::BitView};

        let mut dst = [0u8; #size];
        {
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #(#field_packings)*
        }
        #packed_name(dst)
    })
}

/// Generate statements to pack a value into bytes
fn generate_pack_statement(
    value_expr: TokenStream2,
    field_type: &Type,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                match type_name.as_str() {
                    "bool" => generate_bool_pack(value_expr, ranges),
                    "u8" | "u16" | "u32" | "u64" | "u128" => generate_unsigned_pack(value_expr, &type_name, ranges),
                    "i8" | "i16" | "i32" | "i64" | "i128" => generate_signed_pack(value_expr, &type_name, ranges),
                    "f32" | "f64" => generate_float_pack(value_expr, &type_name, ranges),
                    "usize" | "isize" | "char" => {
                        Err(format!("Type '{}' is not supported for bit manipulation", type_name))
                    }
                    _ => generate_custom_type_pack(value_expr, field_type, ranges),
                }
            } else {
                generate_custom_type_pack(value_expr, field_type, ranges)
            }
        }
        Type::Array(array_type) => generate_array_pack(value_expr, array_type, ranges),
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not supported for bit manipulation".to_string()),
    }
}

/// Generate bool pack statements
fn generate_bool_pack(value_expr: TokenStream2, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit = ranges[0].start;
    Ok(quote! {{
        dst_bits.replace(#bit, (#value_expr) as bool);
    }})
}

/// Generate unsigned integer pack statements
fn generate_unsigned_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let copy_ranges = generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        let src = #value_expr.to_be_bytes();
        let src_bits = src.view_bits::<Msb0>();
        #copy_ranges
    })
}

/// Generate signed integer pack statements
fn generate_signed_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let mut statements = vec![];
    if total_bits == type_bits {
        // We copy the full type so the sign bit is already at the correct position
        statements.push(quote! {
            let src = #value_expr.to_be_bytes();
            let src_bits = src.view_bits::<Msb0>();
        });
    } else {
        // Sign bit must be moved to another location
        let sign_byte_index = (type_bits - total_bits) / 8;
        let sign_bit_in_byte_index = 7 - ((type_bits - total_bits) % 8);

        statements.push(quote! {
            let mut src = #value_expr.to_be_bytes();
            src[#sign_byte_index] |= (src[0] >> 7) << #sign_bit_in_byte_index;
            let src_bits = src.view_bits::<Msb0>();
        });
    }

    statements.push(generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?);

    Ok(quote! {{ #(#statements)* }})
}

/// Generate float pack statements
fn generate_float_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(format!(
            "Float type {} requires exactly {} bits, but {} bits were provided",
            type_name, type_bits, total_bits
        ));
    }

    let copy_ranges = generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        let src = #value_expr.to_be_bytes();
        let src_bits = src.view_bits::<Msb0>();
        #copy_ranges
    })
}

/// Generate array pack statements
fn generate_array_pack(
    value_expr: TokenStream2,
    array_type: &syn::TypeArray,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
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

    // Generate pack statements for each array element
    let element_pack_statements = ranges
        .iter()
        .chunks_exact_bits(element_bits)
        .enumerate()
        .map(|(i, element_ranges)| {
            let element_value_expr = quote! { (#value_expr)[#i] };
            generate_pack_statement(element_value_expr, element_type, &element_ranges)
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(quote! { #({ #element_pack_statements })* })
}

/// Generate custom type pack statements
fn generate_custom_type_pack(
    value_expr: TokenStream2,
    custom_type: &Type,
    ranges: &[NormalizedRange],
) -> Result<TokenStream2, String> {
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
    let copy_ranges = generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )?;

    Ok(quote! {
        const BITS: usize = <#custom_type as embedded_interfaces::packable::UnsignedPackable>::BITS;
        embedded_interfaces::const_format::assertcp_eq!(BITS, #total_bits, #size_error);

        let src = <#custom_type as embedded_interfaces::packable::UnsignedPackable>::to_unsigned(&(#value_expr)) as #type_ident;
        let src = src.to_be_bytes();
        let src_bits = src.view_bits::<Msb0>();
        #copy_ranges
    })
}
