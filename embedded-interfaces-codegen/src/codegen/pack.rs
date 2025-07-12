//! Code generation for packing structs into bit-packed format
//!
//! This module generates the pack method that converts from unpacked field structs
//! to packed bytes.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{Ident, Type};

use super::{
    bit_helpers::{extract_element_ranges, get_element_bits, get_type_bits},
    bit_pattern::{NormalizedRange, ProcessedField},
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
        if processed.field.is_reserved() {
            // TODO: reserved fields must be serialized into the byte array! They have defaults!
            continue;
        }

        let value_expr = quote! { self.#field_name };
        let pack_statements = generate_pack_statements(value_expr, field_type, ranges)
            .map_err(|e| syn::Error::new_spanned(field_name, e))?;
        field_packings.extend(pack_statements);
    }

    Ok(quote! {
        let mut bytes = [0u8; #size];
        #(#field_packings)*
        #packed_name(bytes)
    })
}

/// Generate statements to pack a value into bytes
fn generate_pack_statements(
    value_expr: TokenStream2,
    field_type: &Type,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
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
                    _ => Err(format!("Unknown primitive type '{}' for bit manipulation", type_name)),
                }
            } else {
                Err("Complex path types are not supported for bit manipulation".to_string())
            }
        }
        Type::Array(array_type) => generate_array_pack(value_expr, array_type, ranges),
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not supported for bit manipulation".to_string()),
    }
}

/// Generate bool pack statements
fn generate_bool_pack(value_expr: TokenStream2, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit_offset = ranges[0].start;
    let byte_index = (bit_offset / 8) as usize;
    let bit_index = bit_offset % 8;

    Ok(vec![quote! {
        if #value_expr {
            bytes[#byte_index] |= 1 << (7 - #bit_index);
        }
    }])
}

/// Try to generate optimized byte-aligned pack code
fn try_generate_byte_aligned_pack(
    value_expr: TokenStream2,
    ranges: &[NormalizedRange],
    type_bytes: u32,
) -> Option<TokenStream2> {
    // Check if we have exactly one range that is byte-aligned and full-width
    if ranges.len() == 1 {
        let range = &ranges[0];
        if range.start % 8 == 0 && range.size() == type_bytes * 8 {
            let start_byte = (range.start / 8) as usize;
            let end_byte = start_byte + type_bytes as usize;

            return Some(quote! {
                bytes[#start_byte..#end_byte].copy_from_slice(&(#value_expr).to_be_bytes());
            });
        }
    }

    None
}

/// Generate unsigned integer pack statements
fn generate_unsigned_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(value_expr.clone(), ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(value_expr, ranges)?;

    Ok(bit_insertions)
}

/// Generate signed integer pack statements
fn generate_signed_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(value_expr.clone(), ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(value_expr, ranges)?;

    Ok(bit_insertions)
}

/// Generate float pack statements
fn generate_float_pack(
    value_expr: TokenStream2,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(format!(
            "Float type {} requires exactly {} bits, but {} bits were provided",
            type_name, type_bits, total_bits
        ));
    }

    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(value_expr.clone(), ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(value_expr, ranges)?;

    Ok(bit_insertions)
}

/// Generate array pack statements
fn generate_array_pack(
    value_expr: TokenStream2,
    array_type: &syn::TypeArray,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
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
    let total_expected_bits = element_bits * array_len as u32;
    let total_actual_bits: u32 = ranges.iter().map(|r| r.size()).sum();

    if total_actual_bits != total_expected_bits {
        return Err(format!(
            "Array requires {} bits but {} bits were provided",
            total_expected_bits, total_actual_bits
        ));
    }

    // Generate pack statements for each array element
    let mut statements = Vec::new();
    let mut current_bit_offset = 0u32;

    for i in 0..array_len {
        let element_ranges = extract_element_ranges(ranges, current_bit_offset, element_bits)?;

        // Adjust element ranges to absolute positions
        let absolute_ranges: Vec<NormalizedRange> = element_ranges
            .iter()
            .map(|r| NormalizedRange::new(r.start + current_bit_offset, r.end + current_bit_offset).unwrap())
            .collect();

        let element_value_expr = quote! { (#value_expr)[#i] };
        let element_pack_statements = generate_pack_statements(element_value_expr, element_type, &absolute_ranges)?;

        statements.extend(element_pack_statements);
        current_bit_offset += element_bits;
    }

    Ok(statements)
}

/// Generate bit insertion statements for general case
fn generate_bit_insertions(value_expr: TokenStream2, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    let mut insertions = vec![quote! {
        let source_bytes = (#value_expr).to_be_bytes();
        let mut bit_pos = 0u32;
    }];

    for range in ranges {
        let start_bit = range.start;
        let num_bits = range.size();

        for bit_offset in 0..num_bits {
            let target_bit = start_bit + bit_offset;
            let target_byte = (target_bit / 8) as usize;
            let target_bit_in_byte = target_bit % 8;

            insertions.push(quote! {
                {
                    let source_byte = (bit_pos / 8) as usize;
                    let source_bit = bit_pos % 8;
                    if (source_bytes[source_byte] & (1 << (7 - source_bit))) != 0 {
                        bytes[#target_byte] |= 1 << (7 - #target_bit_in_byte);
                    }
                    bit_pos += 1;
                }
            });
        }
    }

    Ok(insertions)
}
