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
            continue;
        }

        let pack_statements = generate_pack_statements(field_name, field_type, ranges)
            .map_err(|e| syn::Error::new_spanned(field_name, e))?;
        field_packings.extend(pack_statements);
    }

    Ok(quote! {
        let mut bytes = [0u8; #size];
        #(#field_packings)*
        #packed_name(bytes)
    })
}

/// Generate statements to pack a field into bytes
fn generate_pack_statements(
    field_name: &Ident,
    field_type: &Type,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                match type_name.as_str() {
                    "bool" => generate_bool_pack(field_name, ranges),
                    "u8" | "u16" | "u32" | "u64" | "u128" => generate_unsigned_pack(field_name, &type_name, ranges),
                    "i8" | "i16" | "i32" | "i64" | "i128" => generate_signed_pack(field_name, &type_name, ranges),
                    "f32" | "f64" => generate_float_pack(field_name, &type_name, ranges),
                    "usize" | "isize" | "char" => {
                        Err(format!("Type '{}' is not supported for bit manipulation", type_name))
                    }
                    _ => Err(format!("Unknown primitive type '{}' for bit manipulation", type_name)),
                }
            } else {
                Err("Complex path types are not yet supported for bit manipulation".to_string())
            }
        }
        Type::Array(array_type) => generate_array_pack(field_name, array_type, ranges),
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not yet supported for bit manipulation".to_string()),
    }
}

/// Generate bool pack statements
fn generate_bool_pack(field_name: &Ident, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit_offset = ranges[0].start;
    let byte_index = (bit_offset / 8) as usize;
    let bit_index = bit_offset % 8;

    Ok(vec![quote! {
        if self.#field_name {
            bytes[#byte_index] |= 1 << (7 - #bit_index);
        }
    }])
}

/// Try to generate optimized byte-aligned pack code
fn try_generate_byte_aligned_pack(
    field_name: &Ident,
    ranges: &[NormalizedRange],
    type_bytes: u32,
) -> Option<TokenStream2> {
    // Check if we have exactly one range that is byte-aligned and full-width
    if ranges.len() == 1 {
        let range = &ranges[0];
        if range.start % 8 == 0 && range.size() == type_bytes * 8 {
            let start_byte = range.start / 8;
            let end_byte = start_byte + type_bytes;

            return Some(quote! {
                bytes[#start_byte..#end_byte].copy_from_slice(&#field_name.to_be_bytes());
            });
        }
    }

    None
}

/// Generate unsigned integer pack statements
fn generate_unsigned_pack(
    field_name: &Ident,
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
    if let Some(optimized) = try_generate_byte_aligned_pack(field_name, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(field_name, ranges)?;

    Ok(bit_insertions)
}

/// Generate signed integer pack statements
fn generate_signed_pack(
    field_name: &Ident,
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
    if let Some(optimized) = try_generate_byte_aligned_pack(field_name, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(field_name, ranges)?;

    Ok(bit_insertions)
}

/// Generate float pack statements
fn generate_float_pack(
    field_name: &Ident,
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
    if let Some(optimized) = try_generate_byte_aligned_pack(field_name, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(field_name, ranges)?;

    Ok(bit_insertions)
}

/// Generate array pack statements
fn generate_array_pack(
    field_name: &Ident,
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

        // Pack the element using a scoped variable named 'element'
        let element_pack_statements = generate_pack_statements_for_element(element_type, &element_ranges)?;

        statements.push(quote! {
            {
                let element = self.#field_name[#i];
                #(#element_pack_statements)*
            }
        });

        current_bit_offset += element_bits;
    }

    Ok(statements)
}

/// Generate pack statements for an array element (using 'element' as variable name)
fn generate_pack_statements_for_element(
    element_type: &Type,
    ranges: &[NormalizedRange],
) -> Result<Vec<TokenStream2>, String> {
    match element_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                match type_name.as_str() {
                    "bool" => generate_bool_pack_for_element(ranges),
                    "u8" | "u16" | "u32" | "u64" | "u128" => generate_unsigned_pack_for_element(&type_name, ranges),
                    "i8" | "i16" | "i32" | "i64" | "i128" => generate_signed_pack_for_element(&type_name, ranges),
                    "f32" | "f64" => generate_float_pack_for_element(&type_name, ranges),
                    "usize" | "isize" | "char" => {
                        Err(format!("Type '{}' is not supported for bit manipulation", type_name))
                    }
                    _ => Err(format!("Unknown primitive type '{}' for bit manipulation", type_name)),
                }
            } else {
                Err("Complex path types are not yet supported for bit manipulation".to_string())
            }
        }
        Type::Array(_) => {
            // For nested arrays, we would need to recurse, but for now return an error
            Err("Nested arrays are not yet supported".to_string())
        }
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not yet supported for bit manipulation".to_string()),
    }
}

/// Generate bool pack statements for element
fn generate_bool_pack_for_element(ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit_offset = ranges[0].start;
    let byte_index = (bit_offset / 8) as usize;
    let bit_index = bit_offset % 8;

    Ok(vec![quote! {
        if element {
            bytes[#byte_index] |= 1 << (7 - #bit_index);
        }
    }])
}

/// Generate unsigned integer pack statements for element
fn generate_unsigned_pack_for_element(
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
    let element_ident = syn::Ident::new("element", proc_macro2::Span::call_site());

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(&element_ident, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(&element_ident, ranges)?;

    Ok(bit_insertions)
}

/// Generate signed integer pack statements for element
fn generate_signed_pack_for_element(type_name: &str, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_bytes = type_bits / 8;
    let element_ident = syn::Ident::new("element", proc_macro2::Span::call_site());

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(&element_ident, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(&element_ident, ranges)?;

    Ok(bit_insertions)
}

/// Generate float pack statements for element
fn generate_float_pack_for_element(type_name: &str, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(format!(
            "Float type {} requires exactly {} bits, but {} bits were provided",
            type_name, type_bits, total_bits
        ));
    }

    let type_bytes = type_bits / 8;
    let element_ident = syn::Ident::new("element", proc_macro2::Span::call_site());

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_pack(&element_ident, ranges, type_bytes) {
        return Ok(vec![optimized]);
    }

    // General case: convert to bytes and extract bits
    let bit_insertions = generate_bit_insertions(&element_ident, ranges)?;

    Ok(bit_insertions)
}

/// Generate bit insertion statements for general case
fn generate_bit_insertions(field_name: &Ident, ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    let mut insertions = vec![quote! {
        let source_bytes = #field_name.to_be_bytes();
        let mut bit_pos = 0u32;
    }];

    for range in ranges {
        let start_bit = range.start;
        let num_bits = range.size();

        for bit_offset in 0..num_bits {
            let target_bit = start_bit + bit_offset;
            let target_byte = target_bit / 8;
            let target_bit_in_byte = target_bit % 8;

            insertions.push(quote! {
                let source_byte = bit_pos / 8;
                let source_bit = bit_pos % 8;
                if (source_bytes[source_byte as usize] & (1 << (7 - source_bit))) != 0 {
                    bytes[#target_byte] |= 1 << (7 - #target_bit_in_byte);
                }
                bit_pos += 1;
            });
        }
    }

    Ok(insertions)
}
