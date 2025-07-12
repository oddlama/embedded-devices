//! Code generation for unpacking bit-packed structs
//!
//! This module generates the unpack method that converts from packed bytes
//! to unpacked field structs.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{Ident, Type};

use super::{
    bit_helpers::{extract_element_ranges, get_element_bits, get_type_bits},
    bit_pattern::{NormalizedRange, ProcessedField},
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
        let bytes = self.0;
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
                    _ => Err(format!("Unknown primitive type '{}' for bit manipulation", type_name)),
                }
            } else {
                Err("Complex path types are not supported for bit manipulation".to_string())
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

    let bit_offset = ranges[0].start;
    let byte_index = (bit_offset / 8) as usize;
    let bit_index = bit_offset % 8;

    Ok(quote! {
        (bytes[#byte_index] & (1 << (7 - #bit_index))) != 0
    })
}

/// Generate unsigned integer unpack expression
fn generate_unsigned_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_ident = syn::Ident::new(type_name, proc_macro2::Span::call_site());
    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_unpack(&type_ident, ranges, type_bytes) {
        return Ok(optimized);
    }

    // General case: extract bits and construct value
    let bit_extractions = generate_bit_extractions(ranges)?;

    Ok(quote! {
        {
            let mut temp_bytes = [0u8; #type_bytes];
            let mut bit_pos = 0u32;

            #(#bit_extractions)*

            #type_ident::from_be_bytes(temp_bytes)
        }
    })
}

/// Generate signed integer unpack expression
fn generate_signed_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(format!(
            "Field requires {} bits but type {} can only hold {} bits",
            total_bits, type_name, type_bits
        ));
    }

    let type_ident = syn::Ident::new(type_name, proc_macro2::Span::call_site());
    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_unpack(&type_ident, ranges, type_bytes) {
        return Ok(optimized);
    }

    // General case: extract bits, handle sign extension
    let bit_extractions = generate_bit_extractions(ranges)?;
    let sign_bit_offset = ranges[0].start;
    let sign_byte_index = (sign_bit_offset / 8) as usize;
    let sign_bit_index = sign_bit_offset % 8;

    Ok(quote! {
        {
            let mut temp_bytes = [0u8; #type_bytes];
            let mut bit_pos = 0u32;

            #(#bit_extractions)*

            let unsigned_val = #type_ident::from_be_bytes(temp_bytes);

            // Sign extend if the sign bit is set
            let sign_bit_set = (bytes[#sign_byte_index] & (1 << (7 - #sign_bit_index))) != 0;
            if sign_bit_set && #total_bits < #type_bits {
                let sign_extension = !0 << #total_bits;
                (unsigned_val as #type_ident) | (sign_extension as #type_ident)
            } else {
                unsigned_val as #type_ident
            }
        }
    })
}

/// Generate float unpack expression
fn generate_float_unpack(type_name: &str, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(format!(
            "Float type {} requires exactly {} bits, but {} bits were provided",
            type_name, type_bits, total_bits
        ));
    }

    let type_ident = syn::Ident::new(type_name, proc_macro2::Span::call_site());
    let type_bytes = type_bits / 8;

    // Check for byte-aligned optimization
    if let Some(optimized) = try_generate_byte_aligned_unpack(&type_ident, ranges, type_bytes) {
        return Ok(optimized);
    }

    // General case: extract bits and construct value
    let bit_extractions = generate_bit_extractions(ranges)?;

    Ok(quote! {
        {
            let mut temp_bytes = [0u8; #type_bytes];
            let mut bit_pos = 0u32;

            #(#bit_extractions)*

            #type_ident::from_be_bytes(temp_bytes)
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
    let total_expected_bits = element_bits * array_len as u32;
    let total_actual_bits: u32 = ranges.iter().map(|r| r.size()).sum();

    if total_actual_bits != total_expected_bits {
        return Err(format!(
            "Array requires {} bits but {} bits were provided",
            total_expected_bits, total_actual_bits
        ));
    }

    // Split ranges into chunks for each array element
    let mut element_unpacks = Vec::new();
    let mut current_bit_offset = 0u32;

    for _ in 0..array_len {
        let element_ranges = extract_element_ranges(ranges, current_bit_offset, element_bits)?;
        let absolute_ranges: Vec<NormalizedRange> = element_ranges
            .iter()
            .map(|r| NormalizedRange::new(r.start + current_bit_offset, r.end + current_bit_offset).unwrap())
            .collect();
        let element_unpack = generate_unpack_expression(element_type, &absolute_ranges)?;
        element_unpacks.push(element_unpack);
        current_bit_offset += element_bits;
    }

    Ok(quote! {
        [#(#element_unpacks,)*]
    })
}

/// Try to generate optimized byte-aligned unpack code
fn try_generate_byte_aligned_unpack(
    type_ident: &syn::Ident,
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
                #type_ident::from_be_bytes(bytes[#start_byte..#end_byte].try_into().unwrap())
            });
        }
    }

    None
}

/// Generate bit extraction statements for general case
fn generate_bit_extractions(ranges: &[NormalizedRange]) -> Result<Vec<TokenStream2>, String> {
    let mut extractions = Vec::new();

    for range in ranges {
        let start_bit = range.start;
        let num_bits = range.size();

        for bit_offset in 0..num_bits {
            let source_bit = start_bit + bit_offset;
            let source_byte = (source_bit / 8) as usize;
            let source_bit_in_byte = source_bit % 8;

            extractions.push(quote! {
                if (bytes[#source_byte] & (1 << (7 - #source_bit_in_byte))) != 0 {
                    let target_byte = (bit_pos / 8) as usize;
                    let target_bit = bit_pos % 8;
                    temp_bytes[target_byte] |= 1 << (7 - target_bit);
                }
                bit_pos += 1;
            });
        }
    }

    Ok(extractions)
}
