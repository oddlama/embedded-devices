//! Code generation for bit manipulation
//!
//! This module extends the packed struct generation to include the actual
//! bit manipulation logic for converting between packed bytes and unpacked fields.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{Ident, Type};

use super::bit_pattern::{NormalizedRange, ProcessedField};

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

        let pack_statements = generate_pack_statements(field_name, field_type, ranges)?;
        field_packings.extend(pack_statements);
    }

    Ok(quote! {
        let mut bytes = [0u8; #size];
        #(#field_packings)*
        #packed_name(bytes)
    })
}

/// Generate an expression to unpack a field from bytes
pub fn generate_unpack_expression(field_type: &Type, ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
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
                        return Err(format!("Type '{}' is not supported for bit manipulation", type_name));
                    }
                    _ => {
                        return Err(format!("Unknown primitive type '{}' for bit manipulation", type_name));
                    }
                }
            } else {
                Err("Complex path types are not yet supported for bit manipulation".to_string())
            }
        }
        Type::Array(array_type) => generate_array_unpack(array_type, ranges),
        Type::Tuple(_) => Err("Tuple types are not supported for bit manipulation".to_string()),
        _ => Err("This type is not yet supported for bit manipulation".to_string()),
    }
}

/// Generate statements to pack a field into bytes
pub fn generate_pack_statements(
    field_name: &Ident,
    field_type: &Type,
    ranges: &[NormalizedRange],
) -> syn::Result<Vec<TokenStream2>> {
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
                        return Err(syn::Error::new_spanned(
                            field_type,
                            format!("Type '{}' is not supported for bit manipulation", type_name),
                        ));
                    }
                    _ => {
                        return Err(syn::Error::new_spanned(
                            field_type,
                            format!("Unknown primitive type '{}' for bit manipulation", type_name),
                        ));
                    }
                }
            } else {
                Err(syn::Error::new_spanned(
                    field_type,
                    "Complex path types are not yet supported for bit manipulation",
                ))
            }
        }
        Type::Array(array_type) => generate_array_pack(field_name, array_type, ranges),
        Type::Tuple(_) => Err(syn::Error::new_spanned(
            field_type,
            "Tuple types are not supported for bit manipulation",
        )),
        _ => Err(syn::Error::new_spanned(
            field_type,
            "This type is not yet supported for bit manipulation",
        )),
    }
}

/// Generate bool unpack expression
fn generate_bool_unpack(ranges: &[NormalizedRange]) -> Result<TokenStream2, String> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err("bool fields must have exactly 1 bit".to_string());
    }

    let bit_offset = ranges[0].start;
    let byte_index = bit_offset / 8;
    let bit_index = bit_offset % 8;

    Ok(quote! {
        (bytes[#byte_index] & (1 << (7 - #bit_index))) != 0
    })
}

/// Generate bool pack statements
fn generate_bool_pack(field_name: &Ident, ranges: &[NormalizedRange]) -> syn::Result<Vec<TokenStream2>> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err(syn::Error::new_spanned(
            field_name,
            "bool fields must have exactly 1 bit",
        ));
    }

    let bit_offset = ranges[0].start;
    let byte_index = bit_offset / 8;
    let bit_index = bit_offset % 8;

    Ok(vec![quote! {
        if #field_name {
            bytes[#byte_index] |= 1 << (7 - #bit_index);
        }
    }])
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
    let sign_byte_index = sign_bit_offset / 8;
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
        let element_unpack = generate_unpack_expression(element_type, &element_ranges)?;
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
            let start_byte = range.start / 8;
            let end_byte = start_byte + type_bytes;

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
            let source_byte = source_bit / 8;
            let source_bit_in_byte = source_bit % 8;

            extractions.push(quote! {
                if (bytes[#source_byte] & (1 << (7 - #source_bit_in_byte))) != 0 {
                    let target_byte = bit_pos / 8;
                    let target_bit = bit_pos % 8;
                    temp_bytes[target_byte as usize] |= 1 << (7 - target_bit);
                }
                bit_pos += 1;
            });
        }
    }

    Ok(extractions)
}

/// Generate unsigned integer pack statements
fn generate_unsigned_pack(
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<Vec<TokenStream2>> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "Field requires {} bits but type {} can only hold {} bits",
                total_bits, type_name, type_bits
            ),
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
) -> syn::Result<Vec<TokenStream2>> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits > type_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "Field requires {} bits but type {} can only hold {} bits",
                total_bits, type_name, type_bits
            ),
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
) -> syn::Result<Vec<TokenStream2>> {
    let total_bits: u32 = ranges.iter().map(|r| r.size()).sum();
    let type_bits = get_type_bits(type_name);

    if total_bits != type_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "Float type {} requires exactly {} bits, but {} bits were provided",
                type_name, type_bits, total_bits
            ),
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
) -> syn::Result<Vec<TokenStream2>> {
    let element_type = &array_type.elem;
    let array_len = if let syn::Expr::Lit(syn::ExprLit {
        lit: syn::Lit::Int(lit_int),
        ..
    }) = &array_type.len
    {
        lit_int.base10_parse::<usize>()?
    } else {
        return Err(syn::Error::new_spanned(
            field_name,
            "Array length must be a literal integer",
        ));
    };

    let element_bits = get_element_bits(element_type).map_err(|e| syn::Error::new_spanned(field_name, e))?;
    let total_expected_bits = element_bits * array_len as u32;
    let total_actual_bits: u32 = ranges.iter().map(|r| r.size()).sum();

    if total_actual_bits != total_expected_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "Array requires {} bits but {} bits were provided",
                total_expected_bits, total_actual_bits
            ),
        ));
    }

    // Generate pack statements for each array element
    let mut statements = Vec::new();
    let mut current_bit_offset = 0u32;

    for i in 0..array_len {
        let element_ranges = extract_element_ranges(ranges, current_bit_offset, element_bits)
            .map_err(|e| syn::Error::new_spanned(field_name, e))?;
        let element_field = syn::Ident::new(&format!("element_{}", i), proc_macro2::Span::call_site());

        // First extract the element
        statements.push(quote! {
            let #element_field = #field_name[#i];
        });

        // Then pack it
        let element_pack_statements = generate_pack_statements(&element_field, element_type, &element_ranges)?;
        statements.extend(element_pack_statements);

        current_bit_offset += element_bits;
    }

    Ok(statements)
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

/// Generate bit insertion statements for general case
fn generate_bit_insertions(field_name: &Ident, ranges: &[NormalizedRange]) -> syn::Result<Vec<TokenStream2>> {
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

/// Get the size in bits for a primitive type
fn get_type_bits(type_name: &str) -> u32 {
    match type_name {
        "bool" => 1,
        "u8" | "i8" => 8,
        "u16" | "i16" => 16,
        "u32" | "i32" | "f32" => 32,
        "u64" | "i64" | "f64" => 64,
        "u128" | "i128" => 128,
        _ => panic!("Unknown type: {}", type_name),
    }
}

/// Get the size in bits for an element type (recursive for arrays)
fn get_element_bits(element_type: &Type) -> Result<u32, String> {
    match element_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                Ok(get_type_bits(&type_name))
            } else {
                Err("Complex path types are not yet supported".to_string())
            }
        }
        Type::Array(array_type) => {
            let inner_bits = get_element_bits(&array_type.elem)?;
            if let syn::Expr::Lit(syn::ExprLit {
                lit: syn::Lit::Int(lit_int),
                ..
            }) = &array_type.len
            {
                let array_len = lit_int.base10_parse::<u32>().map_err(|e| e.to_string())?;
                Ok(inner_bits * array_len)
            } else {
                Err("Array length must be a literal integer".to_string())
            }
        }
        _ => Err("Unsupported element type for bit calculation".to_string()),
    }
}

/// Extract bit ranges for a specific array element
fn extract_element_ranges(
    ranges: &[NormalizedRange],
    start_bit: u32,
    element_bits: u32,
) -> Result<Vec<NormalizedRange>, String> {
    let end_bit = start_bit + element_bits;
    let mut element_ranges = Vec::new();

    for range in ranges {
        // Check if this range overlaps with our element
        if range.start < end_bit && range.end > start_bit {
            let elem_start = range.start.max(start_bit) - start_bit;
            let elem_end = range.end.min(end_bit) - start_bit;

            element_ranges.push(NormalizedRange::new(elem_start, elem_end)?);
        }
    }

    Ok(element_ranges)
}
