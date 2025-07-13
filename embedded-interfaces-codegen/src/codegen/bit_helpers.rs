use proc_macro2::TokenStream;
use quote::quote;
use syn::{Ident, Type};

use super::bit_pattern::NormalizedRange;

/// Get the size in bits for a primitive type
pub fn get_type_bits(type_name: &str) -> usize {
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
pub fn get_element_bits(element_type: &Type) -> Result<usize, String> {
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
                let array_len = lit_int.base10_parse::<usize>().map_err(|e| e.to_string())?;
                Ok(inner_bits * array_len)
            } else {
                Err("Array length must be a literal integer".to_string())
            }
        }
        _ => Err("Unsupported element type for bit calculation".to_string()),
    }
}

/// Generates code that copies a range of bits from the source to the destination.
///
/// # Arguments
/// * `src_ident` - The identifier for the source byte array
/// * `dst_ident` - The identifier for the destination byte array
/// * `src_range` - The bit range in the source (start..end)
/// * `dst_range` - The bit range in the destination (start..end)
pub fn generate_bit_range_copy(
    src_ident: &Ident,
    dst_ident: &Ident,
    (src_start, src_end): (usize, usize),
    (dst_start, dst_end): (usize, usize),
) -> TokenStream {
    quote! {{
        use embedded_interfaces::bitvec::{order::Msb0, view::BitView};
        let src_bits = #src_ident.view_bits::<Msb0>();
        let dst_bits = #dst_ident.view_bits_mut::<Msb0>();
        let src_range = &src_bits[#src_start..#src_end];
        let dst_range = &mut dst_bits[#dst_start..#dst_end];
        dst_range.copy_from_bitslice(src_range);
    }}
}

pub fn generate_copy_from_normalized_ranges(
    type_bits: usize,
    total_bits: usize,
    src_ident: &Ident,
    dst_ident: &Ident,
    ranges: &[NormalizedRange],
) -> Result<TokenStream, String> {
    let mut statements = vec![];

    let dst_range_start = type_bits - total_bits;
    let mut bits_copied = 0usize;
    for range in ranges {
        let start = dst_range_start + bits_copied;
        statements.push(generate_bit_range_copy(
            src_ident,
            dst_ident,
            (range.start, range.end),
            (start, start + range.size()),
        ));
        bits_copied += range.size();
    }

    Ok(quote! {{ #(#statements)* }})
}

pub fn generate_copy_to_normalized_ranges(
    type_bits: usize,
    total_bits: usize,
    src_ident: &Ident,
    dst_ident: &Ident,
    ranges: &[NormalizedRange],
) -> Result<TokenStream, String> {
    let mut statements = vec![];

    let src_range_start = type_bits - total_bits;
    let mut bits_copied = 0usize;
    for range in ranges {
        let start = src_range_start + bits_copied;
        statements.push(generate_bit_range_copy(
            src_ident,
            dst_ident,
            (start, start + range.size()),
            (range.start, range.end),
        ));
        bits_copied += range.size();
    }

    Ok(quote! {{ #(#statements)* }})
}
