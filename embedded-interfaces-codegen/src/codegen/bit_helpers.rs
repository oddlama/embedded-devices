use syn::Type;

use super::bit_pattern::NormalizedRange;

/// Get the size in bits for a primitive type
pub fn get_type_bits(type_name: &str) -> u32 {
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
pub fn get_element_bits(element_type: &Type) -> Result<u32, String> {
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
pub fn extract_element_ranges(
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
