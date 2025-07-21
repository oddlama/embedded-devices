//! Code generation for packing structs into bit-packed format
//!
//! This module generates the pack method that converts from unpacked field structs
//! to packed bytes.

use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::{Ident, Type};

use crate::parser::{Definition, InterfaceObjectsDefinition, StructDefinition};

use super::{
    bit_helpers::{generate_copy_to_normalized_ranges, get_element_bits, get_type_bits},
    bit_pattern::{IterChunksExactBits, NormalizedRange, ProcessedField, extract_bits, process_field_bit_patterns},
};

/// Generate the body of the pack method
pub fn generate_pack_body(
    interface_def: &InterfaceObjectsDefinition,
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

        let pack_statement = generate_pack_statement(interface_def, value_expr, field_name, field_type, ranges)?;
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
    interface_def: &InterfaceObjectsDefinition,
    value_expr: TokenStream2,
    field_name: &Ident,
    field_type: &Type,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                if let Some(custom_type) = interface_def.get_definition(&type_name) {
                    match custom_type {
                        Definition::Register(_) => Err(syn::Error::new_spanned(
                            field_name,
                            format!("To nest registers, use use the unpacked form '{type_name}Unpacked'"),
                        )),
                        Definition::Struct(_) => Err(syn::Error::new_spanned(
                            field_name,
                            format!("To nest structs, use use the unpacked form '{type_name}Unpacked'"),
                        )),
                        Definition::Enum(_) => generate_custom_type_pack(value_expr, field_name, field_type, ranges),
                    }
                } else if let Some(custom_type) = type_name
                    .strip_suffix("Unpacked")
                    .and_then(|x| interface_def.get_definition(x))
                {
                    match custom_type {
                        Definition::Register(register_definition) => generate_struct_pack(
                            interface_def,
                            &register_definition.clone().into(),
                            value_expr,
                            field_name,
                            ranges,
                        ),
                        Definition::Struct(struct_definition) => {
                            generate_struct_pack(interface_def, struct_definition, value_expr, field_name, ranges)
                        }
                        Definition::Enum(enum_definition) => Err(syn::Error::new_spanned(
                            field_name,
                            format!(
                                "There is no unpacked type with this name, but an enum called '{}' exists",
                                enum_definition.name
                            ),
                        )),
                    }
                } else {
                    match type_name.as_str() {
                        "bool" => generate_bool_pack(value_expr, field_name, ranges),
                        "u8" | "u16" | "u32" | "u64" | "u128" => {
                            generate_unsigned_pack(value_expr, field_name, &type_name, ranges)
                        }
                        "i8" | "i16" | "i32" | "i64" | "i128" => {
                            generate_signed_pack(value_expr, field_name, &type_name, ranges)
                        }
                        "f32" | "f64" => generate_float_pack(value_expr, field_name, &type_name, ranges),
                        "usize" | "isize" | "char" => Err(syn::Error::new_spanned(
                            field_name,
                            format!("Type '{}' is not supported for bit manipulation", type_name),
                        )),
                        _ => generate_custom_type_pack(value_expr, field_name, field_type, ranges),
                    }
                }
            } else {
                generate_custom_type_pack(value_expr, field_name, field_type, ranges)
            }
        }
        Type::Array(array_type) => generate_array_pack(interface_def, value_expr, field_name, array_type, ranges),
        Type::Tuple(_) => Err(syn::Error::new_spanned(
            field_name,
            "Tuple types are not supported for bit manipulation".to_string(),
        )),
        _ => Err(syn::Error::new_spanned(
            field_name,
            "This type is not supported for bit manipulation".to_string(),
        )),
    }
}

/// Generate bool pack statements
fn generate_bool_pack(
    value_expr: TokenStream2,
    field_name: &Ident,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err(syn::Error::new_spanned(
            field_name,
            "bool fields must have exactly 1 bit".to_string(),
        ));
    }

    let bit = ranges[0].start;
    Ok(quote! {{
        dst_bits.replace(#bit, (#value_expr) as bool);
    }})
}

/// Generate unsigned integer pack statements
fn generate_unsigned_pack(
    value_expr: TokenStream2,
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
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

    let copy_ranges = generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

    Ok(quote! {
        let src = #value_expr.to_be_bytes();
        let src_bits = src.view_bits::<Msb0>();
        #copy_ranges
    })
}

/// Generate signed integer pack statements
fn generate_signed_pack(
    value_expr: TokenStream2,
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
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

    statements.push(
        generate_copy_to_normalized_ranges(
            type_bits,
            total_bits,
            &format_ident!("src_bits"),
            &format_ident!("dst_bits"),
            ranges,
        )
        .map_err(|e| syn::Error::new_spanned(field_name, e))?,
    );

    Ok(quote! {{ #(#statements)* }})
}

/// Generate float pack statements
fn generate_float_pack(
    value_expr: TokenStream2,
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
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

    let copy_ranges = generate_copy_to_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

    Ok(quote! {
        let src = #value_expr.to_be_bytes();
        let src_bits = src.view_bits::<Msb0>();
        #copy_ranges
    })
}

/// Generate array pack statements
fn generate_array_pack(
    interface_def: &InterfaceObjectsDefinition,
    value_expr: TokenStream2,
    field_name: &Ident,
    array_type: &syn::TypeArray,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
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
            "Array length must be a literal integer".to_string(),
        ));
    };

    let element_bits = get_element_bits(element_type).map_err(|e| syn::Error::new_spanned(field_name, e))?;
    let total_expected_bits = element_bits * array_len;
    let total_actual_bits: usize = ranges.iter().map(NormalizedRange::size).sum();

    if total_actual_bits != total_expected_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "This array requires {} bits but {} bits were provided",
                total_expected_bits, total_actual_bits
            ),
        ));
    }

    // Generate pack statements for each array element
    let element_pack_statements = ranges
        .iter()
        .chunks_exact_bits(element_bits)
        .enumerate()
        .map(|(i, element_ranges)| {
            let element_value_expr = quote! { (#value_expr)[#i] };
            generate_pack_statement(
                interface_def,
                element_value_expr,
                field_name,
                element_type,
                &element_ranges,
            )
            .map_err(|e| syn::Error::new_spanned(field_name, e))
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(quote! { #({ #element_pack_statements })* })
}

/// Packs any packable struct defined in the same interface scope
fn generate_struct_pack(
    interface_def: &InterfaceObjectsDefinition,
    struct_def: &StructDefinition,
    struct_value_expr: TokenStream2,
    field_name: &Ident,
    struct_ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let packed_name = &struct_def.name;
    // Get effective attributes (only size is allowed for structs)
    let attrs = interface_def.get_effective_struct_attrs(struct_def)?;
    // Extract size attribute
    let size = super::extract_struct_size(&struct_def.name, &attrs)?;
    let type_bits = size * 8;

    let total_bits: usize = struct_ranges.iter().map(NormalizedRange::size).sum();
    if total_bits != type_bits {
        return Err(syn::Error::new_spanned(
            field_name,
            format!(
                "Struct type {} requires exactly {} bits, but {} bits were provided",
                packed_name, type_bits, total_bits
            ),
        ));
    }

    let mut field_packings: Vec<TokenStream2> = Vec::new();
    let processed_fields = process_field_bit_patterns(interface_def, packed_name, &struct_def.fields, type_bits)?;
    for processed in processed_fields {
        let field_name = &processed.field.name;
        let field_type = &processed.field.field_type;
        let field_ranges = &processed.normalized_ranges;
        let ranges = extract_bits(struct_ranges, field_ranges);

        // Skip reserved fields
        let value_expr = if processed.field.is_reserved() {
            if let Some(default) = processed.field.default_value.as_ref() {
                quote! { { let default: #field_type = #default; default } }
            } else {
                // Reserved values without defaults will never be altered.
                continue;
            }
        } else {
            quote! { #struct_value_expr.#field_name }
        };

        let pack_statement = generate_pack_statement(interface_def, value_expr, field_name, field_type, &ranges)?;
        field_packings.push(pack_statement);
    }

    Ok(quote! {
        { #(#field_packings)* }
    })
}

/// Generate custom type pack statements
fn generate_custom_type_pack(
    value_expr: TokenStream2,
    field_name: &Ident,
    custom_type: &Type,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
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
            return Err(syn::Error::new_spanned(
                field_name,
                format!(
                    "Custom types occupying {} bits are not supported. It needs to have at most 128 bits.",
                    total_bits
                ),
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
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

    Ok(quote! {
        {
            const BITS: usize = <#custom_type as embedded_interfaces::packable::UnsignedPackable>::BITS;
            embedded_interfaces::const_format::assertcp_eq!(BITS, #total_bits, #size_error);

            let src = <#custom_type as embedded_interfaces::packable::UnsignedPackable>::to_unsigned(&(#value_expr)) as #type_ident;
            let src = src.to_be_bytes();
            let src_bits = src.view_bits::<Msb0>();
            #copy_ranges
        }
    })
}
