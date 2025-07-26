//! Code generation for unpacking bit-packed structs
//!
//! This module generates the unpack method that converts from packed bytes
//! to unpacked field structs.

use proc_macro2::TokenStream as TokenStream2;
use quote::{ToTokens, format_ident, quote};
use syn::{Ident, Type};

use crate::parser::{Definition, InterfaceObjectsDefinition, StructDefinition};

use super::{
    bit_helpers::{generate_copy_from_normalized_ranges, get_element_bits, get_type_bits},
    bit_pattern::{IterChunksExactBits, NormalizedRange, ProcessedField, extract_bits, process_field_bit_patterns},
    extract_struct_size,
};

/// Generate the body of the unpack method
pub fn generate_unpack_body(
    interface_def: &InterfaceObjectsDefinition,
    unpacked_name: &Ident,
    processed_fields: &[ProcessedField],
) -> syn::Result<TokenStream2> {
    let mut field_extractions: Vec<TokenStream2> = Vec::new();

    for processed in processed_fields {
        let field_name = &processed.field.name;
        let field_type = &processed.field.field_type;
        let ranges = &processed.normalized_ranges;

        // Skip reserved fields
        if processed.field.is_reserved() {
            continue;
        }

        let unpack_expr = generate_unpack_expression(interface_def, field_name, field_type, ranges)?;
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

/// Generate reading accessors for the packed representation
pub fn generate_accessors(
    interface_def: &InterfaceObjectsDefinition,
    unpacked_name: &Ident,
    processed_field: &ProcessedField,
    ranges: &[NormalizedRange],
    prefix: &str,
) -> syn::Result<TokenStream2> {
    let field_name = &processed_field.field.name;
    let field_type = &processed_field.field.field_type;

    let nested_accessors = match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                if let Some(packed_name) = type_name.strip_suffix("Unpacked") {
                    if let Some(custom_type) = interface_def.get_definition(packed_name) {
                        let struct_def = match custom_type {
                            Definition::Register(register_definition) => Some(register_definition.clone().into()),
                            Definition::Struct(struct_definition) => Some(struct_definition.clone()),
                            Definition::Enum(_) => None,
                        };

                        let packed_type = format_ident!("{}", packed_name);
                        struct_def
                            .map(|struct_def| -> syn::Result<_> {
                                // Get effective attributes (only size is allowed for structs)
                                let attrs = interface_def.get_effective_struct_attrs(&struct_def)?;
                                // Extract size attribute
                                let size = extract_struct_size(&struct_def.name, &attrs)?;
                                let type_bits = size * 8;

                                let mut sub_accessors: Vec<TokenStream2> = Vec::new();
                                let sub_processed_fields = process_field_bit_patterns(
                                    interface_def,
                                    &packed_type,
                                    &struct_def.fields,
                                    type_bits,
                                )?;
                                for sub_processed in sub_processed_fields {
                                    let sub_field_ranges = &sub_processed.normalized_ranges;
                                    let ranges = extract_bits(ranges, sub_field_ranges);
                                    // Skip reserved fields
                                    if sub_processed.field.is_reserved() {
                                        continue;
                                    }

                                    sub_accessors.push(generate_accessors(
                                        interface_def,
                                        unpacked_name,
                                        &sub_processed,
                                        &ranges,
                                        &format!("{}{}_", prefix, field_name),
                                    )?);
                                }

                                // Packed accessor
                                let read_fn_name = format_ident!("read_{}{}", prefix, field_name);
                                let read_fn_doc = format!("Unpacks the nested packed struct [`{field_name}`]({unpacked_name}::{field_name}) from this packed representation.");
                                let copy_ranges = generate_copy_from_normalized_ranges(
                                    type_bits,
                                    type_bits,
                                    &format_ident!("src_bits"),
                                    &format_ident!("dst_bits"),
                                    ranges,
                                )
                                .map_err(|e| syn::Error::new_spanned(field_name, e))?;

                                let packed_accessor = quote! {
                                    #[doc = #read_fn_doc]
                                    pub fn #read_fn_name(&self) -> #packed_type {
                                        use embedded_interfaces::bitvec::{order::Msb0, view::BitView};
                                        let src = self.0;
                                        let src_bits = src.view_bits::<Msb0>();
                                        let mut dst = [0u8; #size];
                                        let dst_bits = dst.view_bits_mut::<Msb0>();
                                        #copy_ranges
                                        #packed_type(dst)
                                    }
                                };

                                // Unpacked accessor
                                let unpack_expr =
                                    generate_unpack_expression(interface_def, field_name, field_type, ranges)?;
                                let read_fn_name = format_ident!("read_{}{}_unpacked", prefix, field_name);
                                let read_fn_doc = format!("Fully unpacks the nested struct [`{field_name}`]({unpacked_name}::{field_name}) from this packed representation.");

                                let unpacked_accessor = quote! {
                                    #[doc = #read_fn_doc]
                                    pub fn #read_fn_name(&self) -> #field_type {
                                        use embedded_interfaces::bitvec::{order::Msb0, view::BitView};
                                        let src = self.0;
                                        let src_bits = src.view_bits::<Msb0>();
                                        #unpack_expr
                                    }
                                };

                                // Result
                                Ok(quote! {
                                    #packed_accessor
                                    #unpacked_accessor

                                    #(#sub_accessors)*
                                })
                            })
                            .transpose()?
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                None
            }
        }
        _ => None,
    };

    if let Some(accessors) = nested_accessors {
        Ok(accessors)
    } else {
        let unpack_expr = generate_unpack_expression(interface_def, field_name, field_type, ranges)?;
        let read_fn_name = format_ident!("read_{}{}", prefix, field_name);
        let read_fn_doc = format!(
            "Unpacks only the [`{field_name}`]({unpacked_name}::{field_name}) field from this packed representation."
        );

        Ok(quote! {
            #[doc = #read_fn_doc]
            pub fn #read_fn_name(&self) -> #field_type {
                use embedded_interfaces::bitvec::{order::Msb0, view::BitView};
                let src = self.0;
                let src_bits = src.view_bits::<Msb0>();
                #unpack_expr
            }
        })
    }
}

/// Generate an expression to unpack a field from bytes
fn generate_unpack_expression(
    interface_def: &InterfaceObjectsDefinition,
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
                        Definition::Enum(_) => generate_custom_type_unpack(field_name, field_type, ranges),
                    }
                } else if let Some(custom_type) = type_name
                    .strip_suffix("Unpacked")
                    .and_then(|x| interface_def.get_definition(x))
                {
                    match custom_type {
                        Definition::Register(register_definition) => generate_struct_unpack(
                            interface_def,
                            &register_definition.clone().into(),
                            field_name,
                            ranges,
                        ),
                        Definition::Struct(struct_definition) => {
                            generate_struct_unpack(interface_def, struct_definition, field_name, ranges)
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
                        "bool" => generate_bool_unpack(field_name, ranges),
                        "u8" | "u16" | "u32" | "u64" | "u128" => {
                            generate_unsigned_unpack(field_name, &type_name, ranges)
                        }
                        "i8" | "i16" | "i32" | "i64" | "i128" => generate_signed_unpack(field_name, &type_name, ranges),
                        "f32" | "f64" => generate_float_unpack(field_name, &type_name, ranges),
                        "usize" | "isize" | "char" => Err(syn::Error::new_spanned(
                            field_name,
                            format!("Type '{}' is not supported for bit manipulation", type_name),
                        )),
                        _ => generate_custom_type_unpack(field_name, field_type, ranges),
                    }
                }
            } else {
                generate_custom_type_unpack(field_name, field_type, ranges)
            }
        }
        Type::Array(array_type) => generate_array_unpack(interface_def, field_name, array_type, ranges),
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

/// Generate bool unpack expression
fn generate_bool_unpack(field_name: &Ident, ranges: &[NormalizedRange]) -> syn::Result<TokenStream2> {
    if ranges.len() != 1 || ranges[0].size() != 1 {
        return Err(syn::Error::new_spanned(
            field_name,
            "bool fields must have exactly 1 bit".to_string(),
        ));
    }

    let bit = ranges[0].start;
    Ok(quote! { src_bits[#bit] })
}

/// Generate unsigned integer unpack expression
fn generate_unsigned_unpack(
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
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
    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

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
fn generate_signed_unpack(
    field_name: &Ident,
    type_name: &str,
    ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
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
    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

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
fn generate_float_unpack(field_name: &Ident, type_name: &str, ranges: &[NormalizedRange]) -> syn::Result<TokenStream2> {
    let total_bits: usize = ranges.iter().map(NormalizedRange::size).sum();
    let type_ident = format_ident!("{}", type_name);
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

    let copy_ranges = generate_copy_from_normalized_ranges(
        type_bits,
        total_bits,
        &format_ident!("src_bits"),
        &format_ident!("dst_bits"),
        ranges,
    )
    .map_err(|e| syn::Error::new_spanned(field_name, e))?;

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
fn generate_array_unpack(
    interface_def: &InterfaceObjectsDefinition,
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

    // Generate unpack statements for each array element
    let element_unpacks = ranges
        .iter()
        .chunks_exact_bits(element_bits)
        .map(|element_ranges| {
            generate_unpack_expression(interface_def, field_name, element_type, &element_ranges)
                .map_err(|e| syn::Error::new_spanned(field_name, e))
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(quote! {
        [#(#element_unpacks,)*]
    })
}

/// Unpacks any packable struct defined in the same interface scope
fn generate_struct_unpack(
    interface_def: &InterfaceObjectsDefinition,
    struct_def: &StructDefinition,
    field_name: &Ident,
    struct_ranges: &[NormalizedRange],
) -> syn::Result<TokenStream2> {
    let packed_name = &struct_def.name;
    let unpacked_name = format_ident!("{}Unpacked", struct_def.name);
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

    let mut field_extractions: Vec<TokenStream2> = Vec::new();
    let processed_fields = process_field_bit_patterns(interface_def, packed_name, &struct_def.fields, type_bits)?;
    for processed in processed_fields {
        let field_name = &processed.field.name;
        let field_type = &processed.field.field_type;
        let field_ranges = &processed.normalized_ranges;
        let ranges = extract_bits(struct_ranges, field_ranges);

        // Skip reserved fields
        if processed.field.is_reserved() {
            continue;
        }

        let unpack_expr = generate_unpack_expression(interface_def, field_name, field_type, &ranges)?;
        field_extractions.push(quote! {
            #field_name: #unpack_expr
        });
    }

    Ok(quote! {
        #unpacked_name {
            #(#field_extractions,)*
        }
    })
}

/// Generate array unpack expression
fn generate_custom_type_unpack(
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
    let type_bytes = type_bits / 8;
    let copy_ranges = generate_copy_from_normalized_ranges(
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

            let mut dst = [0u8; #type_bytes];
            let dst_bits = dst.view_bits_mut::<Msb0>();
            #copy_ranges
            let value = #type_ident::from_be_bytes(dst);
            <#custom_type as embedded_interfaces::packable::UnsignedPackable>::from_unsigned(value.into())
        }
    })
}
