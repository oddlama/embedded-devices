//! Reusable packed/unpacked struct generation
//!
//! This module contains the generic logic for generating packed and unpacked
//! struct pairs that can be reused for registers and other similar structures.

use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{Attribute, Expr, Ident, Type};

use super::bit_pattern::process_field_bit_patterns;
use crate::{
    codegen::{
        bit_pattern::{NormalizedRange, extract_bits},
        extract_struct_size,
    },
    parser::{BitConstraint, Definition, FieldDefinition, InterfaceObjectsDefinition},
};

/// Generate a pair of packed and unpacked structs with conversion implementations
///
/// This function is generic and can be used for registers or any other structures
/// that need packed/unpacked representations.
pub fn generate_packed_struct_pair(
    interface_def: &InterfaceObjectsDefinition,
    packed_name: &Ident,
    unpacked_name: &Ident,
    fields: &[FieldDefinition],
    doc_attrs: &[Attribute],
    size: usize,
) -> syn::Result<TokenStream2> {
    // Process and validate bit patterns
    let total_size_bits = size * 8;
    let processed_fields = process_field_bit_patterns(interface_def, packed_name, fields, total_size_bits)?;

    let unpacked_struct = generate_unpacked_struct(
        interface_def,
        &processed_fields,
        packed_name,
        unpacked_name,
        doc_attrs,
        size,
    )?;
    let packed_struct = generate_packed_struct(
        interface_def,
        &processed_fields,
        packed_name,
        unpacked_name,
        size,
        doc_attrs,
    )?;
    let conversions = generate_struct_conversions(packed_name, unpacked_name)?;

    Ok(quote! {
        #unpacked_struct
        #packed_struct
        #conversions
    })
}

fn generate_bitdump_fields(
    interface_def: &InterfaceObjectsDefinition,
    processed_field: &super::bit_pattern::ProcessedField,
    prefix: String,
    ranges: &[NormalizedRange],
    out_fields: &mut Vec<TokenStream2>,
) -> syn::Result<()> {
    use quote::ToTokens;

    if let Type::Path(type_path) = &processed_field.field.field_type {
        if let Some(ident) = type_path.path.get_ident() {
            let type_name = ident.to_string();
            if let Some(packed_name) = type_name.strip_suffix("Unpacked") {
                if let Some(custom_type) = interface_def.get_definition(packed_name) {
                    let Some(struct_def) = (match custom_type {
                        Definition::Register(register_definition) => Some(register_definition.clone().into()),
                        Definition::Struct(struct_definition) => Some(struct_definition.clone()),
                        Definition::Enum(_) => None,
                    }) else {
                        return Ok(());
                    };

                    let packed_type = format_ident!("{}", packed_name);
                    // Get effective attributes (only size is allowed for structs)
                    let attrs = interface_def.get_effective_struct_attrs(&struct_def)?;
                    // Extract size attribute
                    let size = extract_struct_size(&struct_def.name, &attrs)?;
                    let type_bits = size * 8;

                    let sub_processed_fields =
                        process_field_bit_patterns(interface_def, &packed_type, &struct_def.fields, type_bits)?;
                    for sub_processed in sub_processed_fields {
                        let sub_field_ranges = &sub_processed.normalized_ranges;
                        let ranges = extract_bits(ranges, sub_field_ranges);
                        generate_bitdump_fields(
                            interface_def,
                            &sub_processed,
                            format!("{prefix}{fname}.", fname = processed_field.field.name),
                            &ranges,
                            out_fields,
                        )?;
                    }

                    // Don't add the main field if we had sub-fields
                    return Ok(());
                }
            }
        }
    }

    let field_ranges: Vec<TokenStream2> = ranges
        .iter()
        .map(|NormalizedRange { start, end }| quote! { #start..#end })
        .collect();
    let field_name = processed_field.field.name.to_string();
    let full_field_name = format!("{prefix}{field_name}");
    let reserved = processed_field.field.is_reserved();
    let value = if reserved {
        quote! { "_".to_string() }
    } else {
        let expr = syn::parse_str::<Expr>(&format!("self.{full_field_name}"))?;
        quote! { #expr }
    };
    let field_type = processed_field.field.field_type.to_token_stream().to_string();
    let rust_type = match &processed_field.field.bit_constraint {
        Some(BitConstraint::Pattern(pat)) => {
            let rs = pat.ranges.iter().map(|x| format!("{x}")).collect::<Vec<_>>().join(", ");
            format!("{field_type}[{rs}]")
        }
        Some(BitConstraint::Size(_, s)) => format!("{field_type}{{{s}}}"),
        None => field_type,
    };
    out_fields.push(quote! {
        embedded_interfaces::bitdump::Field {
            ranges: vec![#(#field_ranges,)*],
            name: #full_field_name.to_string(),
            rust_type: #rust_type.to_string(),
            value: format!("{:?}", #value),
            reserved: #reserved,
        }
    });

    Ok(())
}

fn generate_unpacked_struct(
    interface_def: &InterfaceObjectsDefinition,
    processed_fields: &[super::bit_pattern::ProcessedField],
    packed_name: &Ident,
    unpacked_name: &Ident,
    doc_attrs: &[Attribute],
    size: usize,
) -> syn::Result<TokenStream2> {
    let mut struct_accessors = Vec::new();
    let mut struct_fields = Vec::new();
    let mut default_values = Vec::new();

    let mut out_fields = Vec::new();
    for processed in processed_fields {
        let field = &processed.field;
        let ranges = &processed.normalized_ranges;
        generate_bitdump_fields(interface_def, processed, "".to_string(), ranges, &mut out_fields)?;

        // Skip reserved fields
        if field.is_reserved() {
            continue;
        }

        let field_name = &field.name;
        let field_type = &field.field_type;
        let field_attrs = &field.attributes;
        let generated_field_doc = processed.generate_doc(&processed.normalized_ranges);

        struct_fields.push(quote! {
            #(#field_attrs)*
            #[doc = ""]
            #generated_field_doc
            pub #field_name: #field_type
        });

        let field_doc: Vec<_> = field_attrs.iter().filter(|x| x.path().is_ident("doc")).collect();
        let field_doc = quote! {
            #(#field_doc)*
            #[doc = ""]
            #generated_field_doc
        };

        let pack_accessors =
            super::pack::generate_accessors(interface_def, unpacked_name, processed, ranges, &field_doc, "")?;
        let unpack_accessors =
            super::unpack::generate_accessors(interface_def, unpacked_name, processed, ranges, &field_doc, "")?;
        struct_accessors.push(pack_accessors);
        struct_accessors.push(unpack_accessors);

        // Collect default values
        if let Some(default_val) = &field.default_value {
            default_values.push(quote! { #field_name: #default_val });
        } else {
            default_values.push(quote! { #field_name: Default::default() });
        }
    }

    // Generate the byte conversion method
    let pack_body = super::pack::generate_pack_body(interface_def, packed_name, processed_fields, size)?;
    let derive_attrs = if cfg!(feature = "defmt") {
        quote! {
            #[derive(Copy, Clone, PartialEq, core::fmt::Debug, defmt::Format)]
        }
    } else {
        quote! {
            #[derive(Copy, Clone, PartialEq, core::fmt::Debug)]
        }
    };

    let (bitdump_trait, bitdump_helper) = if cfg!(feature = "std") {
        (
            quote! {
                impl embedded_interfaces::BitdumpFormattable for #unpacked_name {
                    /// Returns an object that implements Display for pretty-printing
                    /// the contents and layout of this bit-packed struct
                    fn bitdump(&self) -> embedded_interfaces::bitdump::BitdumpFormatter {
                        self.bitdump_with_data(self.pack().0.to_vec())
                    }
                }
            },
            quote! {
                /// Returns an object that implements Display for pretty-printing
                /// the contents and layout of this bit-packed struct
                pub fn bitdump_with_data(&self, data: Vec<u8>) -> embedded_interfaces::bitdump::BitdumpFormatter {
                    embedded_interfaces::bitdump::BitdumpFormatter::new(
                        stringify!(#packed_name).to_string(),
                        data,
                        vec![#(#out_fields,)*],
                    )
                }
            },
        )
    } else {
        (quote! {}, quote! {})
    };

    Ok(quote! {
        #(#doc_attrs)*
        #derive_attrs
        pub struct #unpacked_name {
            #(#struct_fields,)*
        }

        impl #unpacked_name {
            #[inline]
            #[doc = concat!("Pack all fields into a [`", stringify!(#packed_name), "`].")]
            pub fn pack(&self) -> #packed_name {
                #pack_body
            }

            #bitdump_helper
        }

        #bitdump_trait

        impl #packed_name {
            #(#struct_accessors)*
        }

        impl Default for #unpacked_name {
            fn default() -> Self {
                Self {
                    #(#default_values,)*
                }
            }
        }
    })
}

/// Generate the packed struct with byte array representation
fn generate_packed_struct(
    interface_def: &InterfaceObjectsDefinition,
    processed_fields: &[super::bit_pattern::ProcessedField],
    packed_name: &Ident,
    unpacked_name: &Ident,
    size: usize,
    doc_attrs: &[Attribute],
) -> syn::Result<TokenStream2> {
    // Generate the byte conversion method
    let unpack_body = super::unpack::generate_unpack_body(interface_def, unpacked_name, processed_fields)?;

    let bitdump_trait = if cfg!(feature = "std") {
        quote! {
            impl embedded_interfaces::BitdumpFormattable for #packed_name {
                /// Returns an object that implements Display for pretty-printing
                /// the contents and layout of this bit-packed struct
                fn bitdump(&self) -> embedded_interfaces::bitdump::BitdumpFormatter {
                    self.unpack().bitdump_with_data(self.0.to_vec())
                }
            }
        }
    } else {
        quote! {}
    };

    Ok(quote! {
        #(#doc_attrs)*
        #[doc = ""]
        #[doc = concat!("This is the packed representation of [`", stringify!(#unpacked_name), "`].")]
        #[derive(Copy, Clone, PartialEq, Eq, embedded_interfaces::bytemuck::Pod, embedded_interfaces::bytemuck::Zeroable)]
        #[repr(transparent)]
        pub struct #packed_name(pub [u8; #size]);

        impl #packed_name {
            #[inline]
            #[doc = concat!("Unpack all fields into a [`", stringify!(#unpacked_name), "`]. If you don't need all fields, this is more expensive than just using the appropriate `read_*` functions directly.")]
            pub fn unpack(&self) -> #unpacked_name {
                #unpack_body
            }
        }

        #bitdump_trait

        impl Default for #packed_name {
            fn default() -> Self {
                #unpacked_name::default().pack()
            }
        }
    })
}

/// Generate conversion implementations between packed and unpacked structs
fn generate_struct_conversions(packed_name: &Ident, unpacked_name: &Ident) -> syn::Result<TokenStream2> {
    Ok(quote! {
        impl AsRef<#unpacked_name> for #unpacked_name {
            #[inline]
            fn as_ref(&self) -> &#unpacked_name {
                self
            }
        }

        impl AsRef<#packed_name> for #packed_name {
            #[inline]
            fn as_ref(&self) -> &#packed_name {
                self
            }
        }

        impl From<&#packed_name> for #unpacked_name {
            #[inline]
            fn from(value: &#packed_name) -> Self {
                value.unpack()
            }
        }

        impl From<#packed_name> for #unpacked_name {
            #[inline]
            fn from(value: #packed_name) -> Self {
                value.unpack()
            }
        }

        impl From<&#unpacked_name> for #packed_name {
            #[inline]
            fn from(value: &#unpacked_name) -> Self {
                value.pack()
            }
        }

        impl From<#unpacked_name> for #packed_name {
            #[inline]
            fn from(value: #unpacked_name) -> Self {
                value.pack()
            }
        }
    })
}
