//! Reusable packed/unpacked struct generation
//!
//! This module contains the generic logic for generating packed and unpacked
//! struct pairs that can be reused for registers and other similar structures.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{Attribute, Ident};

use super::bit_pattern::{generate_bit_pattern_doc, process_field_bit_patterns};
use crate::parser::{FieldDefinition, InterfaceObjectsDefinition};

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

    for processed in processed_fields {
        let field = &processed.field;
        // Skip reserved fields
        if field.is_reserved() {
            continue;
        }

        let field_name = &field.name;
        let field_type = &field.field_type;
        let field_attrs = &field.attributes;

        // Generate field documentation
        let default_doc = if let Some(default_val) = &field.default_value {
            quote! { #[doc = concat!("Default: `", stringify!(#default_val), "`")] }
        } else {
            quote! { #[doc = "Default: not specified"] }
        };

        // Generate bit pattern documentation
        let bit_pattern_doc = generate_bit_pattern_doc(&processed.normalized_ranges);
        let bit_doc = quote! { #[doc = #bit_pattern_doc] };

        struct_fields.push(quote! {
            #(#field_attrs)*
            #[doc = ""]
            #default_doc
            #bit_doc
            pub #field_name: #field_type
        });

        if !processed.field.is_reserved() {
            let ranges = &processed.normalized_ranges;
            let pack_accessors = super::pack::generate_accessors(interface_def, unpacked_name, processed, ranges, "")?;
            let unpack_accessors =
                super::unpack::generate_accessors(interface_def, unpacked_name, processed, ranges, "")?;
            struct_accessors.push(pack_accessors);
            struct_accessors.push(unpack_accessors);
        }

        // Collect default values
        if let Some(default_val) = &field.default_value {
            default_values.push(quote! { #field_name: #default_val });
        } else {
            default_values.push(quote! { #field_name: Default::default() });
        }
    }

    // Generate the byte conversion method
    let pack_body = super::pack::generate_pack_body(interface_def, packed_name, processed_fields, size)?;

    Ok(quote! {
        #(#doc_attrs)*
        #[derive(Copy, Clone, PartialEq, core::fmt::Debug, defmt::Format)]
        pub struct #unpacked_name {
            #(#struct_fields,)*
        }

        impl #unpacked_name {
            #[inline]
            #[doc = concat!("Pack all fields into a [`", stringify!(#packed_name), "`].")]
            pub fn pack(&self) -> #packed_name {
                #pack_body
            }
        }

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

    Ok(quote! {
        #(#doc_attrs)*
        #[doc = concat!("This is the packed representation of [`", stringify!(#unpacked_name), "`].")]
        #[derive(Copy, Clone, PartialEq, Eq, embedded_interfaces::bytemuck::Pod, embedded_interfaces::bytemuck::Zeroable)]
        #[repr(transparent)]
        pub struct #packed_name([u8; #size]);

        impl #packed_name {
            #[inline]
            #[doc = concat!("Unpack all fields into a [`", stringify!(#unpacked_name), "`]. If you don't need all fields, this is more expensive than just using the appropriate `read_*` functions directly.")]
            pub fn unpack(&self) -> #unpacked_name {
                #unpack_body
            }
        }

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
