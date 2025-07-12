//! Reusable packed/unpacked struct generation
//!
//! This module contains the generic logic for generating packed and unpacked
//! struct pairs that can be reused for registers and other similar structures.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::{Attribute, Ident};

use crate::parser::FieldDefinition;

/// Generate a pair of packed and unpacked structs with conversion implementations
///
/// This function is generic and can be used for registers or any other structures
/// that need packed/unpacked representations.
pub fn generate_packed_struct_pair(
    packed_name: &Ident,
    unpacked_name: &Ident,
    fields: &[FieldDefinition],
    doc_attrs: &[Attribute],
    size: usize,
) -> syn::Result<TokenStream2> {
    let unpacked_struct = generate_unpacked_struct(fields, unpacked_name, doc_attrs)?;
    let packed_struct = generate_packed_struct(packed_name, unpacked_name, size, doc_attrs)?;
    let conversions = generate_struct_conversions(packed_name, unpacked_name)?;

    Ok(quote! {
        #unpacked_struct
        #packed_struct
        #conversions
    })
}

/// Generate the unpacked struct with individual fields
fn generate_unpacked_struct(
    fields: &[FieldDefinition],
    unpacked_name: &Ident,
    doc_attrs: &[Attribute],
) -> syn::Result<TokenStream2> {
    let mut struct_fields = Vec::new();
    let mut default_values = Vec::new();

    for field in fields {
        // Skip reserved fields (fields without names)
        if let Some(field_name) = &field.name {
            let field_type = &field.field_type;
            let field_attrs = &field.attributes;

            // Generate field documentation
            let default_doc = if let Some(default_val) = &field.default_value {
                quote! { #[doc = concat!("Default: `", stringify!(#default_val), "`")] }
            } else {
                quote! { #[doc = "Default: not specified"] }
            };

            struct_fields.push(quote! {
                #(#field_attrs)*
                #[doc = ""]
                #default_doc
                pub #field_name: #field_type
            });

            // Collect default values
            if let Some(default_val) = &field.default_value {
                default_values.push(quote! { #field_name: #default_val });
            } else {
                default_values.push(quote! { #field_name: Default::default() });
            }
        }
    }

    Ok(quote! {
        #(#doc_attrs)*
        #[derive(Copy, Clone, PartialEq, core::fmt::Debug, defmt::Format)]
        pub struct #unpacked_name {
            #(#struct_fields,)*
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
    packed_name: &Ident,
    unpacked_name: &Ident,
    size: usize,
    doc_attrs: &[Attribute],
) -> syn::Result<TokenStream2> {
    Ok(quote! {
        #(#doc_attrs)*
        #[doc = concat!("This is the packed representation of the [`", stringify!(#unpacked_name), "`] struct.")]
        #[derive(Copy, Clone, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
        #[repr(transparent)]
        pub struct #packed_name([u8; #size]);

        impl #packed_name {
            #[inline]
            #[doc = concat!("Pack all fields in the given [`", stringify!(#unpacked_name), "`] representation.")]
            pub fn new(value: #unpacked_name) -> Self {
                Self(value.into_bytes())
            }

            #[inline]
            #[doc = concat!("Unpack all fields and return them as a [`", stringify!(#unpacked_name), "`]. If you don't need all fields, this is more expensive than just using the appropriate `read_*` functions directly.")]
            pub fn read_all(&self) -> #unpacked_name {
                #unpacked_name::from_bytes(self.0)
            }

            #[inline]
            #[doc = concat!("Pack all fields in the given [`", stringify!(#unpacked_name), "`] representation. If you only want to write some fields, this is more expensive than just using the appropriate `write_*` functions directly.")]
            pub fn write_all(&mut self, value: #unpacked_name) {
                self.0 = value.into_bytes();
            }

            #[inline]
            #[doc = "Get the raw byte representation"]
            pub fn as_bytes(&self) -> &[u8; #size] {
                &self.0
            }

            #[inline]
            #[doc = "Get the raw byte representation as a mutable reference"]
            pub fn as_bytes_mut(&mut self) -> &mut [u8; #size] {
                &mut self.0
            }

            #[inline]
            #[doc = "Create from raw bytes"]
            pub fn from_bytes(bytes: [u8; #size]) -> Self {
                Self(bytes)
            }
        }

        impl Default for #packed_name {
            fn default() -> Self {
                Self(#unpacked_name::default().into_bytes())
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
                #unpacked_name::from_bytes(value.0)
            }
        }

        impl From<#packed_name> for #unpacked_name {
            #[inline]
            fn from(value: #packed_name) -> Self {
                #unpacked_name::from_bytes(value.0)
            }
        }

        impl From<&#unpacked_name> for #packed_name {
            #[inline]
            fn from(value: &#unpacked_name) -> Self {
                Self(value.into_bytes())
            }
        }

        impl From<#unpacked_name> for #packed_name {
            #[inline]
            fn from(value: #unpacked_name) -> Self {
                Self(value.into_bytes())
            }
        }
    })
}
