//! Code generation for enum definitions

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;

use crate::parser::{EnumDefinition, EnumPattern};

pub fn generate_enum(enum_def: &EnumDefinition) -> syn::Result<TokenStream2> {
    let enum_name = &enum_def.name;
    let underlying_type = &enum_def.underlying_type;
    let attributes = &enum_def.attributes;
    let variants = &enum_def.variants;

    // Get the effective bit size
    let bit_size = enum_def.get_effective_bit_size()?;

    // Generate enum variants
    let mut enum_variants = Vec::new();
    let mut from_unsigned_arms = Vec::new();
    let mut to_unsigned_arms = Vec::new();

    for variant in variants {
        let variant_name = &variant.name;
        let variant_attrs = &variant.attributes;

        if variant.capture_value {
            // Variant captures the underlying value
            enum_variants.push(quote! {
                #(#variant_attrs)*
                #variant_name(#underlying_type)
            });

            // For from_unsigned, we need to check if the value matches the pattern
            let pattern = generate_pattern(&variant.pattern);
            from_unsigned_arms.push(quote! {
                #pattern => Self::#variant_name(value)
            });

            // For to_unsigned, just return the captured value
            to_unsigned_arms.push(quote! {
                Self::#variant_name(v) => *v
            });
        } else {
            // Variant doesn't capture the value
            enum_variants.push(quote! {
                #(#variant_attrs)*
                #variant_name
            });

            // For from_unsigned, we need to check if the value matches the pattern
            let pattern = generate_pattern(&variant.pattern);
            from_unsigned_arms.push(quote! {
                #pattern => Self::#variant_name
            });

            // For to_unsigned, return the representative value
            let representative = &variant.representative;
            to_unsigned_arms.push(quote! {
                Self::#variant_name => #representative
            });
        }
    }

    // Generate the enum definition
    let enum_def_code = quote! {
        #(#attributes)*
        #[derive(Copy, Clone, PartialEq, Eq, core::fmt::Debug, defmt::Format)]
        pub enum #enum_name {
            #(#enum_variants,)*
        }
    };

    // Check if we have a wildcard pattern to handle unreachable case
    let has_wildcard = variants.iter().any(|v| matches!(v.pattern, EnumPattern::Wildcard));
    // Check if we use all bits in the underlying type. We only require the enum to exhaust the N
    // given bits and need to adjust the match accordingly.
    let uses_all_bits = bit_size == enum_def.get_default_bit_size()?;

    let from_unsigned_match = if has_wildcard || uses_all_bits {
        // If we have a wildcard or our match is fully exhaustive, don't add a default arm
        quote! {
            match value {
                #(#from_unsigned_arms,)*
            }
        }
    } else {
        // If there is no wildcard and our matches are not exhausting the underlying type, we need
        // to add a default case (this should not happen in practice since the trait says the
        // function must succeed for any bit value, but we'll add it for safety)
        quote! {
            match value {
                #(#from_unsigned_arms,)*
                _ => panic!("Unused bits in packed enum values must not be used! This shouldn't have happened, please file a bug report in embedded-interfaces."),
            }
        }
    };

    // Generate the UnsignedPackable implementation
    let packable_impl = quote! {
        impl embedded_interfaces::packable::UnsignedPackable for #enum_name {
            type Base = #underlying_type;
            const BITS: usize = #bit_size;

            fn from_unsigned(value: Self::Base) -> Self {
                #from_unsigned_match
            }

            fn to_unsigned(&self) -> Self::Base {
                match self {
                    #(#to_unsigned_arms,)*
                }
            }
        }
    };

    Ok(quote! {
        #enum_def_code
        #packable_impl
    })
}

/// Generate a macro pattern for a given enum pattern
fn generate_pattern(pattern: &EnumPattern) -> TokenStream2 {
    match pattern {
        EnumPattern::Single(val) => {
            quote! { #val }
        }
        EnumPattern::Range(start, end) => {
            quote! { #start..#end }
        }
        EnumPattern::RangeInclusive(start, end) => {
            quote! { #start..=#end }
        }
        EnumPattern::Multiple(items) => {
            let checks: Vec<_> = items.iter().map(generate_pattern).collect();
            quote! { #(#checks)|* }
        }
        EnumPattern::Wildcard => {
            quote! { _ }
        }
    }
}
