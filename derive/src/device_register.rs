//! This crate provides procedural helper macro for embedded-devices

use darling::ast::NestedMeta;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::spanned::Spanned;

pub(crate) fn device_register(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    let args_span = args.span();
    let args = NestedMeta::parse_meta_list(args)?;
    let input = syn::parse2::<syn::ItemStruct>(orig_input.clone())?;
    let ident = input.ident;

    let mut output = quote! {
        #orig_input
    };

    for device in args {
        let NestedMeta::Meta(device_meta) = device else {
            return Err(syn::Error::new(
                args_span,
                "Could not parse device identifier as path. This should not happen, please report this as a bug.",
            ));
        };

        let mut register_marker = device_meta.path().clone();
        let Some(last_segment) = register_marker.segments.last_mut() else {
            return Err(syn::Error::new(
                args_span,
                "Could not parse device identifier as path. This should not happen, please report this as a bug.",
            ));
        };
        last_segment.ident = format_ident!("{}Register", last_segment.ident);
        last_segment.arguments = syn::PathArguments::None;

        output = quote! {
            #output
            impl #register_marker for #ident {}
            impl AsRef<#ident> for #ident {
                #[inline]
                fn as_ref(&self) -> &#ident {
                    self
                }
            }
        };
    }

    Ok(output)
}
