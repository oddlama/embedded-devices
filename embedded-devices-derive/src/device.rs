use darling::FromMeta;
use darling::ast::NestedMeta;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

#[derive(Debug, FromMeta)]
struct DeviceArgs {}

/// Creates a register marker trait for this device.
pub(crate) fn device(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    let _args = DeviceArgs::from_list(&NestedMeta::parse_meta_list(args)?)?;

    let item = syn::parse2::<syn::ItemStruct>(orig_input.clone())?;
    let ident = item.ident;
    let register_marker = format_ident!("{}Register", ident);

    let output = quote! {
        #orig_input

        /// A marker trait to distinguish registers related to this device from others
        pub trait #register_marker {}
    };

    Ok(output)
}
