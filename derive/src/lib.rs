//! This crate provides procedural helper macro for embedded-devices

use darling::ast::NestedMeta;
use darling::FromMeta;
use proc_macro as pc;
use proc_macro2::TokenStream;
use quote::{format_ident, quote, ToTokens};
use convert_case::{Case, Casing};

#[derive(Debug, FromMeta)]
struct SimpleDeviceRegisterArgs {
    /// The device to which the register belongs.
    device: syn::Path,
    /// The address of the register
    address: u8,
    /// The register mode (one of "r", "w", "rw")
    #[darling(default)]
    mode: String,
}

#[proc_macro_attribute]
pub fn simple_device_register(args: pc::TokenStream, input: pc::TokenStream) -> pc::TokenStream {
    match simple_device_register_impl(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

/// This adds accessor functions for the given register and
/// proxies the embedded_registers::register attribute.
fn simple_device_register_impl(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    let args = SimpleDeviceRegisterArgs::from_list(&NestedMeta::parse_meta_list(args)?)?;
    let device = args.device;
    let address = args.address;
    let mode = args.mode;

    let input = syn::parse2::<syn::ItemStruct>(orig_input.clone())?;
    let ident = input.ident;
    let docattrs: TokenStream = input
        .attrs
        .iter()
        .filter(|x| x.path().is_ident("doc"))
        .map(ToTokens::to_token_stream)
        .collect();

    let read_comment = format!("This reads the [`{ident}`] register from this device:");
    let write_comment = format!("This writes the [`{ident}`] register to this device:");

    let ident_snake = ident.to_string().to_case(Case::Snake);
    let read_fn_name = format_ident!("read_{}", ident_snake);
    let write_fn_name = format_ident!("write_{}", ident_snake);

    let is_read = matches!(mode.as_str(), "r" | "rw");
    let is_write = matches!(mode.as_str(), "w" | "rw");

    if !is_read && !is_write {
        return Ok(orig_input);
    }

    let mut register_fns = quote! {};

    if is_read {
        register_fns = quote! {
            #register_fns

            #[doc = #read_comment]
            #[doc = ""]
            #docattrs
            #[inline]
            pub async fn #read_fn_name(&mut self) -> Result<#ident, I::Error> {
                self.interface.read_register::<#ident>().await
            }
        };
    }

    if is_write {
        register_fns = quote! {
            #register_fns

            #[doc = #write_comment]
            #[doc = ""]
            #docattrs
            #[inline]
            pub async fn #write_fn_name(&mut self, register: &#ident) -> Result<(), I::Error> {
                self.interface.write_register::<#ident>(register).await
            }
        };
    }

    let output = quote! {
        #[embedded_registers::register(address = #address, mode = #mode)]
        #orig_input

        impl<I> #device<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            #register_fns
        }
    };

    Ok(output)
}
