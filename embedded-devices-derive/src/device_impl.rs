use darling::FromMeta;
use darling::ast::NestedMeta;
use proc_macro2::TokenStream;
use quote::{ToTokens, format_ident, quote};
use syn::spanned::Spanned;

#[derive(Debug, FromMeta)]
struct DeviceImplArgs {}

pub(crate) fn device_impl(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    let args_span = args.span();
    let _args = DeviceImplArgs::from_list(&NestedMeta::parse_meta_list(args)?)?;
    let mut item_impl = syn::parse2::<syn::ItemImpl>(orig_input.clone())?;

    let syn::Type::Path(self_ty_path) = item_impl.self_ty.as_ref() else {
        return Err(syn::Error::new(
            args_span,
            "Could not parse device identifier as path. This should not happen, please report this as a bug.",
        ));
    };
    let mut register_marker = self_ty_path.clone();
    let Some(last_segment) = register_marker.path.segments.last_mut() else {
        return Err(syn::Error::new(
            args_span,
            "Could not parse device identifier as path. This should not happen, please report this as a bug.",
        ));
    };
    last_segment.ident = format_ident!("{}Register", last_segment.ident);
    last_segment.arguments = syn::PathArguments::None;

    let read_register_doc = format!(
        "Reads from the given register. For a list of all available registers, refer to implentors of [`{}`].",
        register_marker.to_token_stream()
    );
    let write_register_doc = format!(
        "Writes to the given register. For a list of all available registers, refer to implentors of [`{}`].",
        register_marker.to_token_stream()
    );

    let additional_items = vec![
        quote! {
            #[doc = #read_register_doc]
            #[inline]
            pub async fn read_register<R>(&mut self) -> Result<R, embedded_interfaces::TransportError<<R as embedded_interfaces::registers::Register>::CodecError, I::BusError>>
            where
                R: embedded_interfaces::registers::ReadableRegister + #register_marker
            {
                self.interface.read_register::<R>().await
            }
        },
        quote! {
            #[doc = #write_register_doc]
            #[inline]
            pub async fn write_register<R>(
                &mut self,
                register: impl AsRef<R>,
            ) -> Result<(), embedded_interfaces::TransportError<<R as embedded_interfaces::registers::Register>::CodecError, I::BusError>>
            where
                R: embedded_interfaces::registers::WritableRegister + #register_marker
            {
                self.interface.write_register::<R>(register).await
            }
        },
    ];

    for i in additional_items {
        item_impl.items.push(syn::parse2::<syn::ImplItem>(i)?);
    }

    let output = quote! {
        #item_impl
    };

    Ok(output)
}
