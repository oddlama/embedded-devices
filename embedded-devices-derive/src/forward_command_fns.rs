use darling::FromMeta;
use darling::ast::NestedMeta;
use proc_macro2::TokenStream;
use quote::{ToTokens, format_ident, quote};
use syn::spanned::Spanned;

#[derive(Debug, FromMeta)]
struct ForwardCommandFnsArgs {}

pub(crate) fn forward_command_fns(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    let args_span = args.span();
    let _args = ForwardCommandFnsArgs::from_list(&NestedMeta::parse_meta_list(args)?)?;
    let mut item_impl = syn::parse2::<syn::ItemImpl>(orig_input.clone())?;

    let syn::Type::Path(self_ty_path) = item_impl.self_ty.as_ref() else {
        return Err(syn::Error::new(
            args_span,
            "Could not parse identifier as path. This should not happen, please report this as a bug.",
        ));
    };
    let mut command_marker = self_ty_path.clone();
    let Some(last_segment) = command_marker.path.segments.last_mut() else {
        return Err(syn::Error::new(
            args_span,
            "Could not parse identifier as path. This should not happen, please report this as a bug.",
        ));
    };
    last_segment.ident = format_ident!("{}Command", last_segment.ident);
    last_segment.arguments = syn::PathArguments::None;

    let execute_doc = format!(
        "Execute the given command on this device. For a list of all available commands, refer to implementors of [`{}`].",
        command_marker.to_token_stream()
    );

    let execute_forward = quote! {
        #[doc = #execute_doc]
        #[inline]
        pub async fn execute<C>(
            &mut self,
            input: C::In,
        ) -> Result<C::Out, embedded_interfaces::TransportError<C::ExecutorError, I::BusError>>
        where
            C: embedded_interfaces::commands::Command + #command_marker
        {
            self.interface.execute::<C, D>(&mut self.delay, input).await
        }
    };

    item_impl.items.push(syn::parse2::<syn::ImplItem>(execute_forward)?);
    let output = quote! {
        #item_impl
    };

    Ok(output)
}
