use darling::ast::NestedMeta;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::spanned::Spanned;
use syn::{Attribute, Ident, ItemImpl};

pub(crate) fn sensor(args: TokenStream, orig_input: TokenStream) -> syn::Result<TokenStream> {
    // Parse the arguments - this handles the list like (Temperature, Pressure)
    let meta_list = if args.is_empty() {
        Vec::new()
    } else {
        NestedMeta::parse_meta_list(args)?
    };

    // Convert the meta list to sensor identifiers
    let sensor_idents: Vec<Ident> = meta_list
        .iter()
        .map(|meta| match meta {
            NestedMeta::Meta(syn::Meta::Path(path)) => path
                .get_ident()
                .cloned()
                .ok_or_else(|| syn::Error::new(path.span(), "Expected simple identifier")),
            _ => Err(syn::Error::new(meta.span(), "Expected sensor type identifier")),
        })
        .collect::<syn::Result<Vec<_>>>()?;

    let item_impl = syn::parse2::<ItemImpl>(orig_input.clone())?;

    // Extract attributes that should be copied to generated impls
    // Filter out the #[sensor(...)] attribute itself
    let copyable_attrs: Vec<Attribute> = item_impl
        .attrs
        .iter()
        .filter(|attr| {
            // Skip the sensor attribute - we don't want to copy it
            !attr.path().is_ident("sensor")
        })
        .cloned()
        .collect();

    // Start with the original impl
    let mut result = quote! { #item_impl };

    // Generate additional impl blocks for each sensor type
    for sensor_type in sensor_idents {
        let sensor_trait_name = format_ident!("{}Sensor", sensor_type);

        // Extract the type being implemented and generics
        let self_ty = &item_impl.self_ty;
        let generics = &item_impl.generics;
        let (impl_generics, _, where_clause) = generics.split_for_impl();

        // Generate the new impl block
        let new_impl = quote! {
            #(#copyable_attrs)*
            impl #impl_generics crate::sensors::#sensor_trait_name for #self_ty #where_clause {}
        };

        result.extend(new_impl);
    }

    Ok(result)
}
