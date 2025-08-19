use convert_case::{Case, Casing};
use quote::quote;
use regex::Regex;
use syn::{Attribute, Data, DeriveInput, Error, Field, Fields, Path, Result};

pub fn generate(input: &DeriveInput) -> Result<proc_macro2::TokenStream> {
    let name = &input.ident;

    let fields = extract_struct_fields(input)?;
    let measurement_impls = extract_measurement_impls(name, fields)?;

    Ok(quote! {
        impl crate::sensor::Measurement for #name {}

        #(#measurement_impls)*
    })
}

fn extract_struct_fields(input: &DeriveInput) -> Result<&syn::FieldsNamed> {
    match &input.data {
        Data::Struct(data) => match &data.fields {
            Fields::Named(fields) => Ok(fields),
            _ => Err(Error::new_spanned(
                input,
                "Measurement derive only supports structs with named fields",
            )),
        },
        _ => Err(Error::new_spanned(input, "Measurement derive only supports structs")),
    }
}

fn extract_measurement_impls(
    struct_name: &syn::Ident,
    fields: &syn::FieldsNamed,
) -> Result<Vec<proc_macro2::TokenStream>> {
    fields
        .named
        .iter()
        .filter_map(|field| extract_measurement_impl(struct_name, field).transpose())
        .collect()
}

fn extract_measurement_impl(struct_name: &syn::Ident, field: &Field) -> Result<Option<proc_macro2::TokenStream>> {
    let measurement_type = match find_measurement_attribute(&field.attrs)? {
        Some(attr) => attr,
        None => return Ok(None),
    };

    let field_name = field
        .ident
        .as_ref()
        .ok_or_else(|| Error::new_spanned(field, "Field must have a name"))?;

    let field_type = &field.ty;

    let trait_name = format!("{measurement_type}Measurement");
    let trait_ident = syn::Ident::new(&trait_name, measurement_type.span());

    let method_name = measurement_type.to_string().to_case(Case::Snake);
    let method_name = Regex::new(r"([a-zA-Z])_(\d)")
        .unwrap()
        .replace_all(&method_name, "$1$2");
    let method_ident = syn::Ident::new(&method_name, measurement_type.span());

    let (return_expr, return_type) = if is_option_type(field_type) {
        (quote! { self.#field_name }, extract_option_inner_type(field_type))
    } else {
        (quote! { Some(self.#field_name) }, quote! { #field_type })
    };

    Ok(Some(quote! {
        impl crate::sensor::#trait_ident for #struct_name {
            fn #method_ident(&self) -> Option<#return_type> {
                #return_expr
            }
        }
    }))
}

fn is_option_type(ty: &syn::Type) -> bool {
    if let syn::Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            return segment.ident == "Option";
        }
    }
    false
}

fn extract_option_inner_type(ty: &syn::Type) -> proc_macro2::TokenStream {
    if let syn::Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            if segment.ident == "Option" {
                if let syn::PathArguments::AngleBracketed(args) = &segment.arguments {
                    if let Some(syn::GenericArgument::Type(inner_ty)) = args.args.first() {
                        return quote! { #inner_ty };
                    }
                }
            }
        }
    }
    quote! { #ty } // fallback
}

fn find_measurement_attribute(attrs: &[Attribute]) -> Result<Option<syn::Ident>> {
    for attr in attrs {
        if !attr.path().is_ident("measurement") {
            continue;
        }

        let measurement_type = attr.parse_args::<Path>()?;
        return Ok(measurement_type.get_ident().cloned());
    }

    Ok(None)
}
