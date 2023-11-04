//!
//!
//!
//!

use darling::ast::NestedMeta;
use darling::FromMeta;
use proc_macro as pc;
use proc_macro2::TokenStream;
use quote::{format_ident, quote, ToTokens, TokenStreamExt};
use syn::spanned::Spanned;

#[derive(Debug, FromMeta)]
struct RegisterArgs {
    address: u8,
    #[darling(default)]
    read: Option<()>,
    #[darling(default)]
    write: Option<()>,
}

#[proc_macro_attribute]
pub fn register(args: pc::TokenStream, input: pc::TokenStream) -> pc::TokenStream {
    match register_impl(args.into(), input.into()) {
        Ok(result) => result.into(),
        Err(e) => e.into_compile_error().into(),
    }
}

fn register_impl(args: TokenStream, input: TokenStream) -> syn::Result<TokenStream> {
    let args_span = args.span();
    let args = RegisterArgs::from_list(&NestedMeta::parse_meta_list(args)?)?;
    if args.read.is_none() && args.write.is_none() {
        return Err(syn::Error::new(
            args_span,
            "A register definition must include read, write, or both",
        ));
    }

    let input = syn::parse2::<syn::ItemStruct>(input)?;
    if !input.attrs.iter().any(|x| x.path().is_ident("bondrewd")) {
        return Err(syn::Error::new(
            args_span,
            "A register definition must also include a #[bondrewd()] bitfield spec",
        ));
    }

    let ident = input.ident;
    let debug_format_str = format!("{} ({{:?}}) => {{:?}}", ident);
    let vis = input.vis;
    let fields = input.fields.clone();
    let attrs: TokenStream = input.attrs.iter().map(ToTokens::to_token_stream).collect();
    let bitfield_ident = format_ident!("{}Bitfield", ident);

    let mut forward_fns = quote! {};
    for field in input.fields {
        let type_ident = field.ty;
        let field_name = field.ident.ok_or_else(|| {
            syn::Error::new(args_span, "A field contains a field without an identifier")
        })?;

        let read_field_name = format_ident!("read_{field_name}");
        let read_comment =
            format!("Calls [`{bitfield_ident}::{read_field_name}`] on the stored data.");
        let write_field_name = format_ident!("write_{field_name}");
        let write_comment =
            format!("Calls [`{bitfield_ident}::{write_field_name}`] on the stored data.");

        forward_fns = quote! {
            #forward_fns

            #[inline]
            #[doc = #read_comment]
            pub fn #read_field_name(&self) -> #type_ident {
                #bitfield_ident::#read_field_name(&self.data)
            }

            #[inline]
            #[doc = #write_comment]
            pub fn #write_field_name(&mut self, #field_name: #type_ident) {
                #bitfield_ident::#write_field_name(&mut self.data, #field_name)
            }
        };
    }

    let read_all_comment = format!("Calls [`{bitfield_ident}::from_bytes`] on the stored data.");
    let write_all_comment =
        format!("Calls [`{bitfield_ident}::to_bytes`] replacing the stored data.");
    let address = args.address;

    let mut output = quote! {
        #[derive(bondrewd::Bitfields, Clone, PartialEq, Eq, core::fmt::Debug, defmt::Format)]
        #attrs
        #vis struct #bitfield_ident
        #fields

        #[derive(Clone, Default, PartialEq, Eq)]
        pub struct #ident {
            data: [u8; <#bitfield_ident as bondrewd::Bitfields<_>>::BYTE_SIZE],
        }

        impl #ident {
            #forward_fns

            #[inline]
            #[doc = #read_all_comment]
            fn read_all(&self) -> #bitfield_ident {
                use bondrewd::Bitfields;
                #bitfield_ident::from_bytes(self.data)
            }

            #[inline]
            #[doc = #write_all_comment]
            fn write_all(&mut self, value: #bitfield_ident) {
                use bondrewd::Bitfields;
                self.data = value.into_bytes();
            }
        }

        impl core::fmt::Debug for #ident {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(
                    f,
                    #debug_format_str,
                    self.data,
                    #bitfield_ident::from(self)
                )
            }
        }

        impl defmt::Format for #ident {
            fn format(&self, f: defmt::Formatter) {
                defmt::write!(
                    f,
                    #debug_format_str,
                    self.data,
                    #bitfield_ident::from(self)
                )
            }
        }

        impl embedded_registers::Register for #ident {
            type Data = [u8; Self::REGISTER_SIZE];
            type Bitfield = #bitfield_ident;

            const REGISTER_SIZE: usize = <#bitfield_ident as bondrewd::Bitfields<_>>::BYTE_SIZE;
            const ADDRESS: u8 = #address;

            #[inline]
            fn data(&self) -> &[u8] {
                &self.data
            }

            #[inline]
            fn data_mut(&mut self) -> &mut [u8] {
                &mut self.data
            }
        }

        impl From<&#ident> for #bitfield_ident {
            #[inline]
            fn from(val: &#ident) -> Self {
                use bondrewd::Bitfields;
                #bitfield_ident::from_bytes(val.data)
            }
        }

        impl From<&#bitfield_ident> for #ident {
            #[inline]
            fn from(value: &#bitfield_ident) -> Self {
                use bondrewd::Bitfields;
                Self {
                    data: value.clone().into_bytes(),
                }
            }
        }
    };

    if args.read.is_some() {
        output.append_all(quote! {
            impl embedded_registers::RegisterRead for #ident {}
        });
    }

    if args.write.is_some() {
        output.append_all(quote! {
            impl embedded_registers::RegisterWrite for #ident {}
        });
    }

    Ok(output)
}
