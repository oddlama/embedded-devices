use darling::ast::NestedMeta;
use darling::FromMeta;
use proc_macro as pc;
use proc_macro2::{Ident, TokenStream};
use quote::{quote, ToTokens, TokenStreamExt};
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
    let vis = input.vis;
    let fields = input.fields;
    let attrs: TokenStream = input.attrs.iter().map(ToTokens::to_token_stream).collect();
    let register_ident = Ident::from_string(&format!("Register{}", ident))?;

    let mut output = quote! {
        #[derive(bondrewd::Bitfields, Clone, PartialEq, Eq, core::fmt::Debug, defmt::Format)]
        #attrs
        #vis struct #ident
        #fields

        #[derive(Clone, Default, PartialEq, Eq)]
        pub struct #register_ident {
            data: [u8; <#ident as bondrewd::Bitfields<_>>::BYTE_SIZE],
        }

        impl core::fmt::Debug for #register_ident {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                #ident::from(self).fmt(f)
            }
        }

        impl defmt::Format for #register_ident {
            fn format(&self, f: defmt::Formatter) {
                defmt::write!(
                    f,
                    "#register_ident {{ data: {}, bitfield: {} }}",
                    self.data,
                    #ident::from(self)
                )
            }
        }

        impl embedded_registers::Register for #register_ident {
            type Data = [u8; Self::REGISTER_SIZE];
            type Bitfield = #ident;

            const REGISTER_SIZE: usize = <#ident as bondrewd::Bitfields<_>>::BYTE_SIZE;
            const ADDRESS: u8 = 0b001;

            fn data(&self) -> &[u8] {
                &self.data
            }

            fn data_mut(&mut self) -> &mut [u8] {
                &mut self.data
            }
        }

        impl From<&#register_ident> for #ident {
            fn from(val: &#register_ident) -> Self {
                use bondrewd::Bitfields;
                #ident::from_bytes(val.data)
            }
        }

        impl From<#ident> for #register_ident {
            fn from(value: #ident) -> Self {
                use bondrewd::Bitfields;
                Self {
                    data: value.into_bytes(),
                }
            }
        }
    };

    if args.read.is_some() {
        output.append_all(quote! {
            impl embedded_registers::RegisterRead for #register_ident {}
        });
    }

    if args.write.is_some() {
        output.append_all(quote! {
            impl embedded_registers::RegisterWrite for #register_ident {}
        });
    }

    println!("{}", output.to_token_stream());
    Ok(output)
}
