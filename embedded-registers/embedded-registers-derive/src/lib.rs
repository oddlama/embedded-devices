use darling::ast::NestedMeta;
use darling::FromMeta;
use proc_macro as pc;
use proc_macro2::TokenStream;
use quote::{quote, ToTokens};
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
            "A register definition must include read, write, or both!",
        ));
    }

    let input = syn::parse2::<syn::ItemStruct>(input)?;
    let ident = input.ident.clone();
    let output = quote! {
        #input

        impl Register for #ident {
        }
    };

    println!("{}", output.to_token_stream());
    Ok(output)
}

//#[derive(FromDeriveInput, Default)]
//#[darling(default, attributes(register))]
//struct Opts {
//    address: Option<u8>,
//}
//
//#[proc_macro_derive(Register, attributes(register))]
//pub fn derive(input: TokenStream) -> TokenStream {
//    let input = parse_macro_input!(input);
//    let opts = Opts::from_derive_input(&input).expect("Invalid options");
//    let DeriveInput { ident, .. } = input;
//
//    let answer = match opts.address {
//        Some(x) => quote! {
//            fn answer() -> i32 {
//                #x
//            }
//        },
//        None => quote! {},
//    };
//
//    //#[derive(Clone, Default, PartialEq, Eq)]
//    //pub struct Config {
//    //    data: [u8; ConfigBitfield::BYTE_SIZE],
//    //}
//
//    //impl Debug for Config {
//    //    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//    //        ConfigBitfield::from(self).fmt(f)
//    //    }
//    //}
//
//    //impl Format for Config {
//    //    fn format(&self, f: defmt::Formatter) {
//    //        defmt::write!(
//    //            f,
//    //            "Config {{ data: {}, bitfield: {} }}",
//    //            self.data,
//    //            ConfigBitfield::from(self)
//    //        )
//    //    }
//    //}
//
//    //impl Register for Config {
//    //    type Data = [u8; Self::REGISTER_SIZE];
//    //    type Bitfield = ConfigBitfield;
//
//    //    const REGISTER_SIZE: usize = ConfigBitfield::BYTE_SIZE;
//    //    const ADDRESS: u8 = 0b001;
//
//    //    fn data(&self) -> &[u8] {
//    //        &self.data
//    //    }
//
//    //    fn data_mut(&mut self) -> &mut [u8] {
//    //        &mut self.data
//    //    }
//    //}
//
//    //impl From<&Config> for ConfigBitfield {
//    //    fn from(val: &Config) -> Self {
//    //        ConfigBitfield::from_bytes(val.data)
//    //    }
//    //}
//
//    //impl From<ConfigBitfield> for Config {
//    //    fn from(value: ConfigBitfield) -> Self {
//    //        Self {
//    //            data: value.into_bytes(),
//    //        }
//    //    }
//    //}
//
//    //impl RegisterRead for Config {}
//    //impl RegisterWrite for Config {}
//    let output = quote! {
//        impl Register for #ident {
//            #answer
//        }
//    };
//    output.into()
//}
