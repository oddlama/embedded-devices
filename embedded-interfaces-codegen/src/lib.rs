//! Packed struct and register definition macro library

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;

mod codegen;
mod parser;

use parser::InterfaceObjectsDefinition;

#[proc_macro]
pub fn interface_objects(input: TokenStream) -> TokenStream {
    let input2 = TokenStream2::from(input);

    match interface_objects_impl(input2) {
        Ok(output) => output.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

fn interface_objects_impl(input: TokenStream2) -> syn::Result<TokenStream2> {
    let registers_def = syn::parse2::<InterfaceObjectsDefinition>(input)?;
    let out = codegen::generate_interface_objects(&registers_def)?;

    Ok(out)
}
