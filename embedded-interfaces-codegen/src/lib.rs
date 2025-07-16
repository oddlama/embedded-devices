//! Packed struct and register definition macro library

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;

mod codegen;
mod parser;

use parser::InterfaceObjectsDefinition;

/// Main macro for defining registers
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

    // DEBUG: write to file for inspection, will be removed later (TODO)
    let syn_tree: syn::File = syn::parse_str(&out.to_string()).expect("Failed to parse generated code");
    std::fs::write("/tmp/gen.rs", prettyplease::unparse(&syn_tree)).expect("Failed to write generated source");

    Ok(out)
}
