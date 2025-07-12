//! Packed struct and register definition macro library

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;

mod codegen;
mod parser;

use parser::RegistersDefinition;

/// Main macro for defining registers
#[proc_macro]
pub fn registers(input: TokenStream) -> TokenStream {
    let input2 = TokenStream2::from(input);

    match registers_impl(input2) {
        Ok(output) => output.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

fn registers_impl(input: TokenStream2) -> syn::Result<TokenStream2> {
    let registers_def = syn::parse2::<RegistersDefinition>(input)?;
    let out = codegen::generate_registers(&registers_def)?;

    // DEBUG: write to file for inspection, will be removed later (TODO)
    let syn_tree: syn::File = syn::parse_str(&out.to_string()).expect("Failed to parse generated code");
    std::fs::write("/tmp/gen.rs", prettyplease::unparse(&syn_tree)).expect("Failed to write generated source");

    Ok(out)
}
