//! Code generation

use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::Ident;

use crate::parser::*;

pub fn generate_registers(registers_def: &RegistersDefinition) -> syn::Result<TokenStream2> {
    let mut generated = TokenStream2::new();

    // Generate code for each register
    for register in &registers_def.registers {
        let register_code = generate_register(registers_def, register)?;
        generated.extend(register_code);
    }

    Ok(generated)
}

fn generate_register(registers_def: &RegistersDefinition, register: &RegisterDefinition) -> syn::Result<TokenStream2> {
    let register_name = &register.name;
    let unpacked_name = format_ident!("{}Unpacked", register_name);

    // Get effective attributes (defaults + register-specific)
    let attrs = registers_def.get_effective_attrs(register);

    // Extract register attributes
    let (addr, mode, size) = extract_register_attrs(&attrs)?;
    let (codec_error, i2c_codec, spi_codec) = extract_codec_attrs(&attrs)?;

    // Generate unpacked struct with non-reserved fields
    let unpacked_struct = generate_unpacked_struct(register, &unpacked_name)?;

    // Generate packed struct
    let packed_struct = generate_packed_struct(register, register_name, size)?;

    // Generate trait implementations
    let trait_impls = generate_trait_implementations(
        register_name,
        &unpacked_name,
        &addr,
        &mode,
        size,
        &codec_error,
        &i2c_codec,
        &spi_codec,
    )?;

    Ok(quote! {
        #unpacked_struct
        #packed_struct
        #trait_impls
    })
}

fn extract_register_attrs(attrs: &[RegisterAttr]) -> syn::Result<(TokenStream2, String, usize)> {
    let mut addr = None;
    let mut mode = None;
    let mut size = None;

    for attr in attrs {
        match attr.name.to_string().as_str() {
            "addr" => {
                if let RegisterAttrValue::Int(lit_int) = &attr.value {
                    addr = Some(quote! { #lit_int });
                }
            }
            "mode" => {
                if let RegisterAttrValue::Ident(ident) = &attr.value {
                    mode = Some(ident.to_string());
                }
            }
            "size" => {
                if let RegisterAttrValue::Int(lit_int) = &attr.value {
                    size = Some(lit_int.base10_parse::<usize>()?);
                }
            }
            _ => {}
        }
    }

    let addr = addr.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing addr attribute"))?;
    let mode = mode.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing mode attribute"))?;
    let size = size.ok_or_else(|| syn::Error::new_spanned(&attrs[0].name, "Missing size attribute"))?;

    Ok((addr, mode, size))
}

fn extract_codec_attrs(attrs: &[RegisterAttr]) -> syn::Result<(TokenStream2, TokenStream2, TokenStream2)> {
    let mut codec_error = None;
    let mut i2c_codec = None;
    let mut spi_codec = None;

    for attr in attrs {
        match attr.name.to_string().as_str() {
            "codec_error" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    codec_error = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    codec_error = Some(quote! { #ident });
                }
                _ => {}
            },
            "i2c_codec" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    i2c_codec = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    i2c_codec = Some(quote! { #ident });
                }
                _ => {}
            },
            "spi_codec" => match &attr.value {
                RegisterAttrValue::Ident(ident) => {
                    spi_codec = Some(quote! { #ident });
                }
                RegisterAttrValue::String(lit_str) => {
                    let ident: Ident = syn::parse_str(&lit_str.value())?;
                    spi_codec = Some(quote! { #ident });
                }
                _ => {}
            },
            _ => {}
        }
    }

    let codec_error = codec_error.unwrap_or_else(|| quote! { () });
    let i2c_codec = i2c_codec.unwrap_or_else(|| quote! { () });
    let spi_codec = spi_codec.unwrap_or_else(|| quote! { () });

    Ok((codec_error, i2c_codec, spi_codec))
}

fn generate_unpacked_struct(register: &RegisterDefinition, unpacked_name: &Ident) -> syn::Result<TokenStream2> {
    let mut fields = Vec::new();
    let mut default_values = Vec::new();

    // Copy documentation from register
    let doc_attrs = &register.attributes;

    for field in &register.fields {
        // Skip reserved fields (fields without names)
        if let Some(field_name) = &field.name {
            let field_type = &field.field_type;
            let field_attrs = &field.attributes;

            // Generate field documentation
            let default_doc = if let Some(default_val) = &field.default_value {
                quote! { #[doc = concat!("Default: `", stringify!(#default_val), "`")] }
            } else {
                quote! { #[doc = "Default: not specified"] }
            };

            fields.push(quote! {
                #(#field_attrs)*
                #[doc = ""]
                #default_doc
                pub #field_name: #field_type
            });

            // Collect default values
            if let Some(default_val) = &field.default_value {
                default_values.push(quote! { #field_name: #default_val });
            } else {
                default_values.push(quote! { #field_name: Default::default() });
            }
        }
    }

    Ok(quote! {
        #(#doc_attrs)*
        #[derive(Copy, Clone, PartialEq, core::fmt::Debug, defmt::Format)]
        pub struct #unpacked_name {
            #(#fields,)*
        }

        impl Default for #unpacked_name {
            fn default() -> Self {
                Self {
                    #(#default_values,)*
                }
            }
        }
    })
}

fn generate_packed_struct(
    register: &RegisterDefinition,
    register_name: &Ident,
    size: usize,
) -> syn::Result<TokenStream2> {
    let unpacked_name = format_ident!("{}Unpacked", register_name);

    Ok(quote! {
        #[doc = concat!("This is the packed representation of [`", stringify!(#register_name), "`].")]
        #[derive(Copy, Clone, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
        #[repr(transparent)]
        pub struct #register_name([u8; #size]);

        impl #register_name {
            #[inline]
            #[doc = concat!("Pack all fields in the given [`", stringify!(#unpacked_name), "`] representation.")]
            pub fn new(value: #unpacked_name) -> Self {
                Self(value.into_bytes())
            }

            #[inline]
            #[doc = concat!("Unpack all fields and return them as a [`", stringify!(#unpacked_name), "`]. If you don't need all fields, this is more expensive than just using the appropriate `read_*` functions directly.")]
            pub fn read_all(&self) -> #unpacked_name {
                #unpacked_name::from_bytes(self.0)
            }

            #[inline]
            #[doc = concat!("Pack all fields in the given [`", stringify!(#unpacked_name), "`] representation. If you only want to write some fields, this is more expensive than just using the appropriate `write_*` functions directly.")]
            pub fn write_all(&mut self, value: #unpacked_name) {
                self.0 = value.into_bytes();
            }
        }

        impl Default for #register_name {
            fn default() -> Self {
                Self(#unpacked_name::default().into_bytes())
            }
        }
    })
}

fn generate_trait_implementations(
    register_name: &Ident,
    unpacked_name: &Ident,
    addr: &TokenStream2,
    mode: &str,
    size: usize,
    codec_error: &TokenStream2,
    i2c_codec: &TokenStream2,
    spi_codec: &TokenStream2,
) -> syn::Result<TokenStream2> {
    // Generate readable/writable traits based on mode
    let mode_traits = match mode {
        "r" => quote! { impl embedded_interfaces::registers::ReadableRegister for #register_name {} },
        "w" => quote! { impl embedded_interfaces::registers::WritableRegister for #register_name {} },
        "rw" => quote! {
            impl embedded_interfaces::registers::ReadableRegister for #register_name {}
            impl embedded_interfaces::registers::WritableRegister for #register_name {}
        },
        _ => {
            return Err(syn::Error::new_spanned(
                register_name,
                format!("Invalid mode: {}", mode),
            ));
        }
    };

    Ok(quote! {
        impl embedded_interfaces::registers::Register for #register_name {
            type Unpacked = #unpacked_name;
            type CodecError = #codec_error;
            type SpiCodec = #spi_codec;
            type I2cCodec = #i2c_codec;
            const REGISTER_SIZE: usize = #size;
            const ADDRESS: u64 = #addr;
        }

        #mode_traits

        impl AsRef<#unpacked_name> for #unpacked_name {
            #[inline]
            fn as_ref(&self) -> &#unpacked_name {
                self
            }
        }

        impl AsRef<#register_name> for #register_name {
            #[inline]
            fn as_ref(&self) -> &#register_name {
                self
            }
        }

        impl From<&#register_name> for #unpacked_name {
            #[inline]
            fn from(value: &#register_name) -> Self {
                #unpacked_name::from_bytes(value.0)
            }
        }

        impl From<#register_name> for #unpacked_name {
            #[inline]
            fn from(value: #register_name) -> Self {
                #unpacked_name::from_bytes(value.0)
            }
        }

        impl From<&#unpacked_name> for #register_name {
            #[inline]
            fn from(value: &#unpacked_name) -> Self {
                Self(value.into_bytes())
            }
        }

        impl From<#unpacked_name> for #register_name {
            #[inline]
            fn from(value: #unpacked_name) -> Self {
                Self(value.into_bytes())
            }
        }
    })
}
