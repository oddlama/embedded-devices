//! This crate provides a procedural macro for effortless definitions of registers
//! in embedded device drivers.
//!
//! Currently, embedded-registers requires you to use `#![feature(generic_arg_infer)]`.
//!
//! # Attribute macro
//!
//! Registers are defined by adding `#[register(...)]` to the definition of a bondrewd bitfield.
//! As a short reminder, bondrewd is another proc macro that allows you to define a bitfield structure,
//! which is very handy when dealing with registers, where multiple values are often tightly packed bit-on-bit.
//!
//! The register attribute macro supports the following arguments:
//!
//! <table>
//!   <tr>
//!     <td>address</td>
//!     <td>The virtual address associated to the register.</td>
//!   </tr>
//!   <tr>
//!     <td>read</td>
//!     <td>Add this if the register should be readable</td>
//!   </tr>
//!   <tr>
//!     <td>write</td>
//!     <td>Add this if the register should be writeable</td>
//!   </tr>
//! </table>
//!
//! Adding this attribute to a bondrewd struct `Foo` will result in two types being defined:
//! - `Foo` will become the register, essentially a byte array with the correct size that provides
//!   getter and setter functions for the individual fields.
//! - `FooBitfield` will become the underlying bondrewd bitfield, which may be used to construct
//!   a register from scratch, or can be obtained via [Foo::read_all] if you want to unpack all values.
//!
//! This has the advantage that reading a register incurs no additional memory and CPU cost to unpack all
//! values of the bitfield. You only pay for the members you actually access.
//!
//! # Simple Example
//!
//! This simple example defines the `DeviceId` register of an MCP9808. It has
//! the virtual address `0b111 (0x7)`, uses big endian byte order with the first
//! member of the struct positioned at the most significant bit, is 2 bytes in size
//! and is read-only:
//!
//! ```
//! #[register(address = 0b111, read)]
//! #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
//! pub struct DeviceId {
//!     device_id: u8,
//!     revision: u8,
//! }
//! ```
//!
//! The register may now be read from an I2C bus using sync or async operations:
//!
//! ```
//! let address = 0x24; // I2C device address
//! let reg = DeviceId::read_i2c(&mut i2c, address).await?;
//! // sync: let reg = DeviceId::read_i2c_blocking(&mut i2c, address);
//! ```
//!
//! # Complex Example
//!
//! A more complex example may involve adding your own Bondrewd-capable enums.
//! Have a look at this excerpt of the Configuration register from the MCP9808:
//!
//! ```
//! #[allow(non_camel_case_types)]
//! #[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
//! #[bondrewd_enum(u8)]
//! pub enum Hysteresis {
//!     Deg_0_0C = 0b00,
//!     Deg_1_5C = 0b01,
//!     Deg_3_0C = 0b10,
//!     Deg_6_0C = 0b11,
//! }
//!
//! #[derive(BitfieldEnum, Clone, PartialEq, Eq, Debug, Format)]
//! #[bondrewd_enum(u8)]
//! pub enum ShutdownMode {
//!     Continuous = 0,
//!     Shutdown = 1,
//! }
//!
//! #[register(address = 0b001, read, write)]
//! #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
//! pub struct Config {
//!     // padding
//!     #[bondrewd(bit_length = 5, reserve)]
//!     #[allow(dead_code)]
//!     reserved: u8,
//!
//!     #[bondrewd(enum_primitive = "u8", bit_length = 2)]
//!     pub hysteresis: Hysteresis,
//!     #[bondrewd(enum_primitive = "u8", bit_length = 1)]
//!     pub shutdown_mode: ShutdownMode,
//!     // ... all 16 bits must be filled
//! }
//! ```
//!
//! This now allows us to read and write the register, while only
//! unpacking the fields we require:
//!
//! ```
//! let reg = Config::read_i2c(&mut i2c, address).await?;
//! info!("previous shutdown mode: {}", reg.read_shutdown_mode());
//! reg.write_shutdown_mode(ShutdownMode::Shutdown);
//! reg.write_i2c(&mut i2c, address).await?;
//! ```
//!
//! If you want to get the full decoded bitfield, you can use either `read_all`
//! or `.into()`. If you need to unpack all fields anyway, this might be
//! more convenient as it allows you to access the bitfield members more ergonomically.
//!
//! ```
//! //let bf: ConfigBitfield = reg.into();
//! let mut bf = reg.read_all();
//! info!("previous shutdown mode: {}", reg.shutdown_mode);
//! bf.shutdown_mode = ShutdownMode::Shutdown;
//! reg.write_all(bf);
//! ```

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
