//! Parser implementations

use super::ast::*;
use syn::{
    Attribute, Expr, Ident, LitInt, Token, Type,
    parse::{Parse, ParseStream, Result},
    punctuated::Punctuated,
    token::{Brace, Bracket},
};

impl Parse for RegistersDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let mut defaults = None;
        let mut devices = None;
        let mut registers = Vec::new();

        while !input.is_empty() {
            // Check for defaults block
            if input.peek(Ident) && input.peek2(Brace) {
                let lookahead = input.lookahead1();
                if lookahead.peek(Ident) {
                    let ident: Ident = input.fork().parse()?;
                    if ident == "defaults" {
                        if defaults.is_some() {
                            return Err(input.error("Multiple defaults blocks are not allowed"));
                        }
                        defaults = Some(input.parse()?);
                        continue;
                    }
                }
            }

            // Check for devices block
            if input.peek(Ident) && input.peek2(Bracket) {
                let lookahead = input.lookahead1();
                if lookahead.peek(Ident) {
                    let ident: Ident = input.fork().parse()?;
                    if ident == "devices" {
                        if devices.is_some() {
                            return Err(input.error("Multiple devices blocks are not allowed"));
                        }
                        devices = Some(input.parse()?);
                        continue;
                    }
                }
            }

            // Parse register definition
            registers.push(input.parse()?);
        }

        Ok(RegistersDefinition {
            defaults,
            devices,
            registers,
        })
    }
}

impl Parse for DefaultsBlock {
    fn parse(input: ParseStream) -> Result<Self> {
        let _defaults_token: Ident = input.parse()?;
        let content;
        syn::braced!(content in input);

        let mut defaults = Vec::new();
        while !content.is_empty() {
            let attr_name: Ident = content.parse()?;
            content.parse::<Token![=]>()?;
            let value: Expr = content.parse()?;

            defaults.push(Attr { name: attr_name, value });

            // Optional comma
            if content.peek(Token![,]) {
                content.parse::<Token![,]>()?;
            }
        }

        Ok(DefaultsBlock { defaults })
    }
}

impl Parse for DevicesBlock {
    fn parse(input: ParseStream) -> Result<Self> {
        let _devices_token: Ident = input.parse()?;
        let content;
        syn::bracketed!(content in input);

        let devices = Punctuated::<Ident, Token![,]>::parse_terminated(&content)?
            .into_iter()
            .collect();

        Ok(DevicesBlock { devices })
    }
}

impl Parse for RegisterDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let attributes = input.call(Attribute::parse_outer)?;
        let name: Ident = input.parse()?;

        // Parse register attributes in parentheses
        let register_attrs = if input.peek(syn::token::Paren) {
            let content;
            syn::parenthesized!(content in input);
            let mut attrs = Vec::new();

            while !content.is_empty() {
                let attr_name: Ident = content.parse()?;
                content.parse::<Token![=]>()?;
                let value: Expr = content.parse()?;

                attrs.push(Attr { name: attr_name, value });

                if content.peek(Token![,]) {
                    content.parse::<Token![,]>()?;
                }
            }
            attrs
        } else {
            Vec::new()
        };

        // Parse fields block
        let field_content;
        syn::braced!(field_content in input);

        let mut fields = Vec::new();
        while !field_content.is_empty() {
            fields.push(field_content.parse()?);
        }

        Ok(RegisterDefinition {
            attributes,
            name,
            register_attrs,
            fields,
        })
    }
}

impl Parse for FieldDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let attributes = input.call(Attribute::parse_outer)?;

        // Field name (or _ for reserved)
        let name = if input.peek(Token![_]) {
            input.parse::<Token![_]>()?;
            // Check if it's a named reserved field like _reserved1
            if input.peek(Ident) { Some(input.parse()?) } else { None }
        } else {
            Some(input.parse()?)
        };

        input.parse::<Token![:]>()?;
        let field_type: Type = input.parse()?;

        // Check for size constraint syntax: {size}
        let size_constraint = if input.peek(syn::token::Brace) {
            let content;
            syn::braced!(content in input);
            let size_lit: LitInt = content.parse()?;
            Some(size_lit.base10_parse::<u32>()?)
        } else {
            None
        };

        // Optional bit pattern (mutually exclusive with size constraint)
        let bit_pattern = if input.peek(syn::token::Bracket) {
            if size_constraint.is_some() {
                return Err(input.error("Cannot specify both size constraint {n} and bit pattern [n..m]"));
            }
            Some(input.parse()?)
        } else {
            None
        };

        // Optional default value
        let default_value = if input.peek(Token![=]) {
            input.parse::<Token![=]>()?;
            Some(input.parse()?)
        } else {
            None
        };

        // Optional units block (placeholder)
        let units = if input.peek(syn::token::Brace) && size_constraint.is_none() {
            let _content;
            syn::braced!(_content in input);
            // TODO: Parse units block content
            Some(UnitsBlock {})
        } else {
            None
        };

        // Optional comma
        if input.peek(Token![,]) {
            input.parse::<Token![,]>()?;
        }

        Ok(FieldDefinition {
            attributes,
            name,
            field_type,
            bit_pattern,
            size_constraint,
            default_value,
            units,
        })
    }
}

impl Parse for BitPattern {
    fn parse(input: ParseStream) -> Result<Self> {
        let content;
        syn::bracketed!(content in input);

        let mut ranges = Vec::new();
        while !content.is_empty() {
            let start: LitInt = content.parse()?;
            let start_val = start.base10_parse::<u32>()?;

            if content.peek(Token![..=]) {
                content.parse::<Token![..=]>()?;
                let end: LitInt = content.parse()?;
                let end_val = end.base10_parse::<u32>()?;
                ranges.push(BitRange::RangeInclusive(start_val, end_val));
            } else if content.peek(Token![..]) {
                content.parse::<Token![..]>()?;
                let end: LitInt = content.parse()?;
                let end_val = end.base10_parse::<u32>()?;
                ranges.push(BitRange::Range(start_val, end_val));
            } else {
                ranges.push(BitRange::Single(start_val));
            }

            if content.peek(Token![,]) {
                content.parse::<Token![,]>()?;
            }
        }

        Ok(BitPattern { ranges })
    }
}
