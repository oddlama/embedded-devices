//! Parser implementations

use super::ast::*;
use syn::{
    Attribute, Expr, Ident, LitInt, Token, Type,
    parse::{Parse, ParseStream, Result},
    punctuated::Punctuated,
    token::{Brace, Bracket, Paren},
};

impl Parse for InterfaceObjectsDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let mut register_defaults = None;
        let mut devices = None;
        let mut definitions = Vec::new();

        while !input.is_empty() {
            // Check for register_defaults block
            if input.peek(Ident) && input.peek2(Brace) {
                let lookahead = input.lookahead1();
                if lookahead.peek(Ident) {
                    let ident: Ident = input.fork().parse()?;
                    if ident == "register_defaults" {
                        if register_defaults.is_some() {
                            return Err(input.error("Multiple register_defaults blocks are not allowed"));
                        }
                        register_defaults = Some(input.parse()?);
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

            // Parse definition (register, struct, or enum)
            definitions.push(input.parse()?);
        }

        Ok(InterfaceObjectsDefinition {
            register_defaults,
            devices,
            definitions,
        })
    }
}

impl Parse for Definition {
    fn parse(input: ParseStream) -> Result<Self> {
        let attributes = input.call(Attribute::parse_outer)?;

        let lookahead = input.lookahead1();
        if lookahead.peek(Ident) {
            let ident: Ident = input.fork().parse()?;
            match ident.to_string().as_str() {
                "register" => {
                    input.parse::<Ident>()?; // consume "register"
                    let mut register_def: RegisterDefinition = input.parse()?;
                    register_def.attributes = attributes;
                    Ok(Definition::Register(register_def))
                }
                _ => Err(input.error("Expected 'register', 'struct', or 'enum'")),
            }
        } else if lookahead.peek(Token![enum]) {
            input.parse::<Token![enum]>()?; // consume "enum"
            let mut enum_def: EnumDefinition = input.parse()?;
            enum_def.attributes = attributes;

            // Process variants to validate and set representatives
            enum_def.process_variants()?;

            Ok(Definition::Enum(enum_def))
        } else if lookahead.peek(Token![struct]) {
            input.parse::<Token![struct]>()?; // consume "struct"
            let mut struct_def: StructDefinition = input.parse()?;
            struct_def.attributes = attributes;
            Ok(Definition::Struct(struct_def))
        } else {
            Err(input.error("Expected 'register', 'struct', or 'enum'"))
        }
    }
}

impl Parse for RegisterDefaultsBlock {
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

        Ok(RegisterDefaultsBlock { defaults })
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
        let name: Ident = input.parse()?;

        // Parse register attributes in parentheses
        let register_attrs = if input.peek(Paren) {
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
            attributes: Vec::new(), // Will be set by Definition::parse
            name,
            register_attrs,
            fields,
        })
    }
}

impl Parse for StructDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let name: Ident = input.parse()?;

        // Parse struct attributes in parentheses
        let struct_attrs = if input.peek(Paren) {
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

        Ok(StructDefinition {
            attributes: Vec::new(), // Will be set by Definition::parse
            name,
            struct_attrs,
            fields,
        })
    }
}

impl Parse for EnumDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let name: Ident = input.parse()?;
        input.parse::<Token![:]>()?;
        let underlying_type: Type = input.parse()?;

        // Optional size constraint
        let size_constraint = if input.peek(Brace) {
            let content;
            syn::braced!(content in input);
            let size_lit: LitInt = content.parse()?;
            Some(size_lit.base10_parse::<usize>()?)
        } else {
            None
        };

        // Parse variants block
        let variant_content;
        syn::braced!(variant_content in input);

        let mut variants = Vec::new();
        while !variant_content.is_empty() {
            variants.push(variant_content.parse()?);
        }

        Ok(EnumDefinition {
            attributes: Vec::new(), // Will be set by Definition::parse
            name,
            underlying_type,
            size_constraint,
            variants,
        })
    }
}

impl Parse for EnumVariant {
    fn parse(input: ParseStream) -> Result<Self> {
        let attributes = input.call(Attribute::parse_outer)?;

        let pattern: EnumPattern = input.parse()?;
        let name: Ident = input.parse()?;

        // Check for capture syntax (underlying)
        let capture_value = if input.peek(Paren) {
            let content;
            syn::parenthesized!(content in input);
            let _underlying_type: Type = content.parse()?;
            true
        } else {
            false
        };

        // Optional comma
        if input.peek(Token![,]) {
            input.parse::<Token![,]>()?;
        }

        Ok(EnumVariant {
            attributes,
            pattern,
            name,
            capture_value,
            representative: LitInt::new("0", input.span()),
        })
    }
}

impl Parse for EnumPattern {
    fn parse(input: ParseStream) -> Result<Self> {
        if input.peek(Token![_]) {
            input.parse::<Token![_]>()?;
            return Ok(EnumPattern::Wildcard);
        }

        fn parse_pattern_item(input: ParseStream) -> Result<EnumPattern> {
            let start: LitInt = input.parse()?;
            if input.peek(Token![..=]) {
                input.parse::<Token![..=]>()?;
                let end: LitInt = input.parse()?;
                Ok(EnumPattern::RangeInclusive(start, end))
            } else if input.peek(Token![..]) {
                input.parse::<Token![..]>()?;
                let end: LitInt = input.parse()?;
                Ok(EnumPattern::Range(start, end))
            } else {
                Ok(EnumPattern::Single(start))
            }
        }

        // Parse the first pattern item
        let first_item = parse_pattern_item(input)?;

        // Check if there are more items separated by |
        if input.peek(Token![|]) {
            let mut items = vec![first_item];
            while input.peek(Token![|]) {
                input.parse::<Token![|]>()?;
                items.push(parse_pattern_item(input)?);
            }
            Ok(EnumPattern::Multiple(items))
        } else {
            Ok(first_item)
        }
    }
}

impl Parse for FieldDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let attributes = input.call(Attribute::parse_outer)?;

        // Field name (or _ for reserved)
        let name = if input.peek(Ident) {
            // A named field like field1 or _reserved
            input.parse()?
        } else if input.peek(Token![_]) {
            let underscore = input.parse::<Token![_]>()?;
            Ident::new("_reserved_auto", underscore.span)
        } else {
            return Err(input.error("Missing field identifier or _ for reserved field"));
        };

        input.parse::<Token![:]>()?;
        let field_type: Type = input.parse()?;

        // Check for size constraint syntax: {size}
        let size_constraint = if input.peek(Brace) {
            let content;
            syn::braced!(content in input);
            let size_lit: LitInt = content.parse()?;
            Some(size_lit.base10_parse::<usize>()?)
        } else {
            None
        };

        // Optional bit pattern (mutually exclusive with size constraint)
        let bit_pattern = if input.peek(Bracket) {
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
        let units = if input.peek(Brace) && size_constraint.is_none() {
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
            let start_val = start.base10_parse::<usize>()?;

            if content.peek(Token![..=]) {
                content.parse::<Token![..=]>()?;
                let end: LitInt = content.parse()?;
                let end_val = end.base10_parse::<usize>()?;
                ranges.push(BitRange::RangeInclusive(start_val, end_val));
            } else if content.peek(Token![..]) {
                content.parse::<Token![..]>()?;
                let end: LitInt = content.parse()?;
                let end_val = end.base10_parse::<usize>()?;
                ranges.push(BitRange::Range(start_val, end_val));
            } else {
                ranges.push(BitRange::Single(start_val));
            }

            if content.peek(Token![,]) {
                content.parse::<Token![,]>()?;
            }
        }

        Ok(BitPattern {
            ranges,
            span: input.span(),
        })
    }
}
