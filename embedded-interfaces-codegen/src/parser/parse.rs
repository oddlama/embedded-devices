//! Parser implementations

use super::ast::*;
use syn::{
    Attribute, Expr, Ident, Lit, LitInt, Token, Type,
    parse::{Parse, ParseStream, Result},
    punctuated::Punctuated,
    spanned::Spanned,
    token::{Brace, Bracket, Paren},
};

impl Parse for InterfaceObjectsDefinition {
    fn parse(input: ParseStream) -> Result<Self> {
        let mut register_defaults = None;
        let mut register_devices = None;
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
                    if ident == "register_devices" {
                        if register_devices.is_some() {
                            return Err(input.error("Multiple devices blocks are not allowed"));
                        }
                        register_devices = Some(input.parse()?);
                        continue;
                    }
                }
            }

            // Parse definition (register, struct, or enum)
            definitions.push(input.parse()?);
        }

        Ok(InterfaceObjectsDefinition {
            register_defaults,
            register_devices,
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

impl Parse for RegisterDevicesBlock {
    fn parse(input: ParseStream) -> Result<Self> {
        let _devices_token: Ident = input.parse()?;
        let content;
        syn::bracketed!(content in input);

        let devices = Punctuated::<Ident, Token![,]>::parse_terminated(&content)?
            .into_iter()
            .collect();

        Ok(RegisterDevicesBlock { devices })
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
        // Optional size constraint - use lookahead to check if it's a size constraint
        let size_constraint = if input.peek(Brace) {
            // Fork the input to look ahead without consuming
            let lookahead = input.fork();
            let lookahead_content;
            syn::braced!(lookahead_content in lookahead);

            // Try to parse as a LitInt - if this succeeds and the content is empty after,
            // it's likely a size constraint
            if let Ok(_lit) = lookahead_content.parse::<LitInt>() {
                if lookahead_content.is_empty() {
                    // It's a size constraint, now consume from the real input
                    let content;
                    syn::braced!(content in input);
                    let size_lit: LitInt = content.parse()?;
                    let constraint_bits = size_lit.base10_parse::<usize>()?;

                    match underlying_type {
                        Type::Path(ref type_path) => {
                            if let Some(ident) = type_path.path.get_ident() {
                                let type_name = ident.to_string();
                                let type_bits = match type_name.as_str() {
                                    "u8" => 8,
                                    "u16" => 16,
                                    "u32" => 32,
                                    "u64" => 64,
                                    "u128" => 128,
                                    _ => return Err(input.error("unsupported underlying type for enum")),
                                };
                                if constraint_bits > type_bits {
                                    return Err(input.error(format!(
                                        "size constraint {{{constraint_bits}}} is too large for underlying type {type_name}"
                                    )));
                                }
                            }
                        }
                        _ => return Err(input.error("unsupported underlying type for enum")),
                    }

                    Some(constraint_bits)
                } else {
                    None
                }
            } else {
                None
            }
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
            attributes: Vec::new(), // Will be set later by Definition::parse
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
            Some(content.parse()?)
        } else {
            None
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

        // Check for size constraint {size} or endianness constraint ({be} or {le})
        let mut bit_constraint = None;
        if input.peek(Brace) {
            let content;
            syn::braced!(content in input);

            let lookahead = content.lookahead1();
            if lookahead.peek(Ident) {
                let ident: Ident = content.parse()?; // consume
                bit_constraint = match ident.to_string().as_str() {
                    "le" => Some(BitConstraint::Endianness(ident.span(), Endianness::Little)),
                    "be" => Some(BitConstraint::Endianness(ident.span(), Endianness::Big)),
                    _ => return Err(content.error("Expected size literal, 'be' or 'le'")),
                }
            } else {
                let size_lit: LitInt = content.parse()?;
                bit_constraint = Some(BitConstraint::Size(size_lit.clone(), size_lit.base10_parse::<usize>()?));
            }
        }

        // Optional bit pattern (mutually exclusive with size constraint)
        if input.peek(Bracket) {
            if bit_constraint.is_some() {
                return Err(input.error("Cannot specify both size/endianness constraint and a bit pattern [n..m]"));
            }
            bit_constraint = Some(BitConstraint::Pattern(input.parse()?));
        }

        // Optional default value
        let default_value = if input.peek(Token![=]) {
            input.parse::<Token![=]>()?;
            Some(input.parse()?)
        } else {
            None
        };

        // Optional units block
        let units = if input.peek(Brace) {
            // Ensure name start with raw_
            if !name.to_string().starts_with("raw_") {
                return Err(input.error("When providing units for a field, you must prefix the field name with `raw_`. Additional accessors without the raw prefix will be generated for the specified unit."));
            }

            Some(input.parse()?)
        } else {
            None
        };

        // Optional comma
        if input.peek(Token![,]) {
            input.parse::<Token![,]>()?;
        }

        let bit_constraint = bit_constraint.unwrap_or(BitConstraint::Endianness(field_type.span(), Endianness::Big));
        Ok(FieldDefinition {
            attributes,
            name,
            field_type,
            bit_constraint,
            default_value,
            units,
        })
    }
}

impl Parse for UnitsBlock {
    fn parse(input: ParseStream) -> Result<Self> {
        let content;
        syn::braced!(content in input);

        let mut quantity = None;
        let mut unit = None;
        let mut lsb = None;
        let mut from_raw = None;
        let mut into_raw = None;

        while !content.is_empty() {
            let field_name: Ident = content.parse()?;
            content.parse::<Token![:]>()?;

            match field_name.to_string().as_str() {
                "quantity" => {
                    if quantity.is_some() {
                        return Err(content.error("Duplicate quantity field"));
                    }
                    quantity = Some(content.parse()?);
                }
                "unit" => {
                    if unit.is_some() {
                        return Err(content.error("Duplicate unit field"));
                    }
                    unit = Some(content.parse()?);
                }
                "lsb" => {
                    if lsb.is_some() {
                        return Err(content.error("Duplicate lsb field"));
                    }
                    if from_raw.is_some() || into_raw.is_some() {
                        return Err(content.error("Cannot mix lsb with from_raw/into_raw"));
                    }
                    // Parse lsb as numerator / denominator
                    let numerator: Lit = content.parse()?;
                    content.parse::<Token![/]>()?;
                    let denominator: Lit = content.parse()?;

                    macro_rules! check_lit {
                        ($x:ident) => {{
                            let ok = match $x {
                                Lit::Int(ref lit_int) => lit_int.suffix() != "",
                                Lit::Float(ref lit_float) => lit_float.suffix() != "",
                                _ => false,
                            };

                            if ok {
                                Ok(())
                            } else {
                                Err(syn::Error::new_spanned(
                                    &$x,
                                    concat!(
                                        "The lsb ",
                                        stringify!($x),
                                        " must be an integer or float literal with explicit type suffix!"
                                    ),
                                ))
                            }
                        }};
                    }

                    check_lit!(numerator)?;
                    check_lit!(denominator)?;

                    lsb = Some((numerator, denominator));
                }
                "from_raw" => {
                    if from_raw.is_some() {
                        return Err(content.error("Duplicate from_raw field"));
                    }
                    if lsb.is_some() {
                        return Err(content.error("Cannot mix from_raw/into_raw with lsb"));
                    }
                    from_raw = Some(content.parse()?);
                }
                "into_raw" => {
                    if into_raw.is_some() {
                        return Err(content.error("Duplicate into_raw field"));
                    }
                    if lsb.is_some() {
                        return Err(content.error("Cannot mix from_raw/into_raw with lsb"));
                    }
                    into_raw = Some(content.parse()?);
                }
                _ => {
                    return Err(
                        content.error("Unknown units field. Expected: quantity, unit, lsb, from_raw, or into_raw")
                    );
                }
            }

            // Optional comma
            if content.peek(Token![,]) {
                content.parse::<Token![,]>()?;
            }
        }

        // Validate required fields
        let quantity = quantity.ok_or_else(|| input.error("quantity field is required"))?;
        let unit = unit.ok_or_else(|| input.error("unit field is required"))?;

        // Validate scale specification
        let scale = if let Some((numerator, denominator)) = lsb {
            ScaleSpec::Lsb { numerator, denominator }
        } else if from_raw.is_some() != into_raw.is_some() {
            return Err(input.error("Both from_raw and into_raw must be specified together"));
        } else if let (Some(from_raw), Some(into_raw)) = (from_raw, into_raw) {
            ScaleSpec::Custom { from_raw, into_raw }
        } else {
            return Err(input.error("Either lsb or both from_raw and into_raw must be specified"));
        };

        Ok(UnitsBlock { quantity, unit, scale })
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
