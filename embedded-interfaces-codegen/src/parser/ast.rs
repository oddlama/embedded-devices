//! Abstract Syntax Tree definitions

use proc_macro2::Span;
use syn::{Attribute, Expr, Ident, LitInt, Type};

/// Top-level structure for the entire interface_objects! block
#[derive(Debug, Clone)]
pub struct InterfaceObjectsDefinition {
    pub register_defaults: Option<RegisterDefaultsBlock>,
    pub devices: Option<DevicesBlock>,
    pub definitions: Vec<Definition>,
}

/// Definition can be a register, struct, or enum
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum Definition {
    Register(RegisterDefinition),
    Struct(StructDefinition),
    Enum(EnumDefinition),
}

/// Generic register defaults block: register_defaults { key = value, ... }
/// Values can be any expression (literals, identifiers, paths, etc.)
#[derive(Debug, Clone)]
pub struct RegisterDefaultsBlock {
    pub defaults: Vec<Attr>,
}

/// A single default entry
#[derive(Debug, Clone)]
pub struct Attr {
    pub name: Ident,
    pub value: Expr,
}

/// Devices block: devices [ DeviceA, DeviceB, ... ]
#[derive(Debug, Clone)]
pub struct DevicesBlock {
    pub devices: Vec<Ident>,
}

/// Individual register definition
#[derive(Debug, Clone)]
pub struct RegisterDefinition {
    pub attributes: Vec<Attribute>,
    pub name: Ident,
    pub register_attrs: Vec<Attr>,
    pub fields: Vec<FieldDefinition>,
}

/// Struct definition (similar to register but with limited attributes)
#[derive(Debug, Clone)]
pub struct StructDefinition {
    pub attributes: Vec<Attribute>,
    pub name: Ident,
    pub struct_attrs: Vec<Attr>,
    pub fields: Vec<FieldDefinition>,
}

/// Enum definition with pattern matching
#[derive(Debug, Clone)]
pub struct EnumDefinition {
    pub attributes: Vec<Attribute>,
    pub name: Ident,
    pub underlying_type: Type,
    pub size_constraint: Option<usize>,
    pub variants: Vec<EnumVariant>,
}

/// Individual enum variant
#[derive(Debug, Clone)]
pub struct EnumVariant {
    pub attributes: Vec<Attribute>,
    pub pattern: EnumPattern,
    pub name: Ident,
    pub capture_value: bool,    // true if variant has Variant(underlying) syntax
    pub representative: LitInt, // A representative value for this variant
}

/// Pattern for enum variants
#[derive(Debug, Clone)]
pub enum EnumPattern {
    Single(LitInt),                 // 0x58
    Range(LitInt, LitInt),          // 0x56..0x58
    RangeInclusive(LitInt, LitInt), // 0x56..=0x57
    Multiple(Vec<EnumPattern>),     // 0..5|7|8
    Wildcard,                       // _
}

/// Field definition within a register or struct
#[derive(Debug, Clone)]
pub struct FieldDefinition {
    pub attributes: Vec<Attribute>,
    pub name: Ident, // Reserved fields will start with _
    pub field_type: Type,
    pub bit_pattern: Option<BitPattern>,
    pub size_constraint: Option<usize>,
    pub default_value: Option<Expr>,
    pub units: Option<UnitsBlock>,
}

/// Bit pattern specification
#[derive(Debug, Clone)]
pub struct BitPattern {
    pub ranges: Vec<BitRange>,
    pub span: Span,
}

/// Individual bit range
#[derive(Debug, Clone)]
pub enum BitRange {
    Single(usize),                // [5]
    Range(usize, usize),          // [5..8]
    RangeInclusive(usize, usize), // [5..=8]
}

/// Units block (placeholder for now)
#[derive(Debug, Clone)]
pub struct UnitsBlock {
    // TODO: Parse quantity, unit, scale, etc.
}

impl InterfaceObjectsDefinition {
    /// Get all register attributes including defaults
    /// This method validates that only allowed attributes are used for registers
    pub fn get_effective_register_attrs(&self, register: &RegisterDefinition) -> syn::Result<Vec<Attr>> {
        let mut attrs = Vec::new();
        const ALLOWED_REGISTER_ATTRS: &[&str] = &["addr", "mode", "size", "i2c_codec", "spi_codec", "codec_error"];

        // Add defaults first, validating each one
        if let Some(defaults) = &self.register_defaults {
            for attr in &defaults.defaults {
                let attr_name = attr.name.to_string();
                if !ALLOWED_REGISTER_ATTRS.contains(&attr_name.as_str()) {
                    return Err(syn::Error::new_spanned(
                        &attr.name,
                        format!(
                            "Unknown register attribute '{}'. Allowed attributes are: {}",
                            attr_name,
                            ALLOWED_REGISTER_ATTRS.join(", ")
                        ),
                    ));
                }
                attrs.push(attr.clone());
            }
        }

        // Add register-specific attributes (these override defaults), validating each one
        for attr in &register.register_attrs {
            let attr_name = attr.name.to_string();
            if !ALLOWED_REGISTER_ATTRS.contains(&attr_name.as_str()) {
                return Err(syn::Error::new_spanned(
                    &attr.name,
                    format!(
                        "Unknown register attribute '{}'. Allowed attributes are: {}",
                        attr_name,
                        ALLOWED_REGISTER_ATTRS.join(", ")
                    ),
                ));
            }

            // Remove any existing attribute with the same name
            attrs.retain(|a| a.name != attr.name);
            attrs.push(attr.clone());
        }

        Ok(attrs)
    }

    /// Get all struct attributes (only size is allowed)
    pub fn get_effective_struct_attrs(&self, struct_def: &StructDefinition) -> syn::Result<Vec<Attr>> {
        const ALLOWED_STRUCT_ATTRS: &[&str] = &["size"];

        for attr in &struct_def.struct_attrs {
            let attr_name = attr.name.to_string();
            if !ALLOWED_STRUCT_ATTRS.contains(&attr_name.as_str()) {
                return Err(syn::Error::new_spanned(
                    &attr.name,
                    format!(
                        "Unknown struct attribute '{}'. Allowed attributes are: {}",
                        attr_name,
                        ALLOWED_STRUCT_ATTRS.join(", ")
                    ),
                ));
            }
        }

        Ok(struct_def.struct_attrs.clone())
    }
}

impl FieldDefinition {
    /// Check if this field is reserved
    pub fn is_reserved(&self) -> bool {
        self.name.to_string().starts_with('_')
    }
}

impl EnumDefinition {
    /// Get the default bit size for the underlying type
    pub fn get_default_bit_size(&self) -> syn::Result<usize> {
        // Extract the bit size from the underlying type
        if let Type::Path(type_path) = &self.underlying_type {
            if let Some(segment) = type_path.path.segments.last() {
                match segment.ident.to_string().as_str() {
                    "u8" => Ok(8),
                    "u16" => Ok(16),
                    "u32" => Ok(32),
                    "u64" => Ok(64),
                    "u128" => Ok(128),
                    _ => Err(syn::Error::new_spanned(
                        &self.underlying_type,
                        "Enum underlying type must be one of: u8, u16, u32, u64, u128",
                    )),
                }
            } else {
                Err(syn::Error::new_spanned(
                    &self.underlying_type,
                    "Invalid underlying type",
                ))
            }
        } else {
            Err(syn::Error::new_spanned(
                &self.underlying_type,
                "Enum underlying type must be a path type (u8, u16, etc.)",
            ))
        }
    }

    /// Get the effective bit size (either from constraint or default)
    pub fn get_effective_bit_size(&self) -> syn::Result<usize> {
        if let Some(size) = self.size_constraint {
            Ok(size)
        } else {
            self.get_default_bit_size()
        }
    }
}
