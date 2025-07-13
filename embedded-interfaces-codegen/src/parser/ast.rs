//! Abstract Syntax Tree definitions

use proc_macro2::Span;
use syn::{Attribute, Expr, Ident, Type};

/// Top-level structure for the entire registers! block
#[derive(Debug, Clone)]
pub struct RegistersDefinition {
    pub defaults: Option<DefaultsBlock>,
    pub devices: Option<DevicesBlock>,
    pub registers: Vec<RegisterDefinition>,
}

/// Generic defaults block: defaults { key = value, ... }
/// Values can be any expression (literals, identifiers, paths, etc.)
#[derive(Debug, Clone)]
pub struct DefaultsBlock {
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

/// Field definition within a register
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

impl RegistersDefinition {
    /// Get all register attributes including defaults
    /// This method validates that only allowed attributes are used for registers
    pub fn get_effective_attrs(&self, register: &RegisterDefinition) -> syn::Result<Vec<Attr>> {
        let mut attrs = Vec::new();
        const ALLOWED_REGISTER_ATTRS: &[&str] = &["addr", "mode", "size", "i2c_codec", "spi_codec", "codec_error"];

        // Add defaults first, validating each one
        if let Some(defaults) = &self.defaults {
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
}

impl FieldDefinition {
    /// Check if this field is reserved
    pub fn is_reserved(&self) -> bool {
        self.name.to_string().starts_with('_')
    }
}
