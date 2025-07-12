//! Abstract Syntax Tree definitions

use syn::{Attribute, Expr, Ident, LitInt, LitStr, Type};

/// Top-level structure for the entire registers! block
#[derive(Debug, Clone)]
pub struct RegistersDefinition {
    pub defaults: Option<DefaultsBlock>,
    pub devices: Option<DevicesBlock>,
    pub registers: Vec<RegisterDefinition>,
}

/// Defaults block: defaults { key = value, ... }
#[derive(Debug, Clone)]
pub struct DefaultsBlock {
    pub defaults: Vec<RegisterAttr>,
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
    pub register_attrs: Vec<RegisterAttr>,
    pub fields: Vec<FieldDefinition>,
}

/// Register attributes like addr, mode, size, etc.
#[derive(Debug, Clone)]
pub struct RegisterAttr {
    pub name: Ident,
    pub value: RegisterAttrValue,
}

#[derive(Debug, Clone)]
pub enum RegisterAttrValue {
    Int(LitInt),
    String(LitStr),
    Ident(Ident),
}

/// Field definition within a register
#[derive(Debug, Clone)]
pub struct FieldDefinition {
    pub attributes: Vec<Attribute>,
    pub name: Option<Ident>, // None for reserved fields like _
    pub field_type: Type,
    pub bit_pattern: Option<BitPattern>,
    pub default_value: Option<Expr>,
    pub units: Option<UnitsBlock>,
}

/// Bit pattern specification
#[derive(Debug, Clone)]
pub struct BitPattern {
    pub ranges: Vec<BitRange>,
}

/// Individual bit range
#[derive(Debug, Clone)]
pub enum BitRange {
    Single(u32),              // [5]
    Range(u32, u32),          // [5..8]
    RangeInclusive(u32, u32), // [5..=8]
}

/// Units block (placeholder for now)
#[derive(Debug, Clone)]
pub struct UnitsBlock {
    // TODO: Parse quantity, unit, scale, etc.
}

impl RegistersDefinition {
    /// Get all register attributes including defaults
    pub fn get_effective_attrs(&self, register: &RegisterDefinition) -> Vec<RegisterAttr> {
        let mut attrs = Vec::new();

        // Add defaults first
        if let Some(defaults) = &self.defaults {
            for attr in &defaults.defaults {
                attrs.push(attr.clone());
            }
        }

        // Add register-specific attributes (these override defaults)
        for attr in &register.register_attrs {
            // Remove any existing attribute with the same name
            attrs.retain(|a| a.name != attr.name);
            attrs.push(attr.clone());
        }

        attrs
    }
}
