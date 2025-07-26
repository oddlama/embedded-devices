//! Abstract Syntax Tree definitions

use proc_macro2::Span;
use syn::{Attribute, Expr, Ident, LitInt, Type};

pub(crate) const ALLOWED_REGISTER_ATTRS: &[&str] = &["addr", "mode", "size", "i2c_codec", "spi_codec", "codec_error"];
pub(crate) const ALLOWED_STRUCT_ATTRS: &[&str] = &["size"];

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
    pub capture_value: Option<Type>, // Some if variant has Variant(Type) syntax
    pub representative: LitInt,      // A representative value for this variant
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

/// Units block with quantity, unit, and scale information
#[derive(Debug, Clone)]
pub struct UnitsBlock {
    pub quantity: Type,
    pub unit: Type,
    pub scale: ScaleSpec,
}

/// Scale specification - either LSB rational or custom conversion functions
#[derive(Debug, Clone)]
pub enum ScaleSpec {
    /// LSB specified as numerator/denominator rational number
    Lsb { numerator: LitInt, denominator: LitInt },
    /// Custom conversion functions
    Custom {
        from_raw: Expr, // |x| expression to convert from raw to scaled
        into_raw: Expr, // |x| expression to convert from scaled to raw
    },
}

impl From<RegisterDefinition> for StructDefinition {
    fn from(value: RegisterDefinition) -> Self {
        StructDefinition {
            attributes: value.attributes,
            name: value.name,
            struct_attrs: value
                .register_attrs
                .into_iter()
                .filter(|x| ALLOWED_STRUCT_ATTRS.contains(&x.name.to_string().as_str()))
                .collect(),
            fields: value.fields,
        }
    }
}

impl InterfaceObjectsDefinition {
    /// Get a definition by name if it exists.
    pub fn get_definition(&self, name: &str) -> Option<&Definition> {
        self.definitions.iter().find(|x| match x {
            Definition::Register(register_definition) => register_definition.name == name,
            Definition::Struct(struct_definition) => struct_definition.name == name,
            Definition::Enum(enum_definition) => enum_definition.name == name,
        })
    }

    /// Get all register attributes including defaults
    /// This method validates that only allowed attributes are used for registers
    pub fn get_effective_register_attrs(&self, register: &RegisterDefinition) -> syn::Result<Vec<Attr>> {
        let mut attrs = Vec::new();

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

impl EnumPattern {
    /// Returns the ranges of values captured by this pattern as (start, end) inclusive pairs
    pub fn captured_ranges(&self) -> syn::Result<Vec<(u128, u128)>> {
        match self {
            EnumPattern::Single(lit) => {
                let value = lit.base10_parse::<u128>()?;
                Ok(vec![(value, value)])
            }
            EnumPattern::Range(start, end) => {
                let start_val = start.base10_parse::<u128>()?;
                let end_val = end.base10_parse::<u128>()?;
                if start_val < end_val {
                    Ok(vec![(start_val, end_val - 1)])
                } else {
                    Err(syn::Error::new_spanned(
                        start,
                        "The start of a range must be smaller than its end",
                    ))
                }
            }
            EnumPattern::RangeInclusive(start, end) => {
                let start_val = start.base10_parse::<u128>().unwrap_or(0);
                let end_val = end.base10_parse::<u128>().unwrap_or(0);
                if start_val <= end_val {
                    Ok(vec![(start_val, end_val)])
                } else {
                    Err(syn::Error::new_spanned(
                        start,
                        "The start of an inclusive range must be smaller than or equal to its end",
                    ))
                }
            }
            EnumPattern::Multiple(patterns) => {
                let mut ranges = Vec::new();
                for pattern in patterns {
                    ranges.extend(pattern.captured_ranges()?);
                }
                Ok(merge_ranges(ranges))
            }
            EnumPattern::Wildcard => {
                // Wildcard doesn't capture specific values - it's determined by exclusion
                Ok(vec![])
            }
        }
    }

    /// Returns a representative value for this pattern
    pub fn representative(&self) -> Option<u128> {
        match self {
            EnumPattern::Single(lit) => lit.base10_parse::<u128>().ok(),
            EnumPattern::Range(start, _) => start.base10_parse::<u128>().ok(),
            EnumPattern::RangeInclusive(start, _) => start.base10_parse::<u128>().ok(),
            EnumPattern::Multiple(patterns) => {
                // Return the representative of the first pattern
                patterns.first().and_then(|p| p.representative())
            }
            EnumPattern::Wildcard => None, // Will be set later based on available values
        }
    }
}

/// Merge overlapping ranges and sort them
fn merge_ranges(mut ranges: Vec<(u128, u128)>) -> Vec<(u128, u128)> {
    if ranges.is_empty() {
        return ranges;
    }

    // Sort ranges by start value
    ranges.sort_by_key(|&(start, _)| start);

    let mut merged = Vec::new();
    let mut current = ranges[0];

    for &(start, end) in &ranges[1..] {
        if start <= current.1 + 1 {
            // Ranges overlap or are adjacent, merge them
            current.1 = current.1.max(end);
        } else {
            // No overlap, push current and start new range
            merged.push(current);
            current = (start, end);
        }
    }
    merged.push(current);
    merged
}

/// Checks if two lists of ranges overlap. Both input lists must be sorted by range start
fn ranges_overlap(ranges1: &[(u128, u128)], ranges2: &[(u128, u128)]) -> Option<u128> {
    let mut i = 0;
    let mut j = 0;

    while i < ranges1.len() && j < ranges2.len() {
        let (start1, end1) = ranges1[i];
        let (start2, end2) = ranges2[j];

        // Check for overlap
        if start1 <= end2 && start2 <= end1 {
            return Some(start1.max(start2));
        }

        // Move the pointer that ends first
        if end1 < end2 {
            i += 1;
        } else {
            j += 1;
        }
    }

    None
}

/// Calculate the complement of ranges within [0, max_value]
fn complement_ranges(ranges: &[(u128, u128)], max_value: u128) -> Vec<(u128, u128)> {
    if ranges.is_empty() {
        return vec![(0, max_value)];
    }

    let mut complement = Vec::new();
    let mut current = 0;

    for &(start, end) in ranges {
        if current < start {
            complement.push((current, start - 1));
        }
        current = end + 1;
    }

    if current <= max_value {
        complement.push((current, max_value));
    }

    complement
}

impl EnumDefinition {
    /// Process variants to validate non-overlapping patterns and set representatives
    pub fn process_variants(&mut self) -> syn::Result<()> {
        let max_value = (1u128 << self.get_effective_bit_size()?) - 1;
        let mut all_captured_ranges = Vec::new();
        let mut wildcard_variant_index = None;

        // First pass: collect all explicitly captured ranges and find wildcard
        for (i, variant) in self.variants.iter().enumerate() {
            if matches!(variant.pattern, EnumPattern::Wildcard) {
                if wildcard_variant_index.is_some() {
                    return Err(syn::Error::new_spanned(
                        &variant.name,
                        "Multiple wildcard variants are not allowed",
                    ));
                }
                wildcard_variant_index = Some(i);
            } else {
                let captured_ranges = variant.pattern.captured_ranges()?;

                // Check for overlaps with previously captured ranges
                if let Some(overlap_value) = ranges_overlap(&all_captured_ranges, &captured_ranges) {
                    return Err(syn::Error::new_spanned(
                        &variant.name,
                        format!("Value {} is captured by multiple variants", overlap_value),
                    ));
                }

                all_captured_ranges.extend(captured_ranges);
            }
        }

        // Merge all captured ranges
        all_captured_ranges = merge_ranges(all_captured_ranges);

        // Calculate wildcard ranges (complement of captured ranges)
        let wildcard_ranges = complement_ranges(&all_captured_ranges, max_value);

        if let Some(wildcard_idx) = wildcard_variant_index {
            // Validate wildcard captures at least one value
            if wildcard_ranges.is_empty() {
                return Err(syn::Error::new_spanned(
                    &self.variants[wildcard_idx].name,
                    "Wildcard variant must capture at least one value",
                ));
            }
        } else {
            // No wildcard: Ensure there are no unmatched values
            if !wildcard_ranges.is_empty() {
                return Err(syn::Error::new_spanned(
                    &self.name,
                    format!(
                        "An enum without wildcard variant must cover all values. Uncovered value ranges: {wildcard_ranges:?}"
                    ),
                ));
            }
        }

        // Second pass: set representatives
        for variant in self.variants.iter_mut() {
            let representative_value = if matches!(variant.pattern, EnumPattern::Wildcard) {
                // Use the first available wildcard value
                wildcard_ranges.first().map(|&(start, _)| start).unwrap_or(0)
            } else {
                variant.pattern.representative().unwrap_or(0)
            };

            // Create a LitInt for the representative value
            variant.representative = LitInt::new(&representative_value.to_string(), variant.name.span());
        }

        Ok(())
    }

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

pub fn get_effective_size(belongs_to: &Ident, attrs: &[Attr]) -> syn::Result<usize> {
    let size_attr = attrs.iter().find(|x| x.name == "size").ok_or_else(|| {
        syn::Error::new_spanned(
            belongs_to,
            "A size= attribute is required because the effective size must be known to the macro in advance",
        )
    })?;

    if let syn::Expr::Lit(syn::ExprLit {
        lit: syn::Lit::Int(lit_int),
        ..
    }) = &size_attr.value
    {
        lit_int.base10_parse::<usize>()
    } else {
        Err(syn::Error::new_spanned(
            &size_attr.value,
            "This expression must be a literal value because the effective size must be known to the macro in advance",
        ))
    }
}
