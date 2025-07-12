//! Bit pattern processing and validation
//!
//! This module handles the consolidation, normalization, and validation of bit patterns
//! for packed struct fields.

use proc_macro2::Ident;
use syn::Type;

use crate::parser::{BitPattern, BitRange, FieldDefinition};

/// A normalized bit range (always exclusive end)
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct NormalizedRange {
    pub start: u32,
    pub end: u32, // exclusive
}

impl NormalizedRange {
    pub fn new(start: u32, end: u32) -> syn::Result<Self> {
        if start >= end {
            return Err(syn::Error::new(
                proc_macro2::Span::call_site(),
                format!("Invalid bit range: {}..{} (start must be less than end)", start, end),
            ));
        }
        Ok(NormalizedRange { start, end })
    }

    pub fn size(&self) -> u32 {
        self.end - self.start
    }

    pub fn overlaps(&self, other: &NormalizedRange) -> bool {
        self.start < other.end && other.start < self.end
    }

    pub fn is_adjacent(&self, other: &NormalizedRange) -> bool {
        self.end == other.start || other.end == self.start
    }

    pub fn can_merge(&self, other: &NormalizedRange) -> bool {
        self.overlaps(other) || self.is_adjacent(other)
    }

    pub fn merge(&self, other: &NormalizedRange) -> NormalizedRange {
        NormalizedRange {
            start: self.start.min(other.start),
            end: self.end.max(other.end),
        }
    }
}

/// Processed field information with normalized bit patterns
#[derive(Debug, Clone)]
pub struct ProcessedField {
    pub field: FieldDefinition,
    pub normalized_ranges: Vec<NormalizedRange>,
}

/// Process and validate all fields' bit patterns
pub fn process_field_bit_patterns(
    parent: &Ident,
    fields: &[FieldDefinition],
    total_size_bits: u32,
) -> syn::Result<Vec<ProcessedField>> {
    let mut processed_fields = Vec::new();
    let mut next_auto_bit = 0u32;

    // First pass: process each field's bit pattern
    for field in fields {
        let processed = process_single_field(field, &mut next_auto_bit)?;
        processed_fields.push(processed);
    }

    // Second pass: validate no overlaps and complete coverage
    validate_bit_coverage(parent, &processed_fields, total_size_bits)?;

    Ok(processed_fields)
}

/// Process a single field's bit pattern
fn process_single_field(field: &FieldDefinition, next_auto_bit: &mut u32) -> syn::Result<ProcessedField> {
    let normalized_ranges = if let Some(bit_pattern) = &field.bit_pattern {
        // Explicit bit pattern provided
        let ranges = normalize_bit_pattern(bit_pattern)?;

        // Update next_auto_bit to be after the highest bit used
        if let Some(max_end) = ranges.iter().map(|r| r.end).max() {
            *next_auto_bit = (*next_auto_bit).max(max_end);
        }

        ranges
    } else {
        // Auto-generate bit pattern
        let field_size = infer_field_size_bits(&field.field_type)?;
        let start_bit = *next_auto_bit;
        let end_bit = start_bit + field_size;

        *next_auto_bit = end_bit;
        vec![NormalizedRange::new(start_bit, end_bit)?]
    };

    Ok(ProcessedField {
        field: field.clone(),
        normalized_ranges,
    })
}

/// Normalize a bit pattern by consolidating adjacent ranges
fn normalize_bit_pattern(bit_pattern: &BitPattern) -> syn::Result<Vec<NormalizedRange>> {
    let mut ranges = Vec::new();

    // Convert all bit ranges to normalized ranges
    for bit_range in &bit_pattern.ranges {
        match bit_range {
            BitRange::Single(bit) => {
                ranges.push(NormalizedRange::new(*bit, *bit + 1)?);
            }
            BitRange::Range(start, end) => {
                ranges.push(NormalizedRange::new(*start, *end)?);
            }
            BitRange::RangeInclusive(start, end) => {
                ranges.push(NormalizedRange::new(*start, *end + 1)?);
            }
        }
    }

    // Sort ranges by start position
    ranges.sort();

    // Merge overlapping and adjacent ranges
    let mut merged: Vec<NormalizedRange> = Vec::new();
    for range in ranges {
        if let Some(last) = merged.last_mut() {
            if last.can_merge(&range) {
                *last = last.merge(&range);
            } else {
                merged.push(range);
            }
        } else {
            merged.push(range);
        }
    }

    Ok(merged)
}

/// Infer the size in bits for a field type
fn infer_field_size_bits(field_type: &Type) -> syn::Result<u32> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                match type_name.as_str() {
                    "bool" => Ok(1),
                    "u8" | "i8" => Ok(8),
                    "u16" | "i16" => Ok(16),
                    "u32" | "i32" | "f32" => Ok(32),
                    "u64" | "i64" | "f64" => Ok(64),
                    "u128" | "i128" => Ok(128),
                    "usize" | "isize" => Ok(32), // Assume 32-bit for embedded
                    _ => Err(syn::Error::new_spanned(
                        field_type,
                        format!(
                            "Cannot infer bit size for type '{}'. Please specify an explicit bit pattern.",
                            type_name
                        ),
                    )),
                }
            } else {
                Err(syn::Error::new_spanned(
                    field_type,
                    "Cannot infer bit size for complex type. Please specify an explicit bit pattern.",
                ))
            }
        }
        Type::Array(array_type) => {
            let element_size = infer_field_size_bits(&array_type.elem)?;
            if let syn::Expr::Lit(syn::ExprLit {
                lit: syn::Lit::Int(lit_int),
                ..
            }) = &array_type.len
            {
                let array_len = lit_int.base10_parse::<u32>()?;
                Ok(element_size * array_len)
            } else {
                Err(syn::Error::new_spanned(
                    array_type,
                    "Cannot infer bit size for array with non-literal length. Please specify an explicit bit pattern.",
                ))
            }
        }
        _ => Err(syn::Error::new_spanned(
            field_type,
            "Cannot infer bit size for this type. Please specify an explicit bit pattern.",
        )),
    }
}

/// Validate that all bits are covered exactly once and match the expected size
fn validate_bit_coverage(parent: &Ident, processed_fields: &[ProcessedField], total_size_bits: u32) -> syn::Result<()> {
    let mut all_ranges = Vec::new();

    // Collect all ranges from all fields
    for processed in processed_fields {
        for range in &processed.normalized_ranges {
            all_ranges.push(range.clone());
        }
    }

    // Sort ranges by start position
    all_ranges.sort();

    // Check for overlaps
    for i in 1..all_ranges.len() {
        if all_ranges[i - 1].overlaps(&all_ranges[i]) {
            return Err(syn::Error::new_spanned(
                parent,
                format!(
                    "Bit range overlap detected: {}..{} and {}..{}",
                    all_ranges[i - 1].start,
                    all_ranges[i - 1].end,
                    all_ranges[i].start,
                    all_ranges[i].end
                ),
            ));
        }
    }

    // Merge adjacent ranges to find gaps
    let mut merged: Vec<NormalizedRange> = Vec::new();
    for range in all_ranges {
        if let Some(last) = merged.last_mut() {
            if last.is_adjacent(&range) {
                *last = last.merge(&range);
            } else {
                merged.push(range);
            }
        } else {
            merged.push(range);
        }
    }

    // Check for complete coverage
    if merged.len() != 1 {
        return Err(syn::Error::new_spanned(
            parent,
            format!(
                "Bit coverage is not contiguous. Found {} separate ranges when exactly one is expected.",
                merged.len()
            ),
        ));
    }

    let coverage = &merged[0];
    if coverage.start != 0 {
        return Err(syn::Error::new_spanned(
            parent,
            format!(
                "Bit coverage does not start at 0. Found start at bit {}.",
                coverage.start
            ),
        ));
    }

    if coverage.end != total_size_bits {
        return Err(syn::Error::new_spanned(
            parent,
            format!(
                "Bit coverage does not match expected size. Expected {} bits, found {} bits.",
                total_size_bits, coverage.end
            ),
        ));
    }

    Ok(())
}

/// Generate documentation string for normalized bit ranges
pub fn generate_bit_pattern_doc(ranges: &[NormalizedRange]) -> String {
    let range_strs: Vec<String> = ranges
        .iter()
        .map(|r| {
            if r.size() == 1 {
                format!("{}", r.start)
            } else {
                format!("{}..{}", r.start, r.end)
            }
        })
        .collect();

    format!("Bits: [{}]", range_strs.join(", "))
}
