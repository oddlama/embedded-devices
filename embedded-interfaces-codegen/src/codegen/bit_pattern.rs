//! Bit pattern processing and validation
//!
//! This module handles the consolidation, normalization, and validation of bit patterns
//! for packed struct fields.

use proc_macro2::Ident;
use std::collections::VecDeque;
use syn::Type;

use crate::parser::{BitPattern, BitRange, FieldDefinition};

/// A normalized bit range (always exclusive end)
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct NormalizedRange {
    pub start: usize,
    pub end: usize, // exclusive
}

impl NormalizedRange {
    pub fn new(start: usize, end: usize) -> Result<Self, String> {
        if start == end {
            return Err(format!(
                "Invalid empty bit pattern: {}..{} (must cover at least 1 bit)",
                start, end
            ));
        }
        if start >= end {
            return Err(format!(
                "Invalid bit range: {}..{} (start must be less than end)",
                start, end
            ));
        }
        Ok(NormalizedRange { start, end })
    }

    pub fn size(&self) -> usize {
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

/// A tiny conversion trait so both NormalizedRange and &NormalizedRange work.
pub trait IntoNormalizedRange {
    fn into_normalized(self) -> NormalizedRange;
}

impl IntoNormalizedRange for NormalizedRange {
    #[inline]
    fn into_normalized(self) -> NormalizedRange {
        self
    }
}

impl IntoNormalizedRange for &NormalizedRange {
    #[inline]
    fn into_normalized(self) -> NormalizedRange {
        self.clone()
    }
}

pub trait IterChunksExactBits: Iterator + Sized {
    fn chunks_exact_bits(self, bits: usize) -> ChunksExactBits<Self>
    where
        Self::Item: IntoNormalizedRange;
}

impl<I> IterChunksExactBits for I
where
    I: Iterator,
    I::Item: IntoNormalizedRange,
{
    fn chunks_exact_bits(self, bits: usize) -> ChunksExactBits<Self> {
        ChunksExactBits::new(self, bits)
    }
}

pub struct ChunksExactBits<I> {
    inner: I,
    buf: VecDeque<NormalizedRange>,
    bits: usize,
}

impl<I> ChunksExactBits<I>
where
    I: Iterator,
    I::Item: IntoNormalizedRange,
{
    fn new(inner: I, bits: usize) -> Self {
        assert!(bits > 0);
        Self {
            inner,
            buf: VecDeque::new(),
            bits,
        }
    }
}

impl<I> Iterator for ChunksExactBits<I>
where
    I: Iterator,
    I::Item: IntoNormalizedRange,
{
    type Item = Vec<NormalizedRange>;

    fn next(&mut self) -> Option<Self::Item> {
        let needed = self.bits;
        let mut taken = Vec::new();
        let mut remaining = needed;

        while remaining > 0 {
            if self.buf.is_empty() {
                if let Some(next) = self.inner.next() {
                    // convert into NormalizedRange
                    self.buf.push_back(next.into_normalized());
                } else {
                    break;
                }
            }

            let mut cur = self.buf.pop_front().unwrap();
            if cur.size() <= remaining {
                remaining -= cur.size();
                taken.push(cur);
            } else {
                let split = cur.start + remaining;
                taken.push(NormalizedRange {
                    start: cur.start,
                    end: split,
                });
                cur.start = split;
                self.buf.push_front(cur);
                remaining = 0;
            }
        }

        if taken.is_empty() { None } else { Some(taken) }
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
    total_size_bits: usize,
) -> syn::Result<Vec<ProcessedField>> {
    let mut processed_fields = Vec::new();
    let mut next_auto_bit = 0usize;

    // First pass: process each field's bit pattern
    for field in fields {
        let processed = process_single_field(field, &mut next_auto_bit)?;
        processed_fields.push(processed);
    }

    // Second pass: validate no overlaps and complete coverage
    validate_bit_coverage(parent, &processed_fields, total_size_bits)?;

    Ok(processed_fields)
}

/// Process a single field's bit pattern (updated to handle size constraints)
fn process_single_field(field: &FieldDefinition, next_auto_bit: &mut usize) -> syn::Result<ProcessedField> {
    let normalized_ranges = if let Some(bit_pattern) = &field.bit_pattern {
        // Explicit bit pattern provided
        let ranges = normalize_bit_pattern(bit_pattern).map_err(|e| syn::Error::new(bit_pattern.span, e))?;

        // Update next_auto_bit to be after the highest bit used
        if let Some(max_end) = ranges.iter().map(|r| r.end).max() {
            *next_auto_bit = (*next_auto_bit).max(max_end);
        }

        ranges
    } else if let Some(size_constraint) = field.size_constraint {
        // Size constraint syntax e.g. u8{3} - use constraint size instead of inferred size
        let start_bit = *next_auto_bit;
        let end_bit = start_bit + size_constraint;

        // Validate that the constraint makes sense for the field type
        validate_size_constraint(&field.field_type, size_constraint)?;

        *next_auto_bit = end_bit;
        vec![NormalizedRange::new(start_bit, end_bit).map_err(|e| syn::Error::new_spanned(field.size_constraint, e))?]
    } else {
        // Auto-generate bit pattern using inferred size
        let field_size = infer_field_size_bits(&field.field_type)?;
        let start_bit = *next_auto_bit;
        let end_bit = start_bit + field_size;

        *next_auto_bit = end_bit;
        vec![NormalizedRange::new(start_bit, end_bit).map_err(|e| syn::Error::new_spanned(&field.field_type, e))?]
    };

    Ok(ProcessedField {
        field: field.clone(),
        normalized_ranges,
    })
}
/// Validate that a size constraint makes sense for the given field type
fn validate_size_constraint(field_type: &Type, size_constraint: usize) -> syn::Result<()> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();
                let max_size = match type_name.as_str() {
                    "bool" => 1,
                    "u8" | "i8" => 8,
                    "u16" | "i16" => 16,
                    "u32" | "i32" | "f32" => 32,
                    "u64" | "i64" | "f64" => 64,
                    "u128" | "i128" => 128,
                    "usize" | "isize" => 32, // Assume 32-bit for embedded
                    _ => {
                        // For unknown types, we can't validate - assume it's valid
                        return Ok(());
                    }
                };

                if size_constraint > max_size {
                    return Err(syn::Error::new_spanned(
                        field_type,
                        format!(
                            "Size constraint {} bits exceeds maximum size {} bits for type '{}'",
                            size_constraint, max_size, type_name
                        ),
                    ));
                }

                if size_constraint == 0 {
                    return Err(syn::Error::new_spanned(
                        field_type,
                        "Size constraint must be greater than 0",
                    ));
                }
            }
        }
        Type::Array(array_type) => {
            let element_size = infer_field_size_bits(&array_type.elem)?;
            if let syn::Expr::Lit(syn::ExprLit {
                lit: syn::Lit::Int(lit_int),
                ..
            }) = &array_type.len
            {
                let array_len = lit_int.base10_parse::<usize>()?;
                let max_size = element_size * array_len;

                if size_constraint > max_size {
                    return Err(syn::Error::new_spanned(
                        field_type,
                        format!(
                            "Size constraint {} bits exceeds maximum size {} bits for array type",
                            size_constraint, max_size
                        ),
                    ));
                }

                if size_constraint == 0 {
                    return Err(syn::Error::new_spanned(
                        field_type,
                        "Size constraint must be greater than 0",
                    ));
                }
            }
        }
        _ => {
            // For other types, we can't validate - assume it's valid
            // This includes custom types that might have their own size semantics
        }
    }

    Ok(())
}

/// Normalize a bit pattern by consolidating adjacent ranges
fn normalize_bit_pattern(bit_pattern: &BitPattern) -> Result<Vec<NormalizedRange>, String> {
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
fn infer_field_size_bits(field_type: &Type) -> syn::Result<usize> {
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
                let array_len = lit_int.base10_parse::<usize>()?;
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
fn validate_bit_coverage(
    parent: &Ident,
    processed_fields: &[ProcessedField],
    total_size_bits: usize,
) -> syn::Result<()> {
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
    if merged.is_empty() {
        if total_size_bits != 0 {
            return Err(syn::Error::new_spanned(
                parent,
                format!(
                    "Bit coverage does not match expected size. Expected {} bits, found 0 bits.",
                    total_size_bits
                ),
            ));
        }

        return Ok(());
    }

    if merged.len() != 1 {
        return Err(syn::Error::new_spanned(
            parent,
            format!(
                "Bit coverage is not contiguous. Found {} separate ranges when exactly one is expected. {:#?}",
                merged.len(),
                merged
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
