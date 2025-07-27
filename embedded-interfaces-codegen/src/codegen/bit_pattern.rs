//! Bit pattern processing and validation
//!
//! This module handles the consolidation, normalization, and validation of bit patterns
//! for packed struct fields.

use proc_macro2::{Ident, TokenStream};
use quote::{ToTokens, quote};
use std::collections::VecDeque;
use syn::Type;

use crate::parser::{
    BitConstraint, BitPattern, BitRange, Definition, Endianness, FieldDefinition, InterfaceObjectsDefinition,
    get_effective_size,
};

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

impl ProcessedField {
    pub(crate) fn generate_doc(&self, ranges: &[NormalizedRange]) -> TokenStream {
        // Generate field documentation
        let default_doc = if let Some(default_val) = &self.field.default_value {
            let doc = format!("Default: `{}`", default_val.to_token_stream());
            quote! {
                #[doc = #doc]
            }
        } else {
            quote! { #[doc = "Default: not specified"] }
        };

        // Generate bit pattern documentation
        let bit_pattern_doc = generate_bit_pattern_doc(ranges);
        quote! {
            #default_doc
            #[doc = ""]
            #[doc = #bit_pattern_doc]
        }
    }
}

/// Process and validate all fields' bit patterns
pub fn process_field_bit_patterns(
    interface_def: &InterfaceObjectsDefinition,
    parent: &Ident,
    fields: &[FieldDefinition],
    total_size_bits: usize,
) -> syn::Result<Vec<ProcessedField>> {
    let mut processed_fields = Vec::new();
    let mut next_auto_bit = 0usize;

    // First pass: process each field's bit pattern
    for field in fields {
        let processed = process_single_field(interface_def, field, &mut next_auto_bit)?;
        processed_fields.push(processed);
    }

    // Second pass: validate no overlaps and complete coverage
    validate_bit_coverage(parent, &processed_fields, total_size_bits)?;

    Ok(processed_fields)
}

/// Process a single field's bit pattern (updated to handle size constraints)
fn process_single_field(
    interface_def: &InterfaceObjectsDefinition,
    field: &FieldDefinition,
    next_auto_bit: &mut usize,
) -> syn::Result<ProcessedField> {
    let normalized_ranges = match &field.bit_constraint {
        Some(BitConstraint::Pattern(bit_pattern)) => {
            // Explicit bit pattern provided
            let ranges = normalize_bit_pattern(bit_pattern).map_err(|e| syn::Error::new(bit_pattern.span, e))?;

            // Update next_auto_bit to be after the highest bit used
            if let Some(max_end) = ranges.iter().map(|r| r.end).max() {
                *next_auto_bit = (*next_auto_bit).max(max_end);
            }

            ranges
        }
        Some(BitConstraint::Size(lit, size_constraint)) => {
            // Size constraint syntax e.g. u8{3} - use constraint size instead of inferred size
            let start_bit = *next_auto_bit;
            let end_bit = start_bit + size_constraint;

            // Validate that the constraint makes sense for the field type
            validate_size_constraint(interface_def, &field.field_type, *size_constraint)?;

            *next_auto_bit = end_bit;
            match field.endianness {
                Endianness::Little(span) => {
                    // split range every 8 bits and reverse
                    if (end_bit - start_bit) % 8 != 0 {
                        return Err(syn::Error::new(
                            span,
                            "The little endian constraint can only be used on types which are a multiple of 8 bits in size",
                        ));
                    }
                    let n_bytes = (end_bit - start_bit) / 8;
                    (0..n_bytes)
                        .rev()
                        .map(|first| {
                            NormalizedRange::new(start_bit + first * 8, start_bit + first * 8 + 8)
                                .map_err(|e| syn::Error::new(span, e))
                        })
                        .collect::<syn::Result<Vec<_>>>()?
                }
                Endianness::Big(_) => {
                    vec![NormalizedRange::new(start_bit, end_bit).map_err(|e| syn::Error::new_spanned(lit, e))?]
                }
            }
        }
        // Auto-generate bit pattern using inferred size
        None => {
            // Auto-generate bit pattern using inferred size
            let field_size = infer_field_size_bits(interface_def, &field.field_type)?;
            let start_bit = *next_auto_bit;
            let end_bit = start_bit + field_size;

            *next_auto_bit = end_bit;
            match field.endianness {
                Endianness::Little(span) => {
                    // split range every 8 bits and reverse
                    if (end_bit - start_bit) % 8 != 0 {
                        return Err(syn::Error::new(
                            span,
                            "The little endian constraint can only be used on types which are a multiple of 8 bits in size",
                        ));
                    }
                    let n_bytes = (end_bit - start_bit) / 8;
                    (0..n_bytes)
                        .rev()
                        .map(|first| {
                            NormalizedRange::new(start_bit + first * 8, start_bit + first * 8 + 8)
                                .map_err(|e| syn::Error::new(span, e))
                        })
                        .collect::<syn::Result<Vec<_>>>()?
                }
                Endianness::Big(span) => {
                    vec![NormalizedRange::new(start_bit, end_bit).map_err(|e| syn::Error::new(span, e))?]
                }
            }
        }
    };

    Ok(ProcessedField {
        field: field.clone(),
        normalized_ranges,
    })
}

/// Validate that a size constraint makes sense for the given field type
fn validate_size_constraint(
    interface_def: &InterfaceObjectsDefinition,
    field_type: &Type,
    size_constraint: usize,
) -> syn::Result<()> {
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
            let element_size = infer_field_size_bits(interface_def, &array_type.elem)?;
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

    Ok(normalize_ranges(ranges))
}

fn normalize_ranges(ranges: Vec<NormalizedRange>) -> Vec<NormalizedRange> {
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

    merged
}

/// Infer the size in bits for a field type
fn infer_field_size_bits(interface_def: &InterfaceObjectsDefinition, field_type: &Type) -> syn::Result<usize> {
    match field_type {
        Type::Path(type_path) => {
            if let Some(ident) = type_path.path.get_ident() {
                let type_name = ident.to_string();

                if let Some(custom_type) = type_name
                    .strip_suffix("Unpacked")
                    .and_then(|x| interface_def.get_definition(x))
                    .or_else(|| interface_def.get_definition(&type_name))
                {
                    match custom_type {
                        Definition::Register(register_definition) => Ok(8 * get_effective_size(
                            &register_definition.name,
                            &interface_def.get_effective_register_attrs(register_definition)?,
                        )?),
                        Definition::Struct(struct_definition) => Ok(8 * get_effective_size(
                            &struct_definition.name,
                            &interface_def.get_effective_struct_attrs(struct_definition)?,
                        )?),
                        Definition::Enum(enum_definition) => enum_definition.get_effective_bit_size(),
                    }
                } else {
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
                }
            } else {
                Err(syn::Error::new_spanned(
                    field_type,
                    "Cannot infer bit size for complex type. Please specify an explicit bit pattern.",
                ))
            }
        }
        Type::Array(array_type) => {
            let element_size = infer_field_size_bits(interface_def, &array_type.elem)?;
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
                format!("{}..={}", r.start, r.end - 1)
            }
        })
        .collect();

    format!("Bits: `[{}]`", range_strs.join(", "))
}

pub fn extract_bits(xs: &[NormalizedRange], ys: &[NormalizedRange]) -> Vec<NormalizedRange> {
    // Expand xs into individual bit indices with their logical positions
    let mut x_bits_with_positions = Vec::new();
    let mut logical_pos = 0;

    for range in xs {
        for bit in range.start..range.end {
            x_bits_with_positions.push((logical_pos, bit));
            logical_pos += 1;
        }
    }

    // Expand ys into the logical positions we want to extract
    let mut y_positions = Vec::new();
    for range in ys {
        for pos in range.start..range.end {
            y_positions.push(pos);
        }
    }

    // Extract the actual bit indices that correspond to the requested logical positions
    let mut result_bits = Vec::new();
    for &wanted_pos in &y_positions {
        if let Some(&(_, actual_bit)) = x_bits_with_positions
            .iter()
            .find(|(logical_pos, _)| *logical_pos == wanted_pos)
        {
            result_bits.push(actual_bit);
        }
    }

    // Convert back to ranges
    let mut result = Vec::new();
    if result_bits.is_empty() {
        return result;
    }

    let mut start = result_bits[0];
    let mut end = start + 1;

    for &bit in result_bits.iter().skip(1) {
        if bit == end {
            // Extend current range
            end += 1;
        } else {
            // Start new range
            result.push(NormalizedRange { start, end });
            start = bit;
            end = bit + 1;
        }
    }

    // Don't forget the last range
    result.push(NormalizedRange { start, end });
    // Renormalize
    normalize_ranges(result)
}

#[cfg(test)]
mod extract_tests {
    use super::*;

    #[test]
    fn test_extract_from_s1_example() {
        // S1 layout: [(20,24),(16,20),(12,16),(8,12),(4,8),(0,4)] - 24 bits total
        // This represents logical positions 0-23 mapped to actual bit positions
        let xs = vec![
            NormalizedRange::new(20, 24).unwrap(), // logical 0-3 -> actual 20-23
            NormalizedRange::new(16, 20).unwrap(), // logical 4-7 -> actual 16-19
            NormalizedRange::new(12, 16).unwrap(), // logical 8-11 -> actual 12-15
            NormalizedRange::new(8, 12).unwrap(),  // logical 12-15 -> actual 8-11
            NormalizedRange::new(4, 8).unwrap(),   // logical 16-19 -> actual 4-7
            NormalizedRange::new(0, 4).unwrap(),   // logical 20-23 -> actual 0-3
        ];

        // S1::b occupies logical positions [(16,24),(8,16)] = [16-23, 8-15]
        let ys = vec![
            NormalizedRange::new(16, 24).unwrap(), // logical 16-23
            NormalizedRange::new(8, 16).unwrap(),  // logical 8-15
        ];

        let result = extract_bits(&xs, &ys);

        // logical 16-23 maps to actual 4-7, 0-3
        // logical 8-15 maps to actual 12-15, 8-11
        // So we should get actual bits: 4-7, 0-3, 12-15, 8-11
        let expected = normalize_ranges(vec![
            NormalizedRange::new(4, 8).unwrap(),
            NormalizedRange::new(0, 4).unwrap(),
            NormalizedRange::new(12, 16).unwrap(),
            NormalizedRange::new(8, 12).unwrap(),
        ]);

        assert_eq!(result, expected);
    }

    #[test]
    fn test_extract_simple() {
        // Simple case: extract middle bits
        let xs = vec![
            NormalizedRange::new(10, 15).unwrap(), // logical 0-4 -> actual 10-14
            NormalizedRange::new(20, 25).unwrap(), // logical 5-9 -> actual 20-24
        ];

        let ys = vec![
            NormalizedRange::new(2, 7).unwrap(), // logical 2-6
        ];

        let result = extract_bits(&xs, &ys);

        // logical 2-4 maps to actual 12-14
        // logical 5-6 maps to actual 20-21
        let expected = vec![
            NormalizedRange::new(12, 15).unwrap(),
            NormalizedRange::new(20, 22).unwrap(),
        ];

        assert_eq!(result, expected);
    }

    #[test]
    fn test_extract_single_bits() {
        let xs = vec![
            NormalizedRange::new(0, 4).unwrap(), // logical 0-3 -> actual 0-3
        ];

        let ys = vec![
            NormalizedRange::new(1, 2).unwrap(), // logical 1
            NormalizedRange::new(3, 4).unwrap(), // logical 3
        ];

        let result = extract_bits(&xs, &ys);

        // logical 1 -> actual 1, logical 3 -> actual 3
        let expected = vec![NormalizedRange::new(1, 2).unwrap(), NormalizedRange::new(3, 4).unwrap()];

        assert_eq!(result, expected);
    }

    #[test]
    fn test_extract_empty() {
        let xs = vec![NormalizedRange::new(0, 4).unwrap()];
        let ys: Vec<NormalizedRange> = vec![];

        let result = extract_bits(&xs, &ys);
        assert_eq!(result, vec![]);
    }
}
