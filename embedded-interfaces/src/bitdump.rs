#![cfg(feature = "std")]

use core::ops::Range;
use std::{collections::HashSet, fmt};

#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub ranges: Vec<Range<usize>>, // bit ranges
    pub name: String,
    pub rust_type: String,
    pub value: String,
    pub reserved: bool,
}

const COLORS: &[&str] = &["\x1b[31m", "\x1b[32m", "\x1b[34m", "\x1b[35m"];
const DIM: &str = "\x1b[2m";
const RESET: &str = "\x1b[0m";
const YELLOW: &str = "\x1b[33m";
const GRAY: &str = "\x1b[90m";
const INVERT: &str = "\x1b[7m";
const CELL: usize = 5; // visible width of one bit cell in the border calculations

#[derive(Debug, Clone)]
struct Section {
    bit_range: Range<usize>,                 // contiguous, unsplit
    field_index: usize,                      // the corresponding field
    non_reserved_field_index: Option<usize>, // numbering for non-reserved fields
    color_index: usize,                      // color index
    label: Option<String>,                   // label to show at the *start* of this section
}

pub struct BitdumpFormatter {
    name: String,
    data: Vec<u8>,
    fields: Vec<Field>,
    sections: Vec<Section>, // contiguous, UNSPLIT across lines
}

impl BitdumpFormatter {
    pub fn new(name: String, data: Vec<u8>, fields: Vec<Field>) -> Self {
        let mut sections = Vec::<Section>::new();
        let mut running_index = 0usize;

        // Gather all field ranges and sort by start bit, also produce re-indexed field internal
        // bit position
        let mut all_ranges: Vec<(usize, &Range<usize>, Range<usize>)> = fields
            .iter()
            .enumerate()
            .flat_map(|(i, f)| {
                let mut offset = 0;
                f.ranges
                    .iter()
                    .map(move |r| {
                        let len = r.end - r.start;
                        let local = offset..offset + len;
                        offset += len;
                        (i, r, local)
                    })
                    .collect::<Vec<_>>() // we need to collect here so offset is updated properly
            })
            .collect();

        all_ranges.sort_by_key(|(_, r, _)| r.start);

        for (field_index, range, internal_range) in all_ranges {
            let field = &fields[field_index];
            let non_reserved_field_index = if field.reserved {
                None
            } else if let Some(prev) = sections.iter().find(|s| s.field_index == field_index) {
                prev.non_reserved_field_index
            } else {
                running_index += 1;
                Some(running_index)
            };

            let color_index = if let Some(nrfi) = non_reserved_field_index {
                nrfi % COLORS.len()
            } else {
                0
            };

            let label = if field.reserved {
                Some("rsv".to_string())
            } else if field.ranges.len() > 1 {
                Some(format!(
                    "{}[{}..{}]",
                    non_reserved_field_index.unwrap_or(0),
                    internal_range.start,
                    internal_range.end
                ))
            } else {
                Some(format!("{}", non_reserved_field_index.unwrap_or(0)))
            };

            sections.push(Section {
                bit_range: range.clone(),
                field_index,
                non_reserved_field_index,
                color_index,
                label,
            });
        }

        Self {
            name,
            data,
            fields,
            sections,
        }
    }

    fn total_bits(&self) -> usize {
        self.data.len() * 8
    }

    fn get_bit(&self, bit_index: usize) -> bool {
        if bit_index >= self.total_bits() {
            return false;
        }
        let byte_index = bit_index / 8;
        let bit_offset = bit_index % 8;
        (self.data[byte_index] >> (7 - bit_offset)) & 1 == 1
    }

    /// Return the *global* section index covering a bit, if any.
    fn section_index_at(&self, bit: usize) -> Option<usize> {
        self.sections
            .iter()
            .position(|s| s.bit_range.start <= bit && bit < s.bit_range.end)
    }

    /// Sections restricted to a given line (8-bit slice). Splits only at line edges.
    fn line_sections(&self, line: usize) -> Vec<Section> {
        let start_bit = line * 8;
        let end_bit = ((line + 1) * 8).min(self.total_bits());
        let mut result = Vec::new();
        for sec in &self.sections {
            if sec.bit_range.end <= start_bit || sec.bit_range.start >= end_bit {
                continue;
            }
            let sub_start = sec.bit_range.start.max(start_bit);
            let sub_end = sec.bit_range.end.min(end_bit);
            let label = if sub_start == sec.bit_range.start {
                sec.label.clone()
            } else {
                None
            };
            result.push(Section {
                bit_range: sub_start..sub_end,
                field_index: sec.field_index,
                non_reserved_field_index: sec.non_reserved_field_index,
                color_index: sec.color_index,
                label,
            });
        }
        result
    }

    /// Build the list of bit positions where the border should have a junction character.
    /// This is the union of current-line section boundaries and (for middle rows) the above-line boundaries.
    fn border_breaks(&self, line: usize) -> Vec<usize> {
        let start_bit = line * 8;
        let end_bit = ((line + 1) * 8).min(self.total_bits());
        let mut breaks = Vec::<usize>::new();

        // Current line boundaries
        for b in start_bit + 1..end_bit {
            // boundary between b-1 and b
            let left = self.section_index_at(b - 1);
            let right = self.section_index_at(b);
            if left != right {
                breaks.push(b);
            }
        }

        // Above-line boundaries (to show ┴ where needed)
        if line > 0 {
            for b in start_bit + 1..end_bit {
                let up_left = if b >= 9 { self.section_index_at(b - 1 - 8) } else { None };
                let up_right = if b >= 8 { self.section_index_at(b - 8) } else { None };
                if up_left != up_right {
                    breaks.push(b);
                }
            }
        }

        breaks.sort_unstable();
        breaks.dedup();
        breaks
    }

    fn junction_char(&self, line: usize, boundary_bit: usize) -> char {
        let left = self.section_index_at(boundary_bit - 1);
        let right = self.section_index_at(boundary_bit);
        let curr_boundary = left != right;
        let above_boundary = if line == 0 {
            false
        } else {
            let up_left = if boundary_bit >= 9 {
                self.section_index_at(boundary_bit - 1 - 8)
            } else {
                None
            };
            let up_right = if boundary_bit >= 8 {
                self.section_index_at(boundary_bit - 8)
            } else {
                None
            };
            up_left != up_right
        };
        match (curr_boundary, above_boundary) {
            (true, true) => '┼',
            (true, false) => '┬',
            (false, true) => '┴',
            (false, false) => '─',
        }
    }

    fn generate_line(
        &self,
        f: &mut fmt::Formatter<'_>,
        line: usize,
        last_line: bool,
        displayed_fields: &mut HashSet<usize>,
    ) -> fmt::Result {
        let start_bit = line * 8;
        let end_bit = ((line + 1) * 8).min(self.total_bits());
        let line_sections = self.line_sections(line);

        // Build border in spans between breakpoints so we can place ┬/┴/┼ exactly at bit boundaries.
        let breaks = self.border_breaks(line);
        let mut span_starts = Vec::new();
        span_starts.push(start_bit);
        for b in &breaks {
            span_starts.push(*b);
        }

        // Border prefix
        if line == 0 {
            write!(f, "   ┌")?;
        } else {
            write!(f, "   ├")?;
        }

        for (si, span_start) in span_starts.iter().enumerate() {
            let span_end = if si + 1 < span_starts.len() {
                span_starts[si + 1]
            } else {
                end_bit
            };
            if span_end <= *span_start {
                continue;
            }
            let bits_in_span = span_end - *span_start;
            let total_width = bits_in_span * CELL - 1;

            // Which line section covers the span start?
            let sec_opt = line_sections.iter().find(|s| s.bit_range.contains(span_start));
            if let Some(sec) = sec_opt {
                if let Some(label) = &sec.label {
                    // Only print label at the *first* span of that section in this line
                    if *span_start == sec.bit_range.start {
                        let effective = if label.len() <= total_width {
                            label.len()
                        } else {
                            total_width
                        };
                        write!(f, "{DIM}{}{RESET}", &label[..effective])?;
                        if total_width > effective {
                            write!(f, "{}", "─".repeat(total_width - effective))?;
                        }
                    } else {
                        write!(f, "{}", "─".repeat(total_width))?;
                    }
                } else {
                    write!(f, "{}", "─".repeat(total_width))?;
                }
            } else {
                write!(f, "{}", "─".repeat(total_width))?;
            }

            // Junction character at the end of span (except at the very end of the line)
            if span_end < end_bit {
                let j = self.junction_char(line, span_end);
                write!(f, "{j}")?;
            }
        }

        if line == 0 {
            writeln!(f, "┐")?;
        } else {
            writeln!(f, "┤")?;
        }

        // Data row with inner separators and ellipsis borders
        let continues_left = if start_bit == 0 {
            false
        } else {
            self.section_index_at(start_bit - 1).is_some()
                && self.section_index_at(start_bit - 1) == self.section_index_at(start_bit)
        };
        let left_border = if continues_left { '…' } else { '│' };
        write!(f, "{line:02x} {left_border}")?;

        let mut display_field = None;
        let mut potentially_displayable_fields = 0;
        for bit in start_bit..end_bit {
            let bit_value = if self.get_bit(bit) { "1" } else { "0" };
            let sec_idx = self.section_index_at(bit);
            if let Some(si) = sec_idx {
                let sec = &self.sections[si];
                let field = &self.fields[sec.field_index];
                if field.reserved {
                    write!(f, "{GRAY}  {bit_value} {RESET}")?; // 2 spaces before, 1 after → visible 4
                } else {
                    let color = COLORS[sec.color_index];
                    write!(f, "{color}  {bit_value} {RESET}")?;

                    if !displayed_fields.contains(&si) {
                        if display_field.is_some_and(|x| x != sec.field_index) {
                            potentially_displayable_fields += 1;
                        }
                        if display_field.is_none() {
                            // We are the first section in this line. If this is also a section
                            // that has not been displayed yet (see condition before this block),
                            // then we want to try to display this section at the end of this line.
                            display_field = Some(sec.field_index);
                        }
                    }
                }
            } else {
                write!(f, "  {bit_value} ")?;
            }

            if bit < end_bit - 1 {
                let left = self.section_index_at(bit);
                let right = self.section_index_at(bit + 1);
                if left != right {
                    write!(f, "│")?;
                } else {
                    write!(f, " ")?;
                }
            }
        }

        // If there are too many contenders for diplayable fields,
        // don't display any of them and instead let them be shown in the legend.
        // This prevents displaying one of three or more smaller fields, which
        // is more readable when all of them are shown in the legend.
        if potentially_displayable_fields > 2 {
            display_field = None;
        }

        let continues_right = if end_bit >= self.total_bits() {
            false
        } else {
            self.section_index_at(end_bit - 1).is_some()
                && self.section_index_at(end_bit - 1) == self.section_index_at(end_bit)
        };
        let right_border = if continues_right { '…' } else { '│' };
        if line < self.data.len() {
            write!(f, "{right_border}  {YELLOW}{:02x}{RESET}", self.data[line])?;

            // This line had a section that we can display
            if let Some(fi) = display_field {
                displayed_fields.insert(fi);
                let field = &self.fields[fi];
                let sec = self
                    .sections
                    .iter()
                    .find(|s| s.field_index == fi)
                    .expect("A matching section must exist");
                let color = COLORS[sec.color_index];
                write!(
                    f,
                    "   {color}{name}{RESET} {ty} = {YELLOW}{val}{RESET}",
                    name = field.name,
                    ty = field.rust_type,
                    val = field.value
                )?;
            }
        }
        writeln!(f)?;

        // Border suffix
        if last_line {
            write!(f, "   └")?;
            for (si, span_start) in span_starts.iter().enumerate() {
                let span_end = if si + 1 < span_starts.len() {
                    span_starts[si + 1]
                } else {
                    end_bit
                };
                if span_end <= *span_start {
                    continue;
                }
                let bits_in_span = span_end - *span_start;
                let total_width = bits_in_span * CELL - 1;

                write!(f, "{}", "─".repeat(total_width))?;

                // Junction character at the end of span (except at the very end of the line)
                if span_end < end_bit {
                    let j = match self.junction_char(line, span_end) {
                        '┼' | '┬' => '┴',
                        _ => '─',
                    };
                    write!(f, "{j}")?;
                }
            }
            writeln!(f, "┘")?;
        }

        Ok(())
    }

    fn generate_field_legend(&self, f: &mut fmt::Formatter<'_>, displayed_fields: &HashSet<usize>) -> fmt::Result {
        let non_reserved: Vec<(&Section, &Field)> = self
            .fields
            .iter()
            .enumerate()
            .map(|(i, f)| {
                (
                    self.sections
                        .iter()
                        .find(|s| s.field_index == i)
                        .expect("A matching section must exist"),
                    f,
                )
            })
            .filter(|(_, f)| !f.reserved)
            .collect();
        if non_reserved.is_empty() {
            return Ok(());
        }

        let max_name_width = non_reserved.iter().map(|(_, f)| f.name.len()).max().unwrap_or(0);
        let max_type_width = non_reserved.iter().map(|(_, f)| f.rust_type.len()).max().unwrap_or(0);

        for (i, (section, field)) in non_reserved.iter().enumerate() {
            if displayed_fields.contains(&section.field_index) {
                continue;
            }

            let color = COLORS[section.color_index];
            let border = if i + 1 == non_reserved.len() { '╰' } else { '├' };
            writeln!(
                f,
                "   {DIM}{border}─ {idx}{RESET} {color}{name:<name_width$}{RESET}  {ty:<type_width$} = {YELLOW}{val}{RESET}",
                idx = i + 1,
                name = field.name,
                name_width = max_name_width,
                ty = field.rust_type,
                type_width = max_type_width,
                val = field.value
            )?;
        }
        Ok(())
    }
}

impl fmt::Display for BitdumpFormatter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "   {DIM}╭─{RESET} {INVERT}{YELLOW}{}{RESET}", self.name)?;
        writeln!(
            f,
            "   {DIM}🭭{RESET}  0    {DIM}1{RESET}    2    {DIM}3{RESET}    4    {DIM}5{RESET}    6    {DIM}7{RESET}"
        )?;

        let total_bits = self.total_bits();
        let total_lines = total_bits.div_ceil(8);
        let mut displayed_fields = HashSet::new();
        for line in 0..total_lines {
            self.generate_line(f, line, line == total_lines - 1, &mut displayed_fields)?;
        }

        self.generate_field_legend(f, &displayed_fields)
    }
}
