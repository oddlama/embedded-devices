use embedded_interfaces::codegen::registers;

#[test]
fn test_bit_pattern_consolidation() {
    // This should compile and generate proper bit patterns
    registers! {
        defaults {
            i2c_codec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec::<()>,
            spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
            codec_error = (),
        }

        // Test explicit bit patterns with consolidation
        Consolidated(addr = 0x1, mode = rw, size = 2) {
            // Single bits that should be consolidated: [0, 1, 2] -> [0..3]
            field1: u8[0, 1, 2],
            // Adjacent ranges: [3..5, 5..7] -> [3..7]
            field2: u8[3..5, 5..7],
            // Inclusive ranges: [7..=9] -> [7..10]
            field3: u8[7..=9],
            // Mixed patterns: [10, 11, 12..15] -> [10..15]
            field4: u8[10, 11, 12..15],
            // Single bit
            field5: bool[15],
        }

        // Test automatic bit assignment
        AutoAssigned(addr = 0x2, mode = rw, size = 2) {
            // Should be assigned [0..8]
            field1: u8,
            // Should be assigned [8..9]
            field2: bool,
            // Should be assigned [9..16]
            field3: u8,
        }

        // Test mixed explicit and automatic
        Mixed(addr = 0x3, mode = rw, size = 2) {
            // Explicit assignment [0..4]
            field1: u8[0..4],
            // Auto assignment should start at [4..12] (u8 = 8 bits)
            field2: u8,
            // Explicit assignment [12..16]
            field3: u8[12..16],
        }
    }
}

#[test]
fn test_array_types() {
    registers! {
        defaults {
            i2c_codec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec::<()>,
            spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
            codec_error = (),
        }

        ArrayTest(addr = 0x4, mode = rw, size = 4) {
            // 4 bytes = 32 bits, should auto-assign [0..32]
            data: [u8; 4],
        }
    }
}

// These tests should fail to compile with good error messages
#[cfg(test_errors)]
mod error_tests {
    use register_macro_lib::registers;

    #[test]
    fn test_size_mismatch() {
        // Should fail: only 15 bits defined but size is 2 bytes (16 bits)
        registers! {
            defaults {
                i2c_codec = DummyCodec,
                spi_codec = DummyCodec,
                codec_error = (),
            }

            SizeMismatch(addr = 0x1, mode = rw, size = 2) {
                field1: u8[0..8],
                field2: u8[8..15], // Missing bit 15
            }
        }
    }

    #[test]
    fn test_overlap() {
        // Should fail: overlapping bit ranges
        registers! {
            defaults {
                i2c_codec = DummyCodec,
                spi_codec = DummyCodec,
                codec_error = (),
            }

            Overlap(addr = 0x1, mode = rw, size = 2) {
                field1: u8[0..8],
                field2: u8[4..12], // Overlaps with field1
            }
        }
    }

    #[test]
    fn test_empty_range() {
        // Should fail: empty range
        registers! {
            defaults {
                i2c_codec = DummyCodec,
                spi_codec = DummyCodec,
                codec_error = (),
            }

            EmptyRange(addr = 0x1, mode = rw, size = 2) {
                field1: u8[0..0], // Empty range
            }
        }
    }

    #[test]
    fn test_unknown_type() {
        // Should fail: cannot infer size for custom type
        registers! {
            defaults {
                i2c_codec = DummyCodec,
                spi_codec = DummyCodec,
                codec_error = (),
            }

            UnknownType(addr = 0x1, mode = rw, size = 2) {
                field1: CustomType, // No bit pattern and unknown size
            }
        }
    }

    #[test]
    fn test_gap_in_coverage() {
        // Should fail: gap in bit coverage
        registers! {
            defaults {
                i2c_codec = DummyCodec,
                spi_codec = DummyCodec,
                codec_error = (),
            }

            Gap(addr = 0x1, mode = rw, size = 2) {
                field1: u8[0..8],
                field2: u8[10..16], // Gap at bits 8-9
            }
        }
    }
}
