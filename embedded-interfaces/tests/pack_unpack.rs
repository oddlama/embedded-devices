use embedded_interfaces::codegen::registers;

type DummyI2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
type DummySpiCodec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec<()>;

#[test]
fn test_empty() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Struct(addr = 0x0, mode = rw, size = 0) {}
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0u8; 0]);
}

#[test]
fn test_bool() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Struct(addr = 0x0, mode = rw, size = 1) {
            f1: bool = false,
            f2: bool = true,
            f3: bool[4],
            f4: bool[3] = true,
            f5: bool[2] = true,
            _ : bool,
            _ : bool,
            f8: bool = true,
        }
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0b01110001u8]);

    let unpacked = Struct([0b01010101u8]).unpack();
    assert_eq!(
        unpacked,
        StructUnpacked {
            f1: false,
            f2: true,
            f3: false,
            f4: true,
            f5: false,
            f8: true
        }
    )
}

#[test]
fn test_unsigned() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Struct(addr = 0x0, mode = rw, size = 2) {
            f1: u8,
            f2: u8 = 0x49,
        }

        Narrow(addr = 0x0, mode = rw, size = 1) {
            f1: u8{3} = 0b010,
            f2: u8{5} = 0b11011,
        }
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0u8, 0x49]);
    assert_eq!(packed.unpack(), StructUnpacked::default());

    let packed = NarrowUnpacked::default().pack();
    assert_eq!(packed.0, [0b01011011u8]);
    assert_eq!(packed.unpack(), NarrowUnpacked::default());
}

#[test]
fn test_unsigned_shifted() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Struct(addr = 0x0, mode = rw, size = 2) {
            f1: u8,
            f2: u8{7} = 0x49,
            _: bool,
        }
    }

    let packed = StructUnpacked::default().pack();
    // bit 0 reserved (0), bits 1-8 for f1 (0), bits 9-15 for f2 (0x49)
    // 0x49 = 0b1001001, shifted left by 1 bit = 0b10010010 = 0x92
    assert_eq!(packed.0, [0x00, 0x92]);
    assert_eq!(packed.unpack(), StructUnpacked::default());
}

#[test]
fn test_unsigned_various_sizes() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Small(addr = 0x0, mode = rw, size = 1) {
            f1: u8{4} = 0b1010,
            f2: u8{4} = 0b0101,
        }

        Medium(addr = 0x0, mode = rw, size = 2) {
            f1: u16 = 0x1234,
        }

        Large(addr = 0x0, mode = rw, size = 4) {
            f1: u32 = 0x12345678,
        }

        Mixed(addr = 0x0, mode = rw, size = 3) {
            f1: u8{4} = 0b1100,
            f2: u16 = 0xABCD,
            f3: u8{4} = 0b0011,
        }
    }

    let packed = SmallUnpacked::default().pack();
    assert_eq!(packed.0, [0b10100101u8]);
    assert_eq!(packed.unpack(), SmallUnpacked::default());

    let packed = MediumUnpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34]);
    assert_eq!(packed.unpack(), MediumUnpacked::default());

    let packed = LargeUnpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34, 0x56, 0x78]);
    assert_eq!(packed.unpack(), LargeUnpacked::default());

    let packed = MixedUnpacked::default().pack();
    // f1 (4 bits): 0b1100 = 0xC0 (top 4 bits)
    // f2 (16 bits): 0xABCD
    // f3 (4 bits): 0b0011 = 0x30 (top 4 bits of last byte)
    assert_eq!(packed.0, [0xCA, 0xBC, 0xD3]);
    assert_eq!(packed.unpack(), MixedUnpacked::default());
}

#[test]
fn test_unsigned_cross_byte_boundary() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        CrossBoundary(addr = 0x0, mode = rw, size = 2) {
            f1: u8{4} = 0b1010,
            f2: u8{8} = 0xFF,
            f3: u8{4} = 0b0101,
        }
    }

    let packed = CrossBoundaryUnpacked::default().pack();
    // f1: 0b1010 in bits 0-3 = 0xA0
    // f2: 0xFF in bits 4-11 = 0xFF shifted by 4 bits
    // f3: 0b0101 in bits 12-15 = 0x50
    assert_eq!(packed.0, [0xAF, 0xF5]);
    assert_eq!(packed.unpack(), CrossBoundaryUnpacked::default());
}

#[test]
fn test_signed_integers() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Signed8(addr = 0x0, mode = rw, size = 1) {
            f1: i8 = -1,
        }

        Signed16(addr = 0x0, mode = rw, size = 2) {
            f1: i16 = -1,
        }

        Signed32(addr = 0x0, mode = rw, size = 4) {
            f1: i32 = -1,
        }

        SignedNarrow(addr = 0x0, mode = rw, size = 1) {
            f1: i8{4} = -1,  // 0b1111 in 4 bits
            f2: i8{4} = 3,   // 0b0011 in 4 bits
        }

        SignedMixed(addr = 0x0, mode = rw, size = 2) {
            f1: i8{6} = -2,  // 0b111110 in 6 bits
            f2: i16{10} = -1, // 0b1111111111 in 10 bits
        }
    }

    let packed = Signed8Unpacked::default().pack();
    assert_eq!(packed.0, [0xFF]);
    assert_eq!(packed.unpack(), Signed8Unpacked::default());

    let packed = Signed16Unpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF]);
    assert_eq!(packed.unpack(), Signed16Unpacked::default());

    let packed = Signed32Unpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(packed.unpack(), Signed32Unpacked::default());

    let packed = SignedNarrowUnpacked::default().pack();
    assert_eq!(packed.0, [0b11110011u8]);
    assert_eq!(packed.unpack(), SignedNarrowUnpacked::default());

    let packed = SignedMixedUnpacked::default().pack();
    // f1: -2 in 6 bits = 0b111110 (top 6 bits)
    // f2: -1 in 10 bits = 0b1111111111 (remaining 10 bits)
    assert_eq!(packed.0, [0b11111011, 0b11111111]);
    assert_eq!(packed.unpack(), SignedMixedUnpacked::default());
}

#[test]
fn test_signed_edge_cases() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        SignedEdge(addr = 0x0, mode = rw, size = 2) {
            f1: i8{4} = 7,   // Max positive for 4 bits
            f2: i8{4} = -8,  // Max negative for 4 bits
            f3: i8{4} = 0,   // Zero
            f4: i8{4} = -1,  // -1
        }
    }

    let packed = SignedEdgeUnpacked::default().pack();
    // f1: 7 = 0b0111, f2: -8 = 0b1000, f3: 0 = 0b0000, f4: -1 = 0b1111
    assert_eq!(packed.0, [0b01111000, 0b00001111]);
    assert_eq!(packed.unpack(), SignedEdgeUnpacked::default());
}

#[test]
fn test_float_types() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Float32(addr = 0x0, mode = rw, size = 4) {
            f1: f32 = 1.0,
        }

        Float64(addr = 0x0, mode = rw, size = 8) {
            f1: f64 = 1.0,
        }

        MixedFloat(addr = 0x0, mode = rw, size = 5) {
            f1: f32 = 1.0,
            f2: u8 = 0xFF,
        }
    }

    let packed = Float32Unpacked::default().pack();
    assert_eq!(packed.0, 1.0f32.to_be_bytes());
    assert_eq!(packed.unpack(), Float32Unpacked::default());

    let packed = Float64Unpacked::default().pack();
    assert_eq!(packed.0, 1.0f64.to_be_bytes());
    assert_eq!(packed.unpack(), Float64Unpacked::default());

    let packed = MixedFloatUnpacked::default().pack();
    let mut expected = [0u8; 5];
    expected[0..4].copy_from_slice(&1.0f32.to_be_bytes());
    expected[4] = 0xFF;
    assert_eq!(packed.0, expected);
    assert_eq!(packed.unpack(), MixedFloatUnpacked::default());
}

#[test]
fn test_u8_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Array4(addr = 0x0, mode = rw, size = 4) {
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
        }

        ArrayShifted(addr = 0x0, mode = rw, size = 5) {
            prefix: bool = true,
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
            _: u8{7}
        }

        ArrayShifted2(addr = 0x0, mode = rw, size = 5) {
            _: u8{2} = 0b11
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
            _: u8{6}
        }

        ArrayMixed(addr = 0x0, mode = rw, size = 5) {
            prefix: u8{4} = 0xa,
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
            suffix: u8{4} = 0xb,
        }
    }

    let packed = Array4Unpacked::default().pack();
    assert_eq!(packed.0, [0x01, 0x02, 0x03, 0x05]);
    assert_eq!(packed.unpack(), Array4Unpacked::default());

    let packed = ArrayShiftedUnpacked::default().pack();
    // prefix: true = 0b1 in bit 0
    // data: [0x01, 0x02, 0x03, 0x05] starting at bit 1
    assert_eq!(packed.0, [0x80, 0x81, 0x01, 0x82, 0x80]);
    assert_eq!(packed.unpack(), ArrayShiftedUnpacked::default());

    let packed = ArrayShifted2Unpacked::default().pack();
    // prefix: 0b11 in bit 0 and 1
    // data: [0x01, 0x02, 0x03, 0x05] starting at bit 2
    assert_eq!(packed.0, [0xc0, 0x40, 0x80, 0xc1, 0x40]);
    assert_eq!(packed.unpack(), ArrayShifted2Unpacked::default());

    let packed = ArrayMixedUnpacked::default().pack();
    // prefix: 0xA = 0b1010 in bits 0-3
    // data: [0x01, 0x02, 0x03, 0x05] in bits 4-35
    // suffix: 0xB = 0b1011 in bits 36-39
    assert_eq!(packed.0, [0xa0, 0x10, 0x20, 0x30, 0x5b]);
    assert_eq!(packed.unpack(), ArrayMixedUnpacked::default());
}

#[test]
fn test_u16_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Array2(addr = 0x0, mode = rw, size = 4) {
            data: [u16; 2] = [0x1234, 0x5678],
        }

        ArrayShifted(addr = 0x0, mode = rw, size = 5) {
            prefix: bool = true,
            data: [u16; 2] = [0x1234, 0x5678],
            _: u8{7}
        }
    }

    let packed = Array2Unpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34, 0x56, 0x78]);
    assert_eq!(packed.unpack(), Array2Unpacked::default());

    let packed = ArrayShiftedUnpacked::default().pack();
    // prefix: true = 0b1 in bit 0
    // data: [0x1234, 0x5678] starting at bit 1
    assert_eq!(packed.0, [0x89, 0x1a, 0x2b, 0x3c, 0x0]);
    assert_eq!(packed.unpack(), ArrayShiftedUnpacked::default());
}

#[test]
fn test_bool_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        BoolArray8(addr = 0x0, mode = rw, size = 1) {
            flags: [bool; 8] = [true, false, true, false, true, false, true, false],
        }

        BoolArrayShifted(addr = 0x0, mode = rw, size = 2) {
            prefix: u8{4} = 0xF,
            flags: [bool; 8] = [true, false, true, false, true, false, true, false],
            suffix: u8{4} = 0xA,
        }
    }

    let packed = BoolArray8Unpacked::default().pack();
    assert_eq!(packed.0, [0b10101010u8]);
    assert_eq!(packed.unpack(), BoolArray8Unpacked::default());

    let packed = BoolArrayShiftedUnpacked::default().pack();
    // prefix: 0xF = 0b1111 in bits 0-3
    // flags: [T,F,T,F,T,F,T,F] = 0b10101010 in bits 4-11
    // suffix: 0xA = 0b1010 in bits 12-15
    assert_eq!(packed.0, [0xFA, 0xAA]);
    assert_eq!(packed.unpack(), BoolArrayShiftedUnpacked::default());
}

#[test]
fn test_signed_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        SignedArray(addr = 0x0, mode = rw, size = 4) {
            data: [i16; 2] = [-1, 1000],
        }
    }

    let packed = SignedArrayUnpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0x03, 0xE8]);
    assert_eq!(packed.unpack(), SignedArrayUnpacked::default());
}

#[test]
fn test_float_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        FloatArray(addr = 0x0, mode = rw, size = 8) {
            data: [f32; 2] = [1.0, -1.0],
        }
    }

    let packed = FloatArrayUnpacked::default().pack();
    let mut expected = [0u8; 8];
    expected[0..4].copy_from_slice(&1.0f32.to_be_bytes());
    expected[4..8].copy_from_slice(&(-1.0f32).to_be_bytes());
    assert_eq!(packed.0, expected);
    assert_eq!(packed.unpack(), FloatArrayUnpacked::default());
}

#[test]
fn test_complex_mixed_layout() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Complex(addr = 0x0, mode = rw, size = 7) {
            header: u8{4} = 0xA,
            flags: [bool; 4] = [true, false, true, false],
            id: u16 = 0x1234,
            data: [u8; 3] = [0x11, 0x22, 0x33],
            checksum: u8 = 0xFF,
        }
    }

    let packed = ComplexUnpacked::default().pack();
    // header: 0xA = 0b1010 in bits 0-3
    // flags: [T,F,T,F] = 0b1010 in bits 4-7
    // id: 0x1234 in bits 8-23
    // data: [0x11, 0x22, 0x33] in bits 24-47
    // checksum: 0xFF in bits 48-55
    assert_eq!(packed.0, [0xAA, 0x12, 0x34, 0x11, 0x22, 0x33, 0xFF]);
    assert_eq!(packed.unpack(), ComplexUnpacked::default());
}

#[test]
fn test_roundtrip_consistency() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        RoundTrip(addr = 0x0, mode = rw, size = 4) {
            f1: u8{4} = 0b1010,
            f2: i8{4} = -2,
            f3: bool = true,
            f4: u16{10} = 0x3FF,
            f5: [bool; 5] = [true, false, true, false, true],
            f6: u8{8} = 0xAB,
        }
    }

    // Test that pack -> unpack -> pack produces the same result
    let original = RoundTripUnpacked::default();
    let packed1 = original.pack();
    let unpacked = packed1.unpack();
    let packed2 = unpacked.pack();

    assert_eq!(original, unpacked);
    assert_eq!(packed1.0, packed2.0);

    // Test with different values
    let modified = RoundTripUnpacked {
        f1: 0b0101,
        f2: 3,
        f3: false,
        f4: 0x200,
        f5: [false, true, false, true, false],
        f6: 0x12,
    };

    let packed1 = modified.pack();
    let unpacked = packed1.unpack();
    let packed2 = unpacked.pack();

    assert_eq!(modified, unpacked);
    assert_eq!(packed1.0, packed2.0);
}

#[test]
fn test_edge_case_bit_patterns() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        EdgeCase(addr = 0x0, mode = rw, size = 2) {
            f1: u8{1} = 1,      // Single bit
            f2: u8{7} = 0x7F,   // 7 bits
            f3: u8{1} = 0,      // Single bit
            f4: u8{7} = 0x01,   // 7 bits
        }
    }

    let packed = EdgeCaseUnpacked::default().pack();
    // f1: 1 = 0b1 in bit 0
    // f2: 0x7F = 0b1111111 in bits 1-7
    // f3: 0 = 0b0 in bit 8
    // f4: 0x01 = 0b0000001 in bits 9-15
    assert_eq!(packed.0, [0xFF, 0x01]);
    assert_eq!(packed.unpack(), EdgeCaseUnpacked::default());
}

#[test]
fn test_large_values() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        Large(addr = 0x0, mode = rw, size = 16) {
            f1: u64 = 0x123456789ABCDEF0,
            f2: u64 = 0xFEDCBA9876543210,
        }
    }

    let packed = LargeUnpacked::default().pack();
    let mut expected = [0u8; 16];
    expected[0..8].copy_from_slice(&0x123456789ABCDEF0u64.to_be_bytes());
    expected[8..16].copy_from_slice(&0xFEDCBA9876543210u64.to_be_bytes());
    assert_eq!(packed.0, expected);
    assert_eq!(packed.unpack(), LargeUnpacked::default());
}

#[test]
fn test_nested_arrays() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        // Array of arrays of u8
        U8Matrix(addr = 0x0, mode = rw, size = 12) {
            matrix: [[u8; 3]; 4] = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]],
        }

        // Array of arrays of bool
        BoolMatrix(addr = 0x0, mode = rw, size = 4) {
            flags: [[bool; 8]; 4] = [[true; 8]; 4],
        }

        // Array of arrays of signed integers
        I16Matrix(addr = 0x0, mode = rw, size = 24) {
            temps: [[i16; 3]; 4] = [[-10, 0, 10], [-20, 0, 20], [-30, 0, 30], [-40, 0, 40]],
        }

        // Mixed nested arrays
        MixedNested(addr = 0x0, mode = rw, size = 15) {
            header: u8 = 0xAA,
            data: [[u8; 2]; 3] = [[0x11, 0x22], [0x33, 0x44], [0x55, 0x66]],
            checksums: [u16; 4] = [0x1234, 0x5678, 0x9ABC, 0xDEF0],
        }

        // 3D array
        Cube(addr = 0x0, mode = rw, size = 8) {
            cube: [[[u8; 2]; 2]; 2] = [[[1, 2], [3, 4]], [[5, 6], [7, 8]]],
        }
    }

    // Test u8 matrix
    let packed = U8MatrixUnpacked::default().pack();
    assert_eq!(packed.0, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.matrix, [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]);

    // Test bool matrix (4x8 = 32 bits = 4 bytes)
    let packed = BoolMatrixUnpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0xFF, 0xFF]);
    let unpacked = BoolMatrix([0xAA, 0x55, 0xCC, 0x33]).unpack();
    let expected_flags = [
        [true, false, true, false, true, false, true, false],
        [false, true, false, true, false, true, false, true],
        [true, true, false, false, true, true, false, false],
        [false, false, true, true, false, false, true, true],
    ];
    assert_eq!(unpacked.flags, expected_flags);

    // Test i16 matrix (4x3x16 = 192 bits = 24 bytes)
    let packed = I16MatrixUnpacked::default().pack();
    let expected = [
        0xFF, 0xF6, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xEC, 0x00, 0x00, 0x00, 0x14, 0xFF, 0xE2, 0x00, 0x00, 0x00, 0x1E,
        0xFF, 0xD8, 0x00, 0x00, 0x00, 0x28,
    ];
    assert_eq!(packed.0, expected);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.temps, [[-10, 0, 10], [-20, 0, 20], [-30, 0, 30], [-40, 0, 40]]);

    // Test mixed nested
    let packed = MixedNestedUnpacked::default().pack();
    let expected = [
        0xAA, // header
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, // data[3][2]
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, // checksums[4]
    ];
    assert_eq!(packed.0, expected);

    // Test 3D cube (2x2x2x8 = 32 bits = 4 bytes)
    let packed = CubeUnpacked::default().pack();
    assert_eq!(packed.0, [1, 2, 3, 4, 5, 6, 7, 8]);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.cube, [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]);

    // Test custom values
    let custom = U8MatrixUnpacked {
        matrix: [
            [0xAA, 0xBB, 0xCC],
            [0xDD, 0xEE, 0xFF],
            [0x11, 0x22, 0x33],
            [0x44, 0x55, 0x66],
        ],
    };
    let packed = custom.pack();
    let unpacked = packed.unpack();
    assert_eq!(
        unpacked.matrix,
        [
            [0xAA, 0xBB, 0xCC],
            [0xDD, 0xEE, 0xFF],
            [0x11, 0x22, 0x33],
            [0x44, 0x55, 0x66]
        ]
    );
}

#[test]
fn test_deeply_nested() {
    registers! {
        defaults {
            i2c_codec = DummyI2cCodec,
            spi_codec = DummySpiCodec,
            codec_error = (),
        }

        // 4D array - array of 3D cubes
        HyperCube(addr = 0x0, mode = rw, size = 32) {
            hyper: [[[[u8; 2]; 2]; 2]; 4] = [
                [[[1, 2], [3, 4]], [[5, 6], [7, 8]]],
                [[[9, 10], [11, 12]], [[13, 14], [15, 16]]],
                [[[17, 18], [19, 20]], [[21, 22], [23, 24]]],
                [[[25, 26], [27, 28]], [[29, 30], [31, 32]]],
            ],
        }

        // Array of arrays of arrays of bool
        BoolTensor(addr = 0x0, mode = rw, size = 4) {
            tensor: [[[bool; 2]; 2]; 8] = [
                [[true, false], [false, true]],
                [[false, true], [true, false]],
                [[true, true], [false, false]],
                [[false, false], [true, true]],
                [[true, false], [true, false]],
                [[false, true], [false, true]],
                [[true, true], [true, true]],
                [[false, false], [false, false]],
            ],
        }
    }

    // Test 4D hypercube (what am I doing here??)
    let packed = HyperCubeUnpacked::default().pack();
    let expected: [u8; 32] = (1..=32).collect::<Vec<_>>().try_into().unwrap();
    assert_eq!(packed.0, expected);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.hyper[0][0][0][0], 1);
    assert_eq!(unpacked.hyper[3][1][1][1], 32);

    // Test bool tensor (8x2x2x1 = 32 bits = 4 bytes)
    let packed = BoolTensorUnpacked::default().pack();
    let unpacked = packed.unpack();
    assert_eq!(unpacked.tensor[0], [[true, false], [false, true]]);
    assert_eq!(unpacked.tensor[7], [[false, false], [false, false]]);
}
