use embedded_interfaces::codegen::interface_objects;

#[test]
fn test_empty() {
    interface_objects! {
        struct Struct(size = 0) {}
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0u8; 0]);
}

#[test]
fn test_bool() {
    interface_objects! {
        struct Struct(size = 1) {
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
    interface_objects! {
        struct Struct(size = 2) {
            f1: u8,
            f2: u8 = 0x49,
        }

        struct Narrow(size = 1) {
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
    interface_objects! {
        struct Struct(size = 2) {
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
    interface_objects! {
        struct Small(size = 1) {
            f1: u8{4} = 0b1010,
            f2: u8{4} = 0b0101,
        }

        struct Medium(size = 2) {
            f1: u16 = 0x1234,
        }

        struct Large(size = 4) {
            f1: u32 = 0x12345678,
        }

        struct Mixed(size = 3) {
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
    interface_objects! {
        struct CrossBoundary(size = 2) {
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
    interface_objects! {
        struct Signed8(size = 1) {
            f1: i8 = -1,
        }

        struct Signed16(size = 2) {
            f1: i16 = -1,
        }

        struct Signed32(size = 4) {
            f1: i32 = -1,
        }

        struct SignedNarrow(size = 1) {
            f1: i8{4} = -1,  // 0b1111 in 4 bits
            f2: i8{4} = 3,   // 0b0011 in 4 bits
        }

        struct SignedMixed(size = 2) {
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
    interface_objects! {
        struct SignedEdge(size = 2) {
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
    interface_objects! {
        struct Float32(size = 4) {
            f1: f32 = 1.0,
        }

        struct Float64(size = 8) {
            f1: f64 = 1.0,
        }

        struct MixedFloat(size = 5) {
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
    interface_objects! {
        struct Array4(size = 4) {
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
        }

        struct ArrayShifted(size = 5) {
            prefix: bool = true,
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
            _: u8{7}
        }

        struct ArrayShifted2(size = 5) {
            _: u8{2} = 0b11
            data: [u8; 4] = [0x01, 0x02, 0x03, 0x05],
            _: u8{6}
        }

        struct ArrayMixed(size = 5) {
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
    interface_objects! {
        struct Array2(size = 4) {
            data: [u16; 2] = [0x1234, 0x5678],
        }

        struct ArrayShifted(size = 5) {
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
    interface_objects! {
        struct BoolArray8(size = 1) {
            flags: [bool; 8] = [true, false, true, false, true, false, true, false],
        }

        struct BoolArrayShifted(size = 2) {
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
    interface_objects! {
        struct SignedArray(size = 4) {
            data: [i16; 2] = [-1, 1000],
        }
    }

    let packed = SignedArrayUnpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0x03, 0xE8]);
    assert_eq!(packed.unpack(), SignedArrayUnpacked::default());
}

#[test]
fn test_float_arrays() {
    interface_objects! {
        struct FloatArray(size = 8) {
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
    interface_objects! {
        struct Complex(size = 7) {
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
    interface_objects! {
        struct RoundTrip(size = 4) {
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
    interface_objects! {
        struct EdgeCase(size = 2) {
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
    interface_objects! {
        struct Large(size = 16) {
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
    interface_objects! {
        // Array of arrays of u8
        struct U8Matrix(size = 12) {
            matrix: [[u8; 3]; 4] = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]],
        }

        // Array of arrays of bool
        struct BoolMatrix(size = 4) {
            flags: [[bool; 8]; 4] = [[true; 8]; 4],
        }

        // Array of arrays of signed integers
        struct I16Matrix(size = 24) {
            temps: [[i16; 3]; 4] = [[-10, 0, 10], [-20, 0, 20], [-30, 0, 30], [-40, 0, 40]],
        }

        // Mixed nested arrays
        struct MixedNested(size = 15) {
            header: u8 = 0xAA,
            data: [[u8; 2]; 3] = [[0x11, 0x22], [0x33, 0x44], [0x55, 0x66]],
            checksums: [u16; 4] = [0x1234, 0x5678, 0x9ABC, 0xDEF0],
        }

        // 3D array
        struct Cube(size = 8) {
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
    interface_objects! {
        // 4D array - array of 3D cubes
        struct HyperCube(size = 32) {
            hyper: [[[[u8; 2]; 2]; 2]; 4] = [
                [[[1, 2], [3, 4]], [[5, 6], [7, 8]]],
                [[[9, 10], [11, 12]], [[13, 14], [15, 16]]],
                [[[17, 18], [19, 20]], [[21, 22], [23, 24]]],
                [[[25, 26], [27, 28]], [[29, 30], [31, 32]]],
            ],
        }

        // Array of arrays of arrays of bool
        struct BoolTensor(size = 4) {
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

#[test]
fn test_enum_basic() {
    interface_objects! {
        enum SimpleEnum: u8 {
            0 First,
            1 Second,
            2 Third,
            _ Invalid,
        }

        struct WithEnum(size = 1) {
            mode: SimpleEnum = SimpleEnum::First,
        }
    }

    let packed = WithEnumUnpacked::default().pack();
    assert_eq!(packed.0, [0]);
    assert_eq!(packed.unpack(), WithEnumUnpacked::default());

    let custom = WithEnumUnpacked {
        mode: SimpleEnum::Second,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [1]);
    assert_eq!(packed.unpack(), custom);

    let custom = WithEnumUnpacked {
        mode: SimpleEnum::Third,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [2]);
    assert_eq!(packed.unpack(), custom);
}

#[test]
fn test_enum_with_bit_width() {
    interface_objects! {
        enum Mode: u8{3} {
            0b000 PowerDown,
            0b001 Sleep,
            0b010 Active,
            0b011 HighPower,
            0b100 Debug,
            0b101 Test,
            0b110 Reserved1,
            0b111 Reserved2,
        }

        struct Device(size = 1) {
            mode: Mode = Mode::HighPower,
            flags: u8{5} = 0b10101,
        }
    }

    let packed = DeviceUnpacked::default().pack();
    // mode: 0b011 in bits 0-2, flags: 0b10101 in bits 3-7
    assert_eq!(packed.0, [0b01110101]);
    assert_eq!(packed.unpack(), DeviceUnpacked::default());

    let custom = DeviceUnpacked {
        mode: Mode::Sleep,
        flags: 0b11111,
    };
    let packed = custom.pack();
    // mode: 0b011 in bits 0-2, flags: 0b11111 in bits 3-7
    assert_eq!(packed.0, [0b00111111]);
    assert_eq!(packed.unpack(), custom);
}

#[test]
fn test_enum_with_ranges() {
    interface_objects! {
        enum Status: u8{4} {
            0 Off,
            1..=3 Low(u8),
            4..=7 Medium(u8),
            8..12 High(u8),
            13 Critical,
            12|14|15 Error(u8),
        }

        struct Sensor(size = 1) {
            _: u8{4},
            status: Status = Status::Off,
        }
    }

    let packed = SensorUnpacked::default().pack();
    assert_eq!(packed.0, [0]);
    assert_eq!(packed.unpack(), SensorUnpacked::default());

    // Test range variants
    let custom = SensorUnpacked { status: Status::Low(2) };
    let packed = custom.pack();
    assert_eq!(packed.0, [2]);
    let unpacked = packed.unpack();
    if let Status::Low(val) = unpacked.status {
        assert_eq!(val, 2);
    } else {
        panic!("Expected Low variant");
    }

    let custom = SensorUnpacked {
        status: Status::High(10),
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [10]);
    let unpacked = packed.unpack();
    if let Status::High(val) = unpacked.status {
        assert_eq!(val, 10);
    } else {
        panic!("Expected High variant");
    }

    let custom = SensorUnpacked {
        status: Status::Error(15),
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [15]);
    let unpacked = packed.unpack();
    if let Status::Error(val) = unpacked.status {
        assert_eq!(val, 15);
    } else {
        panic!("Expected Error variant");
    }
}

#[test]
fn test_enum_with_wildcard() {
    interface_objects! {
        enum Protocol: u8{3} {
            0 None,
            1 TCP,
            2 UDP,
            _ Unknown(u8),
        }

        struct Packet(size = 1) {
            protocol: Protocol = Protocol::None,
            _: u8{5},
        }
    }

    let packed = PacketUnpacked::default().pack();
    assert_eq!(packed.0, [0]);
    assert_eq!(packed.unpack(), PacketUnpacked::default());

    let custom = PacketUnpacked {
        protocol: Protocol::TCP,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [1 << 5]);
    assert_eq!(packed.unpack(), custom);

    // Test wildcard matching
    let packet = Packet([5 << 5]); // 5 should match wildcard
    let unpacked = packet.unpack();
    if let Protocol::Unknown(val) = unpacked.protocol {
        assert_eq!(val, 5);
    } else {
        panic!("Expected Unknown variant");
    }

    let packet = Packet([7 << 5]); // 7 should also match wildcard
    let unpacked = packet.unpack();
    if let Protocol::Unknown(val) = unpacked.protocol {
        assert_eq!(val, 7);
    } else {
        panic!("Expected Unknown variant");
    }
}

#[test]
fn test_enum_complex_patterns() {
    interface_objects! {
        enum Command: u8{4} {
            0 Nop,
            1|3|5 Odd(u8),
            2|4|6 Even(u8),
            7..=10 Range(u8),
            11 Special,
            _ Invalid(u8),
        }

        struct Message(size = 1) {
            cmd: Command = Command::Nop,
            _: u8{4},
        }
    }

    let packed = MessageUnpacked::default().pack();
    assert_eq!(packed.0, [0]);
    assert_eq!(packed.unpack(), MessageUnpacked::default());

    // Test or patterns
    let custom = MessageUnpacked { cmd: Command::Odd(3) };
    let packed = custom.pack();
    assert_eq!(packed.0, [3 << 4]);
    let unpacked = packed.unpack();
    if let Command::Odd(val) = unpacked.cmd {
        assert_eq!(val, 3);
    } else {
        panic!("Expected Odd variant");
    }

    let custom = MessageUnpacked { cmd: Command::Even(4) };
    let packed = custom.pack();
    assert_eq!(packed.0, [4 << 4]);
    let unpacked = packed.unpack();
    if let Command::Even(val) = unpacked.cmd {
        assert_eq!(val, 4);
    } else {
        panic!("Expected Even variant");
    }

    // Test range
    let custom = MessageUnpacked { cmd: Command::Range(9) };
    let packed = custom.pack();
    assert_eq!(packed.0, [9 << 4]);
    let unpacked = packed.unpack();
    if let Command::Range(val) = unpacked.cmd {
        assert_eq!(val, 9);
    } else {
        panic!("Expected Range variant");
    }

    // Test wildcard
    let message = Message([12 << 4]); // 12 should match wildcard
    let unpacked = message.unpack();
    if let Command::Invalid(val) = unpacked.cmd {
        assert_eq!(val, 12);
    } else {
        panic!("Expected Invalid variant");
    }
}

#[test]
fn test_enum_without_bit_width() {
    interface_objects! {
        enum FullRange: u8 {
            0..=127 Valid(u8),
            _ Invalid(u8),
        }

        struct Data(size = 1) {
            value: FullRange = FullRange::Valid(42),
        }
    }

    let packed = DataUnpacked::default().pack();
    assert_eq!(packed.0, [42]);
    let unpacked = packed.unpack();
    if let FullRange::Valid(val) = unpacked.value {
        assert_eq!(val, 42);
    } else {
        panic!("Expected Valid variant");
    }

    // Test invalid range
    let data = Data([200]);
    let unpacked = data.unpack();
    if let FullRange::Invalid(val) = unpacked.value {
        assert_eq!(val, 200);
    } else {
        panic!("Expected Invalid variant");
    }
}

#[test]
fn test_enum_different_underlying_types() {
    interface_objects! {
        enum Small: u8{2} {
            0 A,
            1 B,
            2 C,
            3 D,
        }

        enum Medium: u16{10} {
            0..=511 Low(u16),
            512..=1023 High(u16),
        }

        enum Large: u32{20} {
            0 Zero,
            1..=1048575 NonZero(u32),
        }

        struct MultiEnum(size = 4) {
            small: Small = Small::A,
            medium: Medium = Medium::Low(100),
            large: Large = Large::Zero,
        }
    }

    let packed = MultiEnumUnpacked::default().pack();
    // small: 0 in bits 0-1
    // medium: 100 in bits 2-11
    // large: 0 in bits 12-31
    assert_eq!(packed.0, [0b00000110, 0b01000000, 0, 0]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.small, Small::A);
    if let Medium::Low(val) = unpacked.medium {
        assert_eq!(val, 100);
    } else {
        panic!("Expected Low variant");
    }
    assert_eq!(unpacked.large, Large::Zero);

    // Test with different values
    let custom = MultiEnumUnpacked {
        small: Small::D,
        medium: Medium::High(700),
        large: Large::NonZero(12345),
    };
    let packed = custom.pack();
    let unpacked = packed.unpack();
    assert_eq!(unpacked.small, Small::D);
    if let Medium::High(val) = unpacked.medium {
        assert_eq!(val, 700);
    } else {
        panic!("Expected High variant");
    }
    if let Large::NonZero(val) = unpacked.large {
        assert_eq!(val, 12345);
    } else {
        panic!("Expected NonZero variant");
    }
}

#[test]
fn test_enum_mixed_with_other_types() {
    interface_objects! {
        enum Priority: u8{3} {
            0 Low,
            1..=3 Medium(u8),
            4..=6 High(u8),
            7 Critical,
        }

        struct Task(size = 3) {
            id: u16 = 0x1234,
            priority: Priority = Priority::Low,
            flags: [bool; 5] = [true, false, true, false, true],
        }
    }

    let packed = TaskUnpacked::default().pack();
    // id: 0x1234 in bits 0-15
    // priority: 0 in bits 16-18
    // flags: 0b10101 in bits 19-23
    assert_eq!(packed.0, [0x12, 0x34, 0b00010101]);
    assert_eq!(packed.unpack(), TaskUnpacked::default());

    let custom = TaskUnpacked {
        id: 0x5678,
        priority: Priority::High(5),
        flags: [false, true, false, true, false],
    };
    let packed = custom.pack();
    let unpacked = packed.unpack();
    assert_eq!(unpacked.id, 0x5678);
    if let Priority::High(val) = unpacked.priority {
        assert_eq!(val, 5);
    } else {
        panic!("Expected High variant");
    }
    assert_eq!(unpacked.flags, [false, true, false, true, false]);
}

#[test]
fn test_enum_roundtrip_with_captured_values() {
    interface_objects! {
        enum Variable: u8{6} {
            0 None,
            1..=31 Small(u8),
            32..=63 Large(u8),
        }

        struct Container(size = 1) {
            var: Variable = Variable::None,
            _: u8{2},
        }
    }

    // Test that captured values survive roundtrip
    let values_to_test = [
        Variable::None,
        Variable::Small(5),
        Variable::Small(31),
        Variable::Large(32),
        Variable::Large(50),
        Variable::Large(63),
    ];

    for original_var in values_to_test {
        let container = ContainerUnpacked { var: original_var };
        let packed = container.pack();
        let unpacked = packed.unpack();

        match (original_var, unpacked.var) {
            (Variable::None, Variable::None) => {}
            (Variable::Small(a), Variable::Small(b)) => assert_eq!(a, b),
            (Variable::Large(a), Variable::Large(b)) => assert_eq!(a, b),
            _ => panic!("Enum variant mismatch after roundtrip"),
        }
    }
}

#[test]
fn test_enum_exhaustive_without_wildcard() {
    interface_objects! {
        enum Binary: u8{1} {
            0 False,
            1 True,
        }

        enum Quaternary: u8{2} {
            0 Zero,
            1 One,
            2 Two,
            3 Three,
        }

        struct Exhaustive(size = 1) {
            binary: Binary = Binary::False,
            quaternary: Quaternary = Quaternary::Zero,
            _: u8{5},
        }
    }

    let packed = ExhaustiveUnpacked::default().pack();
    assert_eq!(packed.0, [0]);
    assert_eq!(packed.unpack(), ExhaustiveUnpacked::default());

    // Test all combinations
    let combinations = [
        (Binary::False, Quaternary::Zero),
        (Binary::False, Quaternary::One),
        (Binary::False, Quaternary::Two),
        (Binary::False, Quaternary::Three),
        (Binary::True, Quaternary::Zero),
        (Binary::True, Quaternary::One),
        (Binary::True, Quaternary::Two),
        (Binary::True, Quaternary::Three),
    ];

    for (binary, quaternary) in combinations {
        let custom = ExhaustiveUnpacked { binary, quaternary };
        let packed = custom.pack();
        let unpacked = packed.unpack();
        assert_eq!(unpacked.binary, binary);
        assert_eq!(unpacked.quaternary, quaternary);
    }
}

#[test]
fn test_enum_representative_values() {
    interface_objects! {
        enum MultiMatch: u8{3} {
            0|2|4 Even, // Should use 0 as representative
            1|3|5 Odd,  // Should use 1 as representative
            6|7 High,   // Should use 6 as representative
        }

        struct Representative(size = 1) {
            _: u8{5},
            value: MultiMatch = MultiMatch::Even,
        }
    }

    // Test that serialization uses representative values
    let even = RepresentativeUnpacked {
        value: MultiMatch::Even,
    };
    let packed = even.pack();
    assert_eq!(packed.0, [0]); // Should use 0 as representative

    let odd = RepresentativeUnpacked { value: MultiMatch::Odd };
    let packed = odd.pack();
    assert_eq!(packed.0, [1]); // Should use 1 as representative

    let high = RepresentativeUnpacked {
        value: MultiMatch::High,
    };
    let packed = high.pack();
    assert_eq!(packed.0, [6]); // Should use 6 as representative

    // Test that all matching values deserialize to the same variant
    for value in [0, 2, 4] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::Even);
    }

    for value in [1, 3, 5] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::Odd);
    }

    for value in [6, 7] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::High);
    }
}
