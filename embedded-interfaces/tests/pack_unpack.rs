#![allow(clippy::bool_assert_comparison)]

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

    // Test read accessors
    assert_eq!(packed.read_f1(), false);
    assert_eq!(packed.read_f2(), true);
    assert_eq!(packed.read_f3(), false);
    assert_eq!(packed.read_f4(), true);
    assert_eq!(packed.read_f5(), true);
    assert_eq!(packed.read_f8(), true);

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
    );

    // Test write accessors
    let mut packed = StructUnpacked::default().pack();
    packed.write_f1(true);
    packed.write_f2(false);
    packed.write_f8(false);
    assert_eq!(packed.read_f1(), true);
    assert_eq!(packed.read_f2(), false);
    assert_eq!(packed.read_f8(), false);

    // Test chainable with accessors
    let packed = StructUnpacked::default()
        .pack()
        .with_f1(true)
        .with_f2(false)
        .with_f8(false);
    assert_eq!(packed.read_f1(), true);
    assert_eq!(packed.read_f2(), false);
    assert_eq!(packed.read_f8(), false);
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

        struct Split(size = 1) {
            f1: u8[7, 0..4] = 0b11001,
            _: u8[4..7],
        }
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0u8, 0x49]);
    assert_eq!(packed.unpack(), StructUnpacked::default());

    // Test read accessors
    assert_eq!(packed.read_f1(), 0);
    assert_eq!(packed.read_f2(), 0x49);

    let packed = NarrowUnpacked::default().pack();
    assert_eq!(packed.0, [0b01011011u8]);
    assert_eq!(packed.unpack(), NarrowUnpacked::default());

    // Test read accessors for narrow fields
    assert_eq!(packed.read_f1(), 0b010);
    assert_eq!(packed.read_f2(), 0b11011);

    let packed = SplitUnpacked::default().pack();
    assert_eq!(packed.0, [0b10010001]);
    assert_eq!(packed.unpack(), SplitUnpacked::default());

    // Test write accessors
    let mut packed = StructUnpacked::default().pack();
    packed.write_f1(0xAB);
    packed.write_f2(0xCD);
    assert_eq!(packed.read_f1(), 0xAB);
    assert_eq!(packed.read_f2(), 0xCD);

    // Test chainable accessors
    let packed = NarrowUnpacked::default().pack().with_f1(0b111).with_f2(0b00001);
    assert_eq!(packed.read_f1(), 0b111);
    assert_eq!(packed.read_f2(), 0b00001);
}

#[test]
fn test_endianness() {
    interface_objects! {
        struct Struct(size = 8) {
            f1: u32{be} = 0x12345678,
            f2: u32{le} = 0x12345678,
        }

        struct Struct24(size = 6) {
            f1: u32{24,be} = 0x123456,
            f2: u32{24,le} = 0x123456,
        }
    }

    let packed = StructUnpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34, 0x56, 0x78, 0x78, 0x56, 0x34, 0x12]);
    assert_eq!(packed.unpack(), StructUnpacked::default());

    let packed = Struct24Unpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34, 0x56, 0x56, 0x34, 0x12]);
    assert_eq!(packed.unpack(), Struct24Unpacked::default());
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

    // Test read accessors
    assert_eq!(packed.read_f1(), 0);
    assert_eq!(packed.read_f2(), 0x49);

    // Test write accessors
    let mut packed = StructUnpacked::default().pack();
    packed.write_f1(0xAB);
    packed.write_f2(0x25);
    assert_eq!(packed.read_f1(), 0xAB);
    assert_eq!(packed.read_f2(), 0x25);
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
    assert_eq!(packed.read_f1(), 0b1010);
    assert_eq!(packed.read_f2(), 0b0101);

    let packed = MediumUnpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34]);
    assert_eq!(packed.unpack(), MediumUnpacked::default());
    assert_eq!(packed.read_f1(), 0x1234);

    let packed = LargeUnpacked::default().pack();
    assert_eq!(packed.0, [0x12, 0x34, 0x56, 0x78]);
    assert_eq!(packed.unpack(), LargeUnpacked::default());
    assert_eq!(packed.read_f1(), 0x12345678);

    let packed = MixedUnpacked::default().pack();
    // f1 (4 bits): 0b1100 = 0xC0 (top 4 bits)
    // f2 (16 bits): 0xABCD
    // f3 (4 bits): 0b0011 = 0x30 (top 4 bits of last byte)
    assert_eq!(packed.0, [0xCA, 0xBC, 0xD3]);
    assert_eq!(packed.unpack(), MixedUnpacked::default());
    assert_eq!(packed.read_f1(), 0b1100);
    assert_eq!(packed.read_f2(), 0xABCD);
    assert_eq!(packed.read_f3(), 0b0011);

    // Test write accessors
    let packed = MixedUnpacked::default()
        .pack()
        .with_f1(0b0011)
        .with_f2(0x1234)
        .with_f3(0b1100);
    assert_eq!(packed.read_f1(), 0b0011);
    assert_eq!(packed.read_f2(), 0x1234);
    assert_eq!(packed.read_f3(), 0b1100);
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

    // Test read accessors
    assert_eq!(packed.read_f1(), 0b1010);
    assert_eq!(packed.read_f2(), 0xFF);
    assert_eq!(packed.read_f3(), 0b0101);

    // Test write and chain
    let packed = CrossBoundaryUnpacked::default()
        .pack()
        .with_f1(0b0001)
        .with_f2(0x00)
        .with_f3(0b1111);
    assert_eq!(packed.read_f1(), 0b0001);
    assert_eq!(packed.read_f2(), 0x00);
    assert_eq!(packed.read_f3(), 0b1111);
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
    assert_eq!(packed.read_f1(), -1);

    let packed = Signed16Unpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF]);
    assert_eq!(packed.unpack(), Signed16Unpacked::default());
    assert_eq!(packed.read_f1(), -1);

    let packed = Signed32Unpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(packed.unpack(), Signed32Unpacked::default());
    assert_eq!(packed.read_f1(), -1);

    let packed = SignedNarrowUnpacked::default().pack();
    assert_eq!(packed.0, [0b11110011u8]);
    assert_eq!(packed.unpack(), SignedNarrowUnpacked::default());
    assert_eq!(packed.read_f1(), -1);
    assert_eq!(packed.read_f2(), 3);

    let packed = SignedMixedUnpacked::default().pack();
    // f1: -2 in 6 bits = 0b111110 (top 6 bits)
    // f2: -1 in 10 bits = 0b1111111111 (remaining 10 bits)
    assert_eq!(packed.0, [0b11111011, 0b11111111]);
    assert_eq!(packed.unpack(), SignedMixedUnpacked::default());
    assert_eq!(packed.read_f1(), -2);
    assert_eq!(packed.read_f2(), -1);

    // Test write accessors for signed values
    let packed = SignedNarrowUnpacked::default()
        .pack()
        .with_f1(7) // Max positive for 4 bits
        .with_f2(-8); // Max negative for 4 bits
    assert_eq!(packed.read_f1(), 7);
    assert_eq!(packed.read_f2(), -8);
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

    // Test read accessors
    assert_eq!(packed.read_f1(), 7);
    assert_eq!(packed.read_f2(), -8);
    assert_eq!(packed.read_f3(), 0);
    assert_eq!(packed.read_f4(), -1);

    // Test write accessors
    let packed = SignedEdgeUnpacked::default()
        .pack()
        .with_f1(-1)
        .with_f2(7)
        .with_f3(-8)
        .with_f4(0);
    assert_eq!(packed.read_f1(), -1);
    assert_eq!(packed.read_f2(), 7);
    assert_eq!(packed.read_f3(), -8);
    assert_eq!(packed.read_f4(), 0);
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
    assert_eq!(packed.read_f1(), 1.0f32);

    let packed = Float64Unpacked::default().pack();
    assert_eq!(packed.0, 1.0f64.to_be_bytes());
    assert_eq!(packed.unpack(), Float64Unpacked::default());
    assert_eq!(packed.read_f1(), 1.0f64);

    let packed = MixedFloatUnpacked::default().pack();
    let mut expected = [0u8; 5];
    expected[0..4].copy_from_slice(&1.0f32.to_be_bytes());
    expected[4] = 0xFF;
    assert_eq!(packed.0, expected);
    assert_eq!(packed.unpack(), MixedFloatUnpacked::default());
    assert_eq!(packed.read_f1(), 1.0f32);
    assert_eq!(packed.read_f2(), 0xFF);

    // Test write accessors for floats
    let packed = MixedFloatUnpacked::default().pack().with_f1(1.23f32).with_f2(0x42);
    assert_eq!(packed.read_f1(), 1.23f32);
    assert_eq!(packed.read_f2(), 0x42);
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
            _: u8{2} = 0b11,
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
    assert_eq!(packed.read_data(), [0x01, 0x02, 0x03, 0x05]);

    let packed = ArrayShiftedUnpacked::default().pack();
    // prefix: true = 0b1 in bit 0
    // data: [0x01, 0x02, 0x03, 0x05] starting at bit 1
    assert_eq!(packed.0, [0x80, 0x81, 0x01, 0x82, 0x80]);
    assert_eq!(packed.unpack(), ArrayShiftedUnpacked::default());
    assert_eq!(packed.read_prefix(), true);
    assert_eq!(packed.read_data(), [0x01, 0x02, 0x03, 0x05]);

    let packed = ArrayShifted2Unpacked::default().pack();
    // prefix: 0b11 in bit 0 and 1
    // data: [0x01, 0x02, 0x03, 0x05] starting at bit 2
    assert_eq!(packed.0, [0xc0, 0x40, 0x80, 0xc1, 0x40]);
    assert_eq!(packed.unpack(), ArrayShifted2Unpacked::default());
    assert_eq!(packed.read_data(), [0x01, 0x02, 0x03, 0x05]);

    let packed = ArrayMixedUnpacked::default().pack();
    // prefix: 0xA = 0b1010 in bits 0-3
    // data: [0x01, 0x02, 0x03, 0x05] in bits 4-35
    // suffix: 0xB = 0b1011 in bits 36-39
    assert_eq!(packed.0, [0xa0, 0x10, 0x20, 0x30, 0x5b]);
    assert_eq!(packed.unpack(), ArrayMixedUnpacked::default());
    assert_eq!(packed.read_prefix(), 0xa);
    assert_eq!(packed.read_data(), [0x01, 0x02, 0x03, 0x05]);
    assert_eq!(packed.read_suffix(), 0xb);

    // Test write accessors for arrays
    let packed = ArrayMixedUnpacked::default()
        .pack()
        .with_prefix(0x5)
        .with_data([0xAA, 0xBB, 0xCC, 0xDD])
        .with_suffix(0x3);
    assert_eq!(packed.read_prefix(), 0x5);
    assert_eq!(packed.read_data(), [0xAA, 0xBB, 0xCC, 0xDD]);
    assert_eq!(packed.read_suffix(), 0x3);
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
    assert_eq!(packed.read_data(), [0x1234, 0x5678]);

    let packed = ArrayShiftedUnpacked::default().pack();
    // prefix: true = 0b1 in bit 0
    // data: [0x1234, 0x5678] starting at bit 1
    assert_eq!(packed.0, [0x89, 0x1a, 0x2b, 0x3c, 0x0]);
    assert_eq!(packed.unpack(), ArrayShiftedUnpacked::default());
    assert_eq!(packed.read_prefix(), true);
    assert_eq!(packed.read_data(), [0x1234, 0x5678]);

    // Test write accessors
    let packed = ArrayShiftedUnpacked::default()
        .pack()
        .with_prefix(false)
        .with_data([0xABCD, 0xEF01]);
    assert_eq!(packed.read_prefix(), false);
    assert_eq!(packed.read_data(), [0xABCD, 0xEF01]);
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
    assert_eq!(
        packed.read_flags(),
        [true, false, true, false, true, false, true, false]
    );

    let packed = BoolArrayShiftedUnpacked::default().pack();
    // prefix: 0xF = 0b1111 in bits 0-3
    // flags: [T,F,T,F,T,F,T,F] = 0b10101010 in bits 4-11
    // suffix: 0xA = 0b1010 in bits 12-15
    assert_eq!(packed.0, [0xFA, 0xAA]);
    assert_eq!(packed.unpack(), BoolArrayShiftedUnpacked::default());
    assert_eq!(packed.read_prefix(), 0xF);
    assert_eq!(
        packed.read_flags(),
        [true, false, true, false, true, false, true, false]
    );
    assert_eq!(packed.read_suffix(), 0xA);

    // Test write accessors for bool arrays
    let packed = BoolArrayShiftedUnpacked::default()
        .pack()
        .with_prefix(0x5)
        .with_flags([false, true, false, true, false, true, false, true])
        .with_suffix(0x3);
    assert_eq!(packed.read_prefix(), 0x5);
    assert_eq!(
        packed.read_flags(),
        [false, true, false, true, false, true, false, true]
    );
    assert_eq!(packed.read_suffix(), 0x3);
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
    assert_eq!(packed.read_data(), [-1, 1000]);

    // Test write accessors
    let packed = SignedArrayUnpacked::default().pack().with_data([500, -2000]);
    assert_eq!(packed.read_data(), [500, -2000]);
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
    assert_eq!(packed.read_data(), [1.0, -1.0]);

    // Test write accessors
    let packed = FloatArrayUnpacked::default().pack().with_data([1.23, 2.71]);
    assert_eq!(packed.read_data(), [1.23, 2.71]);
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

    // Test read accessors
    assert_eq!(packed.read_header(), 0xA);
    assert_eq!(packed.read_flags(), [true, false, true, false]);
    assert_eq!(packed.read_id(), 0x1234);
    assert_eq!(packed.read_data(), [0x11, 0x22, 0x33]);
    assert_eq!(packed.read_checksum(), 0xFF);

    // Test write accessors
    let mut packed = ComplexUnpacked::default().pack();
    packed.write_header(0x5);
    packed.write_flags([false, true, false, true]);
    packed.write_id(0xABCD);
    packed.write_data([0xAA, 0xBB, 0xCC]);
    packed.write_checksum(0x00);

    assert_eq!(packed.read_header(), 0x5);
    assert_eq!(packed.read_flags(), [false, true, false, true]);
    assert_eq!(packed.read_id(), 0xABCD);
    assert_eq!(packed.read_data(), [0xAA, 0xBB, 0xCC]);
    assert_eq!(packed.read_checksum(), 0x00);

    // Test chainable accessors
    let packed = ComplexUnpacked::default()
        .pack()
        .with_header(0x3)
        .with_flags([true, true, false, false])
        .with_id(0x9876)
        .with_data([0x44, 0x55, 0x66])
        .with_checksum(0x42);

    assert_eq!(packed.read_header(), 0x3);
    assert_eq!(packed.read_flags(), [true, true, false, false]);
    assert_eq!(packed.read_id(), 0x9876);
    assert_eq!(packed.read_data(), [0x44, 0x55, 0x66]);
    assert_eq!(packed.read_checksum(), 0x42);
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

    // Test read accessors
    assert_eq!(packed1.read_f1(), 0b1010);
    assert_eq!(packed1.read_f2(), -2);
    assert_eq!(packed1.read_f3(), true);
    assert_eq!(packed1.read_f4(), 0x3FF);
    assert_eq!(packed1.read_f5(), [true, false, true, false, true]);
    assert_eq!(packed1.read_f6(), 0xAB);

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

    // Test read accessors with modified values
    assert_eq!(packed1.read_f1(), 0b0101);
    assert_eq!(packed1.read_f2(), 3);
    assert_eq!(packed1.read_f3(), false);
    assert_eq!(packed1.read_f4(), 0x200);
    assert_eq!(packed1.read_f5(), [false, true, false, true, false]);
    assert_eq!(packed1.read_f6(), 0x12);

    // Test write accessors
    let packed = RoundTripUnpacked::default()
        .pack()
        .with_f1(0b1111)
        .with_f2(-8)
        .with_f3(false)
        .with_f4(0x100)
        .with_f5([true, true, true, true, true])
        .with_f6(0xFF);

    assert_eq!(packed.read_f1(), 0b1111);
    assert_eq!(packed.read_f2(), -8);
    assert_eq!(packed.read_f3(), false);
    assert_eq!(packed.read_f4(), 0x100);
    assert_eq!(packed.read_f5(), [true, true, true, true, true]);
    assert_eq!(packed.read_f6(), 0xFF);
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

    // Test read accessors
    assert_eq!(packed.read_f1(), 1);
    assert_eq!(packed.read_f2(), 0x7F);
    assert_eq!(packed.read_f3(), 0);
    assert_eq!(packed.read_f4(), 0x01);

    // Test write accessors
    let packed = EdgeCaseUnpacked::default()
        .pack()
        .with_f1(0)
        .with_f2(0x55)
        .with_f3(1)
        .with_f4(0x7F);

    assert_eq!(packed.read_f1(), 0);
    assert_eq!(packed.read_f2(), 0x55);
    assert_eq!(packed.read_f3(), 1);
    assert_eq!(packed.read_f4(), 0x7F);
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

    // Test read accessors
    assert_eq!(packed.read_f1(), 0x123456789ABCDEF0);
    assert_eq!(packed.read_f2(), 0xFEDCBA9876543210);

    // Test write accessors
    let packed = LargeUnpacked::default()
        .pack()
        .with_f1(0xAAAAAAAAAAAAAAAA)
        .with_f2(0x5555555555555555);

    assert_eq!(packed.read_f1(), 0xAAAAAAAAAAAAAAAA);
    assert_eq!(packed.read_f2(), 0x5555555555555555);
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
    assert_eq!(packed.read_matrix(), [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]);

    // Test bool matrix (4x8 = 32 bits = 4 bytes)
    let packed = BoolMatrixUnpacked::default().pack();
    assert_eq!(packed.0, [0xFF, 0xFF, 0xFF, 0xFF]);
    assert_eq!(packed.read_flags(), [[true; 8]; 4]);

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
    assert_eq!(
        packed.read_temps(),
        [[-10, 0, 10], [-20, 0, 20], [-30, 0, 30], [-40, 0, 40]]
    );

    // Test mixed nested
    let packed = MixedNestedUnpacked::default().pack();
    let expected = [
        0xAA, // header
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, // data[3][2]
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, // checksums[4]
    ];
    assert_eq!(packed.0, expected);
    assert_eq!(packed.read_header(), 0xAA);
    assert_eq!(packed.read_data(), [[0x11, 0x22], [0x33, 0x44], [0x55, 0x66]]);
    assert_eq!(packed.read_checksums(), [0x1234, 0x5678, 0x9ABC, 0xDEF0]);

    // Test 3D cube (2x2x2x8 = 32 bits = 4 bytes)
    let packed = CubeUnpacked::default().pack();
    assert_eq!(packed.0, [1, 2, 3, 4, 5, 6, 7, 8]);
    let unpacked = packed.unpack();
    assert_eq!(unpacked.cube, [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]);
    assert_eq!(packed.read_cube(), [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]);

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
    assert_eq!(
        packed.read_matrix(),
        [
            [0xAA, 0xBB, 0xCC],
            [0xDD, 0xEE, 0xFF],
            [0x11, 0x22, 0x33],
            [0x44, 0x55, 0x66]
        ]
    );

    // Test write accessors for nested arrays
    let packed = MixedNestedUnpacked::default()
        .pack()
        .with_header(0x42)
        .with_data([[0xAA, 0xBB], [0xCC, 0xDD], [0xEE, 0xFF]])
        .with_checksums([0x1111, 0x2222, 0x3333, 0x4444]);

    assert_eq!(packed.read_header(), 0x42);
    assert_eq!(packed.read_data(), [[0xAA, 0xBB], [0xCC, 0xDD], [0xEE, 0xFF]]);
    assert_eq!(packed.read_checksums(), [0x1111, 0x2222, 0x3333, 0x4444]);
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
    assert_eq!(packed.read_hyper()[0][0][0][0], 1);
    assert_eq!(packed.read_hyper()[3][1][1][1], 32);

    // Test bool tensor (8x2x2x1 = 32 bits = 4 bytes)
    let packed = BoolTensorUnpacked::default().pack();
    let unpacked = packed.unpack();
    assert_eq!(unpacked.tensor[0], [[true, false], [false, true]]);
    assert_eq!(unpacked.tensor[7], [[false, false], [false, false]]);
    assert_eq!(packed.read_tensor()[0], [[true, false], [false, true]]);
    assert_eq!(packed.read_tensor()[7], [[false, false], [false, false]]);

    // Test write accessors for deeply nested arrays
    let new_hyper = [
        [[[0xAA, 0xBB], [0xCC, 0xDD]], [[0xEE, 0xFF], [0x11, 0x22]]],
        [[[0x33, 0x44], [0x55, 0x66]], [[0x77, 0x88], [0x99, 0xAA]]],
        [[[0xBB, 0xCC], [0xDD, 0xEE]], [[0xFF, 0x00], [0x11, 0x22]]],
        [[[0x33, 0x44], [0x55, 0x66]], [[0x77, 0x88], [0x99, 0xAA]]],
    ];

    let packed = HyperCubeUnpacked::default().pack().with_hyper(new_hyper);

    assert_eq!(packed.read_hyper(), new_hyper);
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
    assert_eq!(packed.read_mode(), SimpleEnum::First);

    let custom = WithEnumUnpacked {
        mode: SimpleEnum::Second,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [1]);
    assert_eq!(packed.unpack(), custom);
    assert_eq!(packed.read_mode(), SimpleEnum::Second);

    let custom = WithEnumUnpacked {
        mode: SimpleEnum::Third,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [2]);
    assert_eq!(packed.unpack(), custom);
    assert_eq!(packed.read_mode(), SimpleEnum::Third);

    // Test write accessors for enums
    let packed = WithEnumUnpacked::default().pack().with_mode(SimpleEnum::Second);
    assert_eq!(packed.read_mode(), SimpleEnum::Second);

    let mut packed = WithEnumUnpacked::default().pack();
    packed.write_mode(SimpleEnum::Third);
    assert_eq!(packed.read_mode(), SimpleEnum::Third);
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

    // Test read accessors
    assert_eq!(packed.read_mode(), Mode::HighPower);
    assert_eq!(packed.read_flags(), 0b10101);

    let custom = DeviceUnpacked {
        mode: Mode::Sleep,
        flags: 0b11111,
    };
    let packed = custom.pack();
    // mode: 0b011 in bits 0-2, flags: 0b11111 in bits 3-7
    assert_eq!(packed.0, [0b00111111]);
    assert_eq!(packed.unpack(), custom);

    // Test read accessors with custom values
    assert_eq!(packed.read_mode(), Mode::Sleep);
    assert_eq!(packed.read_flags(), 0b11111);

    // Test write accessors
    let mut packed = DeviceUnpacked::default().pack();
    packed.write_mode(Mode::Debug);
    packed.write_flags(0b01010);
    assert_eq!(packed.read_mode(), Mode::Debug);
    assert_eq!(packed.read_flags(), 0b01010);
    assert_eq!(packed.0, [0b10001010]);

    // Test chainable with accessors
    let packed = DeviceUnpacked::default()
        .pack()
        .with_mode(Mode::Test)
        .with_flags(0b00001);
    assert_eq!(packed.read_mode(), Mode::Test);
    assert_eq!(packed.read_flags(), 0b00001);
    assert_eq!(packed.0, [0b10100001]);
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
    assert_eq!(packed.read_status(), Status::Off);

    // Test range variants with accessors
    let custom = SensorUnpacked { status: Status::Low(2) };
    let packed = custom.pack();
    assert_eq!(packed.0, [2]);
    let unpacked = packed.unpack();
    if let Status::Low(val) = unpacked.status {
        assert_eq!(val, 2);
    } else {
        panic!("Expected Low variant");
    }
    // Test read accessor
    if let Status::Low(val) = packed.read_status() {
        assert_eq!(val, 2);
    } else {
        panic!("Expected Low variant from read_status");
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
    // Test read accessor
    if let Status::High(val) = packed.read_status() {
        assert_eq!(val, 10);
    } else {
        panic!("Expected High variant from read_status");
    }

    // Test write accessor
    let mut packed = SensorUnpacked::default().pack();
    packed.write_status(Status::Error(15));
    if let Status::Error(val) = packed.read_status() {
        assert_eq!(val, 15);
    } else {
        panic!("Expected Error variant from write_status");
    }

    // Test chainable with accessor
    let packed = SensorUnpacked::default().pack().with_status(Status::Critical);
    assert_eq!(packed.read_status(), Status::Critical);
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
    assert_eq!(packed.read_protocol(), Protocol::None);

    let custom = PacketUnpacked {
        protocol: Protocol::TCP,
    };
    let packed = custom.pack();
    assert_eq!(packed.0, [1 << 5]);
    assert_eq!(packed.unpack(), custom);
    assert_eq!(packed.read_protocol(), Protocol::TCP);

    // Test wildcard matching with accessor
    let packet = Packet([5 << 5]); // 5 should match wildcard
    let unpacked = packet.unpack();
    if let Protocol::Unknown(val) = unpacked.protocol {
        assert_eq!(val, 5);
    } else {
        panic!("Expected Unknown variant");
    }
    if let Protocol::Unknown(val) = packet.read_protocol() {
        assert_eq!(val, 5);
    } else {
        panic!("Expected Unknown variant from read_protocol");
    }

    // Test write with wildcard
    let mut packet = PacketUnpacked::default().pack();
    packet.write_protocol(Protocol::Unknown(7));
    if let Protocol::Unknown(val) = packet.read_protocol() {
        assert_eq!(val, 7);
    } else {
        panic!("Expected Unknown variant from write_protocol");
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
    assert_eq!(packed.read_cmd(), Command::Nop);

    // Test or patterns with accessors
    let mut packed = MessageUnpacked::default().pack();
    packed.write_cmd(Command::Odd(3));
    if let Command::Odd(val) = packed.read_cmd() {
        assert_eq!(val, 3);
    } else {
        panic!("Expected Odd variant from write_cmd");
    }

    // Test chainable with accessor
    let packed = MessageUnpacked::default().pack().with_cmd(Command::Even(4));
    if let Command::Even(val) = packed.read_cmd() {
        assert_eq!(val, 4);
    } else {
        panic!("Expected Even variant from with_cmd");
    }

    // Test range with accessor
    let packed = MessageUnpacked::default().pack().with_cmd(Command::Range(9));
    if let Command::Range(val) = packed.read_cmd() {
        assert_eq!(val, 9);
    } else {
        panic!("Expected Range variant from with_cmd");
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
    // Test read accessor
    if let FullRange::Valid(val) = packed.read_value() {
        assert_eq!(val, 42);
    } else {
        panic!("Expected Valid variant from read_value");
    }

    // Test invalid range with write accessor
    let mut packed = DataUnpacked::default().pack();
    packed.write_value(FullRange::Invalid(200));
    if let FullRange::Invalid(val) = packed.read_value() {
        assert_eq!(val, 200);
    } else {
        panic!("Expected Invalid variant from write_value");
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
            0..=511 Low(i32), // Test type widening
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

    // Test read accessors
    assert_eq!(packed.read_small(), Small::A);
    if let Medium::Low(val) = packed.read_medium() {
        assert_eq!(val, 100);
    } else {
        panic!("Expected Low variant from read_medium");
    }
    assert_eq!(packed.read_large(), Large::Zero);

    // Test with write accessors
    let mut packed = MultiEnumUnpacked::default().pack();
    packed.write_small(Small::D);
    packed.write_medium(Medium::High(700));
    packed.write_large(Large::NonZero(12345));

    assert_eq!(packed.read_small(), Small::D);
    if let Medium::High(val) = packed.read_medium() {
        assert_eq!(val, 700);
    } else {
        panic!("Expected High variant from write_medium");
    }
    if let Large::NonZero(val) = packed.read_large() {
        assert_eq!(val, 12345);
    } else {
        panic!("Expected NonZero variant from write_large");
    }

    // Test chainable accessors
    let packed = MultiEnumUnpacked::default()
        .pack()
        .with_small(Small::C)
        .with_medium(Medium::Low(256))
        .with_large(Large::NonZero(54321));

    assert_eq!(packed.read_small(), Small::C);
    if let Medium::Low(val) = packed.read_medium() {
        assert_eq!(val, 256);
    } else {
        panic!("Expected Low variant from with_medium");
    }
    if let Large::NonZero(val) = packed.read_large() {
        assert_eq!(val, 54321);
    } else {
        panic!("Expected NonZero variant from with_large");
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

    // Test read accessors
    assert_eq!(packed.read_id(), 0x1234);
    assert_eq!(packed.read_priority(), Priority::Low);
    assert_eq!(packed.read_flags(), [true, false, true, false, true]);

    // Test write accessors
    let mut packed = TaskUnpacked::default().pack();
    packed.write_id(0x5678);
    packed.write_priority(Priority::High(5));
    packed.write_flags([false, true, false, true, false]);

    assert_eq!(packed.read_id(), 0x5678);
    if let Priority::High(val) = packed.read_priority() {
        assert_eq!(val, 5);
    } else {
        panic!("Expected High variant from write_priority");
    }
    assert_eq!(packed.read_flags(), [false, true, false, true, false]);

    // Test chainable accessors
    let packed = TaskUnpacked::default()
        .pack()
        .with_id(0xABCD)
        .with_priority(Priority::Medium(2))
        .with_flags([true, true, false, false, true]);

    assert_eq!(packed.read_id(), 0xABCD);
    if let Priority::Medium(val) = packed.read_priority() {
        assert_eq!(val, 2);
    } else {
        panic!("Expected Medium variant from with_priority");
    }
    assert_eq!(packed.read_flags(), [true, true, false, false, true]);
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

        // Test accessor roundtrip
        match (original_var, packed.read_var()) {
            (Variable::None, Variable::None) => {}
            (Variable::Small(a), Variable::Small(b)) => assert_eq!(a, b),
            (Variable::Large(a), Variable::Large(b)) => assert_eq!(a, b),
            _ => panic!("Enum variant mismatch after accessor roundtrip"),
        }

        // Test write accessor roundtrip
        let mut test_packed = ContainerUnpacked::default().pack();
        test_packed.write_var(original_var);
        match (original_var, test_packed.read_var()) {
            (Variable::None, Variable::None) => {}
            (Variable::Small(a), Variable::Small(b)) => assert_eq!(a, b),
            (Variable::Large(a), Variable::Large(b)) => assert_eq!(a, b),
            _ => panic!("Enum variant mismatch after write accessor roundtrip"),
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
    assert_eq!(packed.read_binary(), Binary::False);
    assert_eq!(packed.read_quaternary(), Quaternary::Zero);

    // Test all combinations with accessors
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

        // Test read accessors
        assert_eq!(packed.read_binary(), binary);
        assert_eq!(packed.read_quaternary(), quaternary);

        // Test write accessors
        let mut test_packed = ExhaustiveUnpacked::default().pack();
        test_packed.write_binary(binary);
        test_packed.write_quaternary(quaternary);
        assert_eq!(test_packed.read_binary(), binary);
        assert_eq!(test_packed.read_quaternary(), quaternary);

        // Test chainable accessors
        let chained_packed = ExhaustiveUnpacked::default()
            .pack()
            .with_binary(binary)
            .with_quaternary(quaternary);
        assert_eq!(chained_packed.read_binary(), binary);
        assert_eq!(chained_packed.read_quaternary(), quaternary);
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

    // Test that serialization uses representative values with accessors
    let even = RepresentativeUnpacked {
        value: MultiMatch::Even,
    };
    let packed = even.pack();
    assert_eq!(packed.0, [0]); // Should use 0 as representative
    assert_eq!(packed.read_value(), MultiMatch::Even);

    // Test write accessor uses representative values
    let mut packed = RepresentativeUnpacked::default().pack();
    packed.write_value(MultiMatch::Odd);
    assert_eq!(packed.0, [1]); // Should use 1 as representative
    assert_eq!(packed.read_value(), MultiMatch::Odd);

    // Test chainable accessor
    let packed = RepresentativeUnpacked::default().pack().with_value(MultiMatch::High);
    assert_eq!(packed.0, [6]); // Should use 6 as representative
    assert_eq!(packed.read_value(), MultiMatch::High);

    // Test that all matching values deserialize to the same variant
    for value in [0, 2, 4] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::Even);
        assert_eq!(Representative([value]).read_value(), MultiMatch::Even);
    }

    for value in [1, 3, 5] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::Odd);
        assert_eq!(Representative([value]).read_value(), MultiMatch::Odd);
    }

    for value in [6, 7] {
        let unpacked = Representative([value]).unpack();
        assert_eq!(unpacked.value, MultiMatch::High);
        assert_eq!(Representative([value]).read_value(), MultiMatch::High);
    }
}

#[test]
fn test_basic_custom_struct_embedding() {
    interface_objects! {
        struct Inner(size = 2) {
            a: u8{3} = 0b101,
            b: u16{9} = 0x123,
            c: u8{4} = 0b1010,
        }

        struct Outer(size = 2) {
            inner: InnerUnpacked = InnerUnpacked { a: 5, b: 0x123, c: 10 },
        }
    }

    let packed = OuterUnpacked::default().pack();
    // Inner should pack to 2 bytes with the given bit layout
    let inner_packed = InnerUnpacked { a: 5, b: 0x123, c: 10 }.pack();
    assert_eq!(packed.0[0..2], inner_packed.0);
    assert_eq!(packed.0, [inner_packed.0[0], inner_packed.0[1]]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.inner.a, 5);
    assert_eq!(unpacked.inner.b, 0x123);
    assert_eq!(unpacked.inner.c, 10);

    // Test nested read accessors
    let inner_read = packed.read_inner();
    assert_eq!(inner_read.0, inner_packed.0);

    let inner_unpacked = packed.read_inner_unpacked();
    assert_eq!(inner_unpacked.a, 5);
    assert_eq!(inner_unpacked.b, 0x123);
    assert_eq!(inner_unpacked.c, 10);

    // Test promoted nested accessors
    assert_eq!(packed.read_inner_a(), 5);
    assert_eq!(packed.read_inner_b(), 0x123);
    assert_eq!(packed.read_inner_c(), 10);

    // Test write accessors
    let mut packed = OuterUnpacked::default().pack();
    packed.write_inner_a(7);
    packed.write_inner_b(0x56);
    packed.write_inner_c(12);

    assert_eq!(packed.read_inner_a(), 7);
    assert_eq!(packed.read_inner_b(), 0x56);
    assert_eq!(packed.read_inner_c(), 12);

    // Test chainable accessors
    let packed = OuterUnpacked::default()
        .pack()
        .with_inner_a(3)
        .with_inner_b(0x78)
        .with_inner_c(8);

    assert_eq!(packed.read_inner_a(), 3);
    assert_eq!(packed.read_inner_b(), 0x78);
    assert_eq!(packed.read_inner_c(), 8);

    // Test writing the whole inner struct
    let mut packed = OuterUnpacked::default().pack();
    let new_inner = InnerUnpacked { a: 2, b: 0xAB, c: 6 };
    packed.write_inner_unpacked(new_inner);

    let read_inner = packed.read_inner_unpacked();
    assert_eq!(read_inner.a, 2);
    assert_eq!(read_inner.b, 0xAB);
    assert_eq!(read_inner.c, 6);
}

#[test]
fn test_multiple_custom_structs_order() {
    interface_objects! {
        struct Point(size = 2) {
            x: u8 = 10,
            y: u8 = 20,
        }

        struct Pixel(size = 5) {
            pos: PointUnpacked = PointUnpacked { x: 10, y: 20 },
            color: ColorUnpacked = ColorUnpacked { r: 255, g: 128, b: 64 },
        }

        struct Color(size = 3) {
            r: u8 = 0xFF,
            g: u8 = 0x80,
            b: u8 = 0x40,
        }
    }

    let packed = PixelUnpacked::default().pack();
    assert_eq!(packed.0, [10, 20, 0xFF, 0x80, 0x40]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.pos.x, 10);
    assert_eq!(unpacked.pos.y, 20);
    assert_eq!(unpacked.color.r, 255);
    assert_eq!(unpacked.color.g, 128);
    assert_eq!(unpacked.color.b, 64);

    // Test nested read accessors
    assert_eq!(packed.read_pos_x(), 10);
    assert_eq!(packed.read_pos_y(), 20);
    assert_eq!(packed.read_color_r(), 255);
    assert_eq!(packed.read_color_g(), 128);
    assert_eq!(packed.read_color_b(), 64);

    let pos_read = packed.read_pos_unpacked();
    assert_eq!(pos_read.x, 10);
    assert_eq!(pos_read.y, 20);

    let color_read = packed.read_color_unpacked();
    assert_eq!(color_read.r, 255);
    assert_eq!(color_read.g, 128);
    assert_eq!(color_read.b, 64);

    // Test write accessors
    let mut packed = PixelUnpacked::default().pack();
    packed.write_pos_x(100);
    packed.write_pos_y(200);
    packed.write_color_r(0x11);
    packed.write_color_g(0x22);
    packed.write_color_b(0x33);

    assert_eq!(packed.read_pos_x(), 100);
    assert_eq!(packed.read_pos_y(), 200);
    assert_eq!(packed.read_color_r(), 0x11);
    assert_eq!(packed.read_color_g(), 0x22);
    assert_eq!(packed.read_color_b(), 0x33);

    // Test chainable accessors
    let packed = PixelUnpacked::default()
        .pack()
        .with_pos_x(50)
        .with_pos_y(75)
        .with_color_r(0xAA)
        .with_color_g(0xBB)
        .with_color_b(0xCC);

    assert_eq!(packed.read_pos_x(), 50);
    assert_eq!(packed.read_pos_y(), 75);
    assert_eq!(packed.read_color_r(), 0xAA);
    assert_eq!(packed.read_color_g(), 0xBB);
    assert_eq!(packed.read_color_b(), 0xCC);

    // Test with custom values using struct-level accessors
    let new_pos = PointUnpacked { x: 150, y: 250 };
    let new_color = ColorUnpacked {
        r: 0x44,
        g: 0x55,
        b: 0x66,
    };

    let packed = PixelUnpacked::default()
        .pack()
        .with_pos_unpacked(new_pos)
        .with_color_unpacked(new_color);

    assert_eq!(packed.read_pos_unpacked(), new_pos);
    assert_eq!(packed.read_color_unpacked(), new_color);
}

#[test]
fn test_nested_custom_structs() {
    interface_objects! {
        struct Point(size = 2) {
            x: u8 = 1,
            y: u8 = 2,
        }

        struct Rectangle(size = 4) {
            top_left: PointUnpacked = PointUnpacked { x: 1, y: 2 },
            bottom_right: PointUnpacked = PointUnpacked { x: 3, y: 4 },
        }

        struct Scene(size = 8) {
            rect1: RectangleUnpacked = RectangleUnpacked {
                top_left: PointUnpacked { x: 1, y: 2 },
                bottom_right: PointUnpacked { x: 3, y: 4 }
            },
            rect2: RectangleUnpacked = RectangleUnpacked {
                top_left: PointUnpacked { x: 5, y: 6 },
                bottom_right: PointUnpacked { x: 7, y: 8 }
            },
        }
    }

    let packed = SceneUnpacked::default().pack();
    assert_eq!(packed.0, [1, 2, 3, 4, 5, 6, 7, 8]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.rect1.top_left.x, 1);
    assert_eq!(unpacked.rect1.top_left.y, 2);
    assert_eq!(unpacked.rect1.bottom_right.x, 3);
    assert_eq!(unpacked.rect1.bottom_right.y, 4);
    assert_eq!(unpacked.rect2.top_left.x, 5);
    assert_eq!(unpacked.rect2.top_left.y, 6);
    assert_eq!(unpacked.rect2.bottom_right.x, 7);
    assert_eq!(unpacked.rect2.bottom_right.y, 8);

    // Test deeply nested read accessors
    assert_eq!(packed.read_rect1_top_left_x(), 1);
    assert_eq!(packed.read_rect1_top_left_y(), 2);
    assert_eq!(packed.read_rect1_bottom_right_x(), 3);
    assert_eq!(packed.read_rect1_bottom_right_y(), 4);
    assert_eq!(packed.read_rect2_top_left_x(), 5);
    assert_eq!(packed.read_rect2_top_left_y(), 6);
    assert_eq!(packed.read_rect2_bottom_right_x(), 7);
    assert_eq!(packed.read_rect2_bottom_right_y(), 8);

    // Test intermediate level accessors
    let rect1_tl = packed.read_rect1_top_left_unpacked();
    assert_eq!(rect1_tl.x, 1);
    assert_eq!(rect1_tl.y, 2);

    let rect1_br = packed.read_rect1_bottom_right_unpacked();
    assert_eq!(rect1_br.x, 3);
    assert_eq!(rect1_br.y, 4);

    let rect1_full = packed.read_rect1_unpacked();
    assert_eq!(rect1_full.top_left.x, 1);
    assert_eq!(rect1_full.top_left.y, 2);
    assert_eq!(rect1_full.bottom_right.x, 3);
    assert_eq!(rect1_full.bottom_right.y, 4);

    // Test write accessors at different levels
    let mut packed = SceneUnpacked::default().pack();

    // Write individual coordinates
    packed.write_rect1_top_left_x(10);
    packed.write_rect1_top_left_y(20);
    packed.write_rect2_bottom_right_x(80);
    packed.write_rect2_bottom_right_y(90);

    assert_eq!(packed.read_rect1_top_left_x(), 10);
    assert_eq!(packed.read_rect1_top_left_y(), 20);
    assert_eq!(packed.read_rect2_bottom_right_x(), 80);
    assert_eq!(packed.read_rect2_bottom_right_y(), 90);

    // Test chainable accessors
    let packed = SceneUnpacked::default()
        .pack()
        .with_rect1_top_left_x(100)
        .with_rect1_top_left_y(200)
        .with_rect1_bottom_right_x(30)
        .with_rect1_bottom_right_y(40)
        .with_rect2_top_left_x(50)
        .with_rect2_top_left_y(60)
        .with_rect2_bottom_right_x(70)
        .with_rect2_bottom_right_y(80);

    assert_eq!(packed.read_rect1_top_left_x(), 100);
    assert_eq!(packed.read_rect1_top_left_y(), 200);
    assert_eq!(packed.read_rect1_bottom_right_x(), 30);
    assert_eq!(packed.read_rect1_bottom_right_y(), 40);
    assert_eq!(packed.read_rect2_top_left_x(), 50);
    assert_eq!(packed.read_rect2_top_left_y(), 60);
    assert_eq!(packed.read_rect2_bottom_right_x(), 70);
    assert_eq!(packed.read_rect2_bottom_right_y(), 80);

    // Test writing intermediate structs
    let mut packed = SceneUnpacked::default().pack();
    let new_point = PointUnpacked { x: 42, y: 84 };
    packed.write_rect1_top_left_unpacked(new_point);

    let read_point = packed.read_rect1_top_left_unpacked();
    assert_eq!(read_point.x, 42);
    assert_eq!(read_point.y, 84);

    // Test writing full rectangles
    let new_rect = RectangleUnpacked {
        top_left: PointUnpacked { x: 11, y: 22 },
        bottom_right: PointUnpacked { x: 33, y: 44 },
    };
    packed.write_rect2_unpacked(new_rect);

    let read_rect = packed.read_rect2_unpacked();
    assert_eq!(read_rect.top_left.x, 11);
    assert_eq!(read_rect.top_left.y, 22);
    assert_eq!(read_rect.bottom_right.x, 33);
    assert_eq!(read_rect.bottom_right.y, 44);
}

#[test]
fn test_custom_struct_with_arrays() {
    interface_objects! {
        struct RGB(size = 3) {
            values: [u8; 3] = [0xFF, 0x80, 0x40],
        }

        struct Palette(size = 9) {
            primary: RGBUnpacked = RGBUnpacked { values: [255, 128, 64] },
            secondary: RGBUnpacked = RGBUnpacked { values: [64, 128, 255] },
            tertiary: RGBUnpacked = RGBUnpacked { values: [128, 255, 64] },
        }
    }

    let packed = PaletteUnpacked::default().pack();
    assert_eq!(packed.0, [255, 128, 64, 64, 128, 255, 128, 255, 64]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.primary.values, [255, 128, 64]);
    assert_eq!(unpacked.secondary.values, [64, 128, 255]);
    assert_eq!(unpacked.tertiary.values, [128, 255, 64]);

    // Test nested read accessors
    assert_eq!(packed.read_primary_values(), [255, 128, 64]);
    assert_eq!(packed.read_secondary_values(), [64, 128, 255]);
    assert_eq!(packed.read_tertiary_values(), [128, 255, 64]);

    let primary_read = packed.read_primary_unpacked();
    assert_eq!(primary_read.values, [255, 128, 64]);

    // Test write accessors
    let mut packed = PaletteUnpacked::default().pack();
    packed.write_primary_values([100, 101, 102]);
    packed.write_secondary_values([200, 201, 202]);
    packed.write_tertiary_values([50, 51, 52]);

    assert_eq!(packed.read_primary_values(), [100, 101, 102]);
    assert_eq!(packed.read_secondary_values(), [200, 201, 202]);
    assert_eq!(packed.read_tertiary_values(), [50, 51, 52]);

    // Test chainable accessors
    let packed = PaletteUnpacked::default()
        .pack()
        .with_primary_values([10, 20, 30])
        .with_secondary_values([40, 50, 60])
        .with_tertiary_values([70, 80, 90]);

    assert_eq!(packed.read_primary_values(), [10, 20, 30]);
    assert_eq!(packed.read_secondary_values(), [40, 50, 60]);
    assert_eq!(packed.read_tertiary_values(), [70, 80, 90]);

    // Test writing whole RGB structs
    let mut packed = PaletteUnpacked::default().pack();
    let new_rgb = RGBUnpacked {
        values: [111, 222, 233],
    };
    packed.write_primary_unpacked(new_rgb);

    let read_rgb = packed.read_primary_unpacked();
    assert_eq!(read_rgb.values, [111, 222, 233]);
}

#[test]
fn test_custom_struct_with_enums() {
    interface_objects! {
        enum Status: u8{2} {
            0 Ok,
            1 Warning,
            2 Error,
            3 Critical,
        }

        struct StatusBlock(size = 1) {
            main: Status = Status::Ok,
            backup: Status = Status::Warning,
            _: u8{4},
        }

        struct System(size = 3) {
            status: StatusBlockUnpacked = StatusBlockUnpacked {
                main: Status::Ok,
                backup: Status::Warning
            },
            temperature: u8 = 75,
            uptime: u8 = 42,
        }
    }

    let packed = SystemUnpacked::default().pack();
    let status_packed = StatusBlockUnpacked {
        main: Status::Ok,
        backup: Status::Warning,
    }
    .pack();

    assert_eq!(packed.0[0], status_packed.0[0]);
    assert_eq!(packed.0[1], 75);
    assert_eq!(packed.0[2], 42);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.status.main, Status::Ok);
    assert_eq!(unpacked.status.backup, Status::Warning);
    assert_eq!(unpacked.temperature, 75);
    assert_eq!(unpacked.uptime, 42);

    // Test nested read accessors
    assert_eq!(packed.read_status_main(), Status::Ok);
    assert_eq!(packed.read_status_backup(), Status::Warning);
    assert_eq!(packed.read_temperature(), 75);
    assert_eq!(packed.read_uptime(), 42);

    let status_read = packed.read_status_unpacked();
    assert_eq!(status_read.main, Status::Ok);
    assert_eq!(status_read.backup, Status::Warning);

    // Test write accessors
    let mut packed = SystemUnpacked::default().pack();
    packed.write_status_main(Status::Critical);
    packed.write_status_backup(Status::Error);
    packed.write_temperature(85);
    packed.write_uptime(100);

    assert_eq!(packed.read_status_main(), Status::Critical);
    assert_eq!(packed.read_status_backup(), Status::Error);
    assert_eq!(packed.read_temperature(), 85);
    assert_eq!(packed.read_uptime(), 100);

    // Test chainable accessors
    let packed = SystemUnpacked::default()
        .pack()
        .with_status_main(Status::Warning)
        .with_status_backup(Status::Ok)
        .with_temperature(65)
        .with_uptime(200);

    assert_eq!(packed.read_status_main(), Status::Warning);
    assert_eq!(packed.read_status_backup(), Status::Ok);
    assert_eq!(packed.read_temperature(), 65);
    assert_eq!(packed.read_uptime(), 200);

    // Test writing the whole status block
    let mut packed = SystemUnpacked::default().pack();
    let new_status = StatusBlockUnpacked {
        main: Status::Error,
        backup: Status::Critical,
    };
    packed.write_status_unpacked(new_status);

    let read_status = packed.read_status_unpacked();
    assert_eq!(read_status.main, Status::Error);
    assert_eq!(read_status.backup, Status::Critical);
}

#[test]
fn test_custom_struct_mixed_with_bit_fields() {
    interface_objects! {
        struct Flags(size = 1) {
            enabled: bool = true,
            debug: bool = false,
            verbose: bool = true,
            _: u8{5},
        }

        struct Config(size = 3) {
            version: u8{4} = 0b1111,
            flags: FlagsUnpacked = FlagsUnpacked { enabled: true, debug: false, verbose: true },
            timeout: u16{10} = 0x3FF,
            reserved: u8{2} = 0b11,
        }
    }

    let packed = ConfigUnpacked::default().pack();
    // Verify bit layout is correct
    let unpacked = packed.unpack();
    assert_eq!(unpacked.version, 15);
    assert_eq!(unpacked.flags.enabled, true);
    assert_eq!(unpacked.flags.debug, false);
    assert_eq!(unpacked.flags.verbose, true);
    assert_eq!(unpacked.timeout, 0x3FF);
    assert_eq!(unpacked.reserved, 0b11);

    // Test read accessors
    assert_eq!(packed.read_version(), 15);
    assert_eq!(packed.read_flags_enabled(), true);
    assert_eq!(packed.read_flags_debug(), false);
    assert_eq!(packed.read_flags_verbose(), true);
    assert_eq!(packed.read_timeout(), 0x3FF);
    assert_eq!(packed.read_reserved(), 0b11);

    let flags_read = packed.read_flags_unpacked();
    assert_eq!(flags_read.enabled, true);
    assert_eq!(flags_read.debug, false);
    assert_eq!(flags_read.verbose, true);

    // Test write accessors
    let mut packed = ConfigUnpacked::default().pack();
    packed.write_version(8);
    packed.write_flags_enabled(false);
    packed.write_flags_debug(true);
    packed.write_flags_verbose(false);
    packed.write_timeout(0x200);
    packed.write_reserved(0b01);

    assert_eq!(packed.read_version(), 8);
    assert_eq!(packed.read_flags_enabled(), false);
    assert_eq!(packed.read_flags_debug(), true);
    assert_eq!(packed.read_flags_verbose(), false);
    assert_eq!(packed.read_timeout(), 0x200);
    assert_eq!(packed.read_reserved(), 0b01);

    // Test chainable accessors
    let packed = ConfigUnpacked::default()
        .pack()
        .with_version(5)
        .with_flags_enabled(true)
        .with_flags_debug(false)
        .with_flags_verbose(true)
        .with_timeout(0x100)
        .with_reserved(0b10);

    assert_eq!(packed.read_version(), 5);
    assert_eq!(packed.read_flags_enabled(), true);
    assert_eq!(packed.read_flags_debug(), false);
    assert_eq!(packed.read_flags_verbose(), true);
    assert_eq!(packed.read_timeout(), 0x100);
    assert_eq!(packed.read_reserved(), 0b10);

    // Test writing the whole flags struct
    let mut packed = ConfigUnpacked::default().pack();
    let new_flags = FlagsUnpacked {
        enabled: false,
        debug: true,
        verbose: false,
    };
    packed.write_flags_unpacked(new_flags);

    let read_flags = packed.read_flags_unpacked();
    assert_eq!(read_flags.enabled, false);
    assert_eq!(read_flags.debug, true);
    assert_eq!(read_flags.verbose, false);
}

#[test]
fn test_custom_struct_roundtrip() {
    interface_objects! {
        struct Coordinate(size = 4) {
            x: i16 = -100,
            y: i16 = 200,
        }

        struct Entity(size = 12) {
            id: u32 = 0x12345678,
            position: CoordinateUnpacked = CoordinateUnpacked { x: -100, y: 200 },
            velocity: CoordinateUnpacked = CoordinateUnpacked { x: 50, y: -75 },
        }
    }

    // Test roundtrip with default values
    let original = EntityUnpacked::default();
    let packed = original.pack();
    let unpacked = packed.unpack();
    assert_eq!(original, unpacked);

    // Test accessor roundtrip
    assert_eq!(packed.read_id(), 0x12345678);
    assert_eq!(packed.read_position_x(), -100);
    assert_eq!(packed.read_position_y(), 200);
    assert_eq!(packed.read_velocity_x(), 50);
    assert_eq!(packed.read_velocity_y(), -75);

    let pos_read = packed.read_position_unpacked();
    assert_eq!(pos_read.x, -100);
    assert_eq!(pos_read.y, 200);

    let vel_read = packed.read_velocity_unpacked();
    assert_eq!(vel_read.x, 50);
    assert_eq!(vel_read.y, -75);

    // Test roundtrip with custom values using write accessors
    let mut packed = EntityUnpacked::default().pack();
    packed.write_id(0xDEADBEEF);
    packed.write_position_x(1000);
    packed.write_position_y(-2000);
    packed.write_velocity_x(-500);
    packed.write_velocity_y(750);

    assert_eq!(packed.read_id(), 0xDEADBEEF);
    assert_eq!(packed.read_position_x(), 1000);
    assert_eq!(packed.read_position_y(), -2000);
    assert_eq!(packed.read_velocity_x(), -500);
    assert_eq!(packed.read_velocity_y(), 750);

    // Test chainable accessors
    let packed = EntityUnpacked::default()
        .pack()
        .with_id(0xCAFEBABE)
        .with_position_x(2000)
        .with_position_y(-1000)
        .with_velocity_x(100)
        .with_velocity_y(-200);

    assert_eq!(packed.read_id(), 0xCAFEBABE);
    assert_eq!(packed.read_position_x(), 2000);
    assert_eq!(packed.read_position_y(), -1000);
    assert_eq!(packed.read_velocity_x(), 100);
    assert_eq!(packed.read_velocity_y(), -200);

    // Test writing whole coordinate structs
    let mut packed = EntityUnpacked::default().pack();
    let new_pos = CoordinateUnpacked { x: 3000, y: 4000 };
    let new_vel = CoordinateUnpacked { x: -300, y: 400 };

    packed.write_position_unpacked(new_pos);
    packed.write_velocity_unpacked(new_vel);

    assert_eq!(packed.read_position_unpacked(), new_pos);
    assert_eq!(packed.read_velocity_unpacked(), new_vel);

    // Verify the packed data makes sense
    let expected_id_bytes = 0xDEADBEEFu32.to_be_bytes();
    let mut test_packed = EntityUnpacked::default().pack();
    test_packed.write_id(0xDEADBEEF);
    test_packed.write_position_x(1000);
    test_packed.write_position_y(-2000);
    test_packed.write_velocity_x(-500);
    test_packed.write_velocity_y(750);

    assert_eq!(&test_packed.0[0..4], &expected_id_bytes);

    let pos_packed = CoordinateUnpacked { x: 1000, y: -2000 }.pack();
    assert_eq!(&test_packed.0[4..8], &pos_packed.0);

    let vel_packed = CoordinateUnpacked { x: -500, y: 750 }.pack();
    assert_eq!(&test_packed.0[8..12], &vel_packed.0);
}

#[test]
fn test_complex_custom_struct_composition() {
    interface_objects! {
        struct Header(size = 4) {
            magic: u32 = 0xDEADBEEF,
        }

        struct Metadata(size = 8) {
            timestamp: u64 = 0x123456789ABCDEF0,
        }

        struct Payload(size = 16) {
            data: [u8; 16] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10],
        }

        struct Message(size = 28) {
            header: HeaderUnpacked = HeaderUnpacked { magic: 0xDEADBEEF },
            metadata: MetadataUnpacked = MetadataUnpacked { timestamp: 0x123456789ABCDEF0 },
            payload: PayloadUnpacked = PayloadUnpacked {
                data: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
            },
        }
    }

    let packed = MessageUnpacked::default().pack();

    // Verify header
    assert_eq!(&packed.0[0..4], &0xDEADBEEFu32.to_be_bytes());

    // Verify metadata
    assert_eq!(&packed.0[4..12], &0x123456789ABCDEF0u64.to_be_bytes());

    // Verify payload
    let expected_payload: [u8; 16] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    assert_eq!(&packed.0[12..28], &expected_payload);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.header.magic, 0xDEADBEEF);
    assert_eq!(unpacked.metadata.timestamp, 0x123456789ABCDEF0);
    assert_eq!(unpacked.payload.data, expected_payload);

    // Test nested read accessors
    assert_eq!(packed.read_header_magic(), 0xDEADBEEF);
    assert_eq!(packed.read_metadata_timestamp(), 0x123456789ABCDEF0);
    assert_eq!(packed.read_payload_data(), expected_payload);

    let header_read = packed.read_header_unpacked();
    assert_eq!(header_read.magic, 0xDEADBEEF);

    let metadata_read = packed.read_metadata_unpacked();
    assert_eq!(metadata_read.timestamp, 0x123456789ABCDEF0);

    let payload_read = packed.read_payload_unpacked();
    assert_eq!(payload_read.data, expected_payload);

    // Test write accessors
    let mut packed = MessageUnpacked::default().pack();
    packed.write_header_magic(0xCAFEBABE);
    packed.write_metadata_timestamp(0xFEDCBA9876543210);
    let new_data = [16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1];
    packed.write_payload_data(new_data);

    assert_eq!(packed.read_header_magic(), 0xCAFEBABE);
    assert_eq!(packed.read_metadata_timestamp(), 0xFEDCBA9876543210);
    assert_eq!(packed.read_payload_data(), new_data);

    // Test chainable accessors
    let packed = MessageUnpacked::default()
        .pack()
        .with_header_magic(0x12345678)
        .with_metadata_timestamp(0xABCDEF0123456789)
        .with_payload_data([
            100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
        ]);

    assert_eq!(packed.read_header_magic(), 0x12345678);
    assert_eq!(packed.read_metadata_timestamp(), 0xABCDEF0123456789);
    assert_eq!(
        packed.read_payload_data(),
        [
            100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115
        ]
    );

    // Test writing whole structs
    let mut packed = MessageUnpacked::default().pack();
    let new_header = HeaderUnpacked { magic: 0x11111111 };
    let new_metadata = MetadataUnpacked {
        timestamp: 0x2222222222222222,
    };
    let new_payload = PayloadUnpacked { data: [33; 16] };

    packed.write_header_unpacked(new_header);
    packed.write_metadata_unpacked(new_metadata);
    packed.write_payload_unpacked(new_payload);

    assert_eq!(packed.read_header_unpacked(), new_header);
    assert_eq!(packed.read_metadata_unpacked(), new_metadata);
    assert_eq!(packed.read_payload_unpacked(), new_payload);
}

#[test]
fn test_custom_struct_with_bit_swivelling() {
    interface_objects! {
        struct Foo(size = 2) {
            a: u8{3},
            b: u8{5},
            c: u8{4},
            d: u8{4},
        }

        struct Container(size = 4) {
            packed1: FooUnpacked[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30] = FooUnpacked {
                a: 5, b: 26, c: 12, d: 3
            },
            packed2: FooUnpacked[1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31] = FooUnpacked {
                a: 2, b: 0, c: 0, d: 2
            },
        }
    }

    let packed = ContainerUnpacked::default().pack();
    assert_eq!(packed.0, [0b10011010, 0b10001000, 0b10100000, 0b00001110]);

    let unpacked = packed.unpack();
    assert_eq!(unpacked.packed1.a, 5);
    assert_eq!(unpacked.packed1.b, 26);
    assert_eq!(unpacked.packed1.c, 12);
    assert_eq!(unpacked.packed1.d, 3);
    assert_eq!(unpacked.packed2.a, 2);
    assert_eq!(unpacked.packed2.b, 0);
    assert_eq!(unpacked.packed2.c, 0);
    assert_eq!(unpacked.packed2.d, 2);

    // Test nested read accessors with bit swivelling
    assert_eq!(packed.read_packed1_a(), 5);
    assert_eq!(packed.read_packed1_b(), 26);
    assert_eq!(packed.read_packed1_c(), 12);
    assert_eq!(packed.read_packed1_d(), 3);
    assert_eq!(packed.read_packed2_a(), 2);
    assert_eq!(packed.read_packed2_b(), 0);
    assert_eq!(packed.read_packed2_c(), 0);
    assert_eq!(packed.read_packed2_d(), 2);

    let packed1_read = packed.read_packed1_unpacked();
    assert_eq!(packed1_read.a, 5);
    assert_eq!(packed1_read.b, 26);
    assert_eq!(packed1_read.c, 12);
    assert_eq!(packed1_read.d, 3);

    let packed2_read = packed.read_packed2_unpacked();
    assert_eq!(packed2_read.a, 2);
    assert_eq!(packed2_read.b, 0);
    assert_eq!(packed2_read.c, 0);
    assert_eq!(packed2_read.d, 2);

    // Test write accessors with bit swivelling
    let mut packed = ContainerUnpacked::default().pack();
    packed.write_packed1_a(7);
    packed.write_packed1_b(15);
    packed.write_packed1_c(8);
    packed.write_packed1_d(1);
    packed.write_packed2_a(1);
    packed.write_packed2_b(31);
    packed.write_packed2_c(15);
    packed.write_packed2_d(0);

    assert_eq!(packed.read_packed1_a(), 7);
    assert_eq!(packed.read_packed1_b(), 15);
    assert_eq!(packed.read_packed1_c(), 8);
    assert_eq!(packed.read_packed1_d(), 1);
    assert_eq!(packed.read_packed2_a(), 1);
    assert_eq!(packed.read_packed2_b(), 31);
    assert_eq!(packed.read_packed2_c(), 15);
    assert_eq!(packed.read_packed2_d(), 0);

    // Test chainable accessors
    let packed = ContainerUnpacked::default()
        .pack()
        .with_packed1_a(4)
        .with_packed1_b(20)
        .with_packed1_c(6)
        .with_packed1_d(5)
        .with_packed2_a(3)
        .with_packed2_b(10)
        .with_packed2_c(2)
        .with_packed2_d(1);

    assert_eq!(packed.read_packed1_a(), 4);
    assert_eq!(packed.read_packed1_b(), 20);
    assert_eq!(packed.read_packed1_c(), 6);
    assert_eq!(packed.read_packed1_d(), 5);
    assert_eq!(packed.read_packed2_a(), 3);
    assert_eq!(packed.read_packed2_b(), 10);
    assert_eq!(packed.read_packed2_c(), 2);
    assert_eq!(packed.read_packed2_d(), 1);

    // Test writing whole structs with bit swivelling
    let mut packed = ContainerUnpacked::default().pack();
    let new_foo1 = FooUnpacked { a: 1, b: 2, c: 3, d: 4 };
    let new_foo2 = FooUnpacked { a: 5, b: 6, c: 7, d: 0 };

    packed.write_packed1_unpacked(new_foo1);
    packed.write_packed2_unpacked(new_foo2);

    assert_eq!(packed.read_packed1_unpacked(), new_foo1);
    assert_eq!(packed.read_packed2_unpacked(), new_foo2);
}

#[test]
fn test_custom_struct_with_units() {
    use uom::si::f64::{Pressure, ThermodynamicTemperature};
    use uom::si::pressure::pascal;
    use uom::si::thermodynamic_temperature::degree_celsius;

    interface_objects! {
        struct Units(size = 4) {
            raw_temperature: u16 = 75 {
                quantity: ThermodynamicTemperature,
                unit: degree_celsius,
                lsb: 1f64 / 128f64,
            },
            raw_pressure: u16 = 12 {
                quantity: Pressure,
                unit: pascal,
                from_raw: |x| (x as f64 + 7.0) * 31.0,
                into_raw: |x| (x / 31.0 - 7.0) as u16,
            },
        }
    }

    let packed = UnitsUnpacked::default().pack();
    assert_eq!(packed.0, [0, 75, 0, 12]);
    assert_eq!(packed.read_raw_temperature(), 75);
    assert_eq!(packed.read_temperature().get::<degree_celsius>(), 75.0 / 128.0);
    assert_eq!(packed.read_raw_pressure(), 12);
    assert_eq!(packed.read_pressure().get::<pascal>(), 589.0);

    // Test write accessors for raw values
    let mut packed = UnitsUnpacked::default().pack();
    packed.write_raw_temperature(100);
    packed.write_raw_pressure(20);

    assert_eq!(packed.read_raw_temperature(), 100);
    assert_eq!(packed.read_raw_pressure(), 20);
    assert_eq!(packed.read_temperature().get::<degree_celsius>(), 100.0 / 128.0);
    assert_eq!(packed.read_pressure().get::<pascal>(), 837.0);

    // Test write accessors for unit values
    let temp = ThermodynamicTemperature::new::<degree_celsius>(25.0);
    let pressure = Pressure::new::<pascal>(1000.0);

    packed.write_temperature(temp);
    packed.write_pressure(pressure);

    // Check that the raw values are correctly converted
    let expected_temp_raw = (25.0 * 128.0) as u16;
    let expected_pressure_raw = (1000.0 / 31.0 - 7.0) as u16;

    assert_eq!(packed.read_raw_temperature(), expected_temp_raw);
    assert_eq!(packed.read_raw_pressure(), expected_pressure_raw);

    // Check that reading back gives approximately the same values
    let read_temp = packed.read_temperature();
    let read_pressure = packed.read_pressure();

    assert!((read_temp.get::<degree_celsius>() - 25.0).abs() < 0.1);
    assert!((read_pressure.get::<pascal>() - 1000.0).abs() < 50.0);

    // Test chainable accessors
    let temp2 = ThermodynamicTemperature::new::<degree_celsius>(30.0);
    let pressure2 = Pressure::new::<pascal>(2000.0);

    let packed = UnitsUnpacked::default()
        .pack()
        .with_raw_temperature(64)
        .with_raw_pressure(50)
        .with_temperature(temp2)
        .with_pressure(pressure2);

    // The unit values should override the raw values
    let final_temp = packed.read_temperature();
    let final_pressure = packed.read_pressure();

    assert!((final_temp.get::<degree_celsius>() - 30.0).abs() < 0.1);
    assert!((final_pressure.get::<pascal>() - 2000.0).abs() < 100.0);
}

#[test]
fn test_accessor_write_and_read_consistency() {
    interface_objects! {
        enum Mode: u8{2} {
            0 Off,
            1 Low,
            2 High,
            3 Max,
        }

        struct Inner(size = 2) {
            value: u16 = 0x1234,
        }

        struct Complex(size = 7) {
            flag: bool = true,
            mode: Mode = Mode::High,
            count: u8{6} = 42,
            inner: InnerUnpacked = InnerUnpacked { value: 0x1234 },
            data: [u8; 3] = [0xAA, 0xBB, 0xCC],
            _: u8{7}
        }
    }

    // Test that writing and reading back gives consistent results
    let mut packed = ComplexUnpacked::default().pack();

    // Test all field types
    packed.write_flag(false);
    packed.write_mode(Mode::Max);
    packed.write_count(63);
    packed.write_inner_value(0x5678);
    packed.write_data([0x11, 0x22, 0x33]);

    assert_eq!(packed.read_flag(), false);
    assert_eq!(packed.read_mode(), Mode::Max);
    assert_eq!(packed.read_count(), 63);
    assert_eq!(packed.read_inner_value(), 0x5678);
    assert_eq!(packed.read_data(), [0x11, 0x22, 0x33]);

    let inner_read = packed.read_inner_unpacked();
    assert_eq!(inner_read.value, 0x5678);

    // Test chainable consistency
    let chained = ComplexUnpacked::default()
        .pack()
        .with_flag(true)
        .with_mode(Mode::Low)
        .with_count(21)
        .with_inner_value(0x9ABC)
        .with_data([0x44, 0x55, 0x66]);

    assert_eq!(chained.read_flag(), true);
    assert_eq!(chained.read_mode(), Mode::Low);
    assert_eq!(chained.read_count(), 21);
    assert_eq!(chained.read_inner_value(), 0x9ABC);
    assert_eq!(chained.read_data(), [0x44, 0x55, 0x66]);

    // Test that packed and unpacked representations are consistent
    let original = ComplexUnpacked {
        flag: false,
        mode: Mode::Off,
        count: 10,
        inner: InnerUnpacked { value: 0xDEAD },
        data: [0x99, 0x88, 0x77],
    };

    let packed_from_struct = original.pack();
    let chained_equivalent = ComplexUnpacked::default()
        .pack()
        .with_flag(false)
        .with_mode(Mode::Off)
        .with_count(10)
        .with_inner_value(0xDEAD)
        .with_data([0x99, 0x88, 0x77]);

    // Both should produce the same packed representation
    assert_eq!(packed_from_struct.0, chained_equivalent.0);

    // And both should read back the same values
    assert_eq!(packed_from_struct.read_flag(), chained_equivalent.read_flag());
    assert_eq!(packed_from_struct.read_mode(), chained_equivalent.read_mode());
    assert_eq!(packed_from_struct.read_count(), chained_equivalent.read_count());
    assert_eq!(
        packed_from_struct.read_inner_value(),
        chained_equivalent.read_inner_value()
    );
    assert_eq!(packed_from_struct.read_data(), chained_equivalent.read_data());
}

#[test]
fn test_nested_struct_accessor_promotion() {
    interface_objects! {
        struct Level3(size = 1) {
            deep_value: u8 = 42,
        }

        struct Level2(size = 2) {
            mid_value: u8 = 21,
            level3: Level3Unpacked = Level3Unpacked { deep_value: 42 },
        }

        struct Level1(size = 4) {
            top_value: u16 = 0x1234,
            level2: Level2Unpacked = Level2Unpacked {
                mid_value: 21,
                level3: Level3Unpacked { deep_value: 42 }
            },
        }
    }

    let packed = Level1Unpacked::default().pack();

    // Test that deeply nested accessors are promoted to the top level
    assert_eq!(packed.read_top_value(), 0x1234);
    assert_eq!(packed.read_level2_mid_value(), 21);
    assert_eq!(packed.read_level2_level3_deep_value(), 42);

    // Test intermediate level accessors
    let level2_read = packed.read_level2_unpacked();
    assert_eq!(level2_read.mid_value, 21);
    assert_eq!(level2_read.level3.deep_value, 42);

    let level3_read = packed.read_level2_level3_unpacked();
    assert_eq!(level3_read.deep_value, 42);

    // Test write accessors at all levels
    let mut packed = Level1Unpacked::default().pack();
    packed.write_top_value(0x5678);
    packed.write_level2_mid_value(84);
    packed.write_level2_level3_deep_value(168);

    assert_eq!(packed.read_top_value(), 0x5678);
    assert_eq!(packed.read_level2_mid_value(), 84);
    assert_eq!(packed.read_level2_level3_deep_value(), 168);

    // Test chainable accessors
    let packed = Level1Unpacked::default()
        .pack()
        .with_top_value(0x9ABC)
        .with_level2_mid_value(63)
        .with_level2_level3_deep_value(126);

    assert_eq!(packed.read_top_value(), 0x9ABC);
    assert_eq!(packed.read_level2_mid_value(), 63);
    assert_eq!(packed.read_level2_level3_deep_value(), 126);

    // Test writing intermediate structs
    let mut packed = Level1Unpacked::default().pack();
    let new_level3 = Level3Unpacked { deep_value: 200 };
    let new_level2 = Level2Unpacked {
        mid_value: 100,
        level3: new_level3,
    };

    packed.write_level2_unpacked(new_level2);

    assert_eq!(packed.read_level2_mid_value(), 100);
    assert_eq!(packed.read_level2_level3_deep_value(), 200);

    // Test that the change is reflected at all levels
    let read_level2 = packed.read_level2_unpacked();
    assert_eq!(read_level2.mid_value, 100);
    assert_eq!(read_level2.level3.deep_value, 200);
}
