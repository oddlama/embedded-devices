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

// #[test]
// fn test_unsigned_shifted() {
//     registers! {
//         defaults {
//             i2c_codec = DummyI2cCodec,
//             spi_codec = DummySpiCodec,
//             codec_error = (),
//         }
//
//         Struct(addr = 0x0, mode = rw, size = 2) {
//             _: bool,
//             f1: u8,
//             f2: u8 = 0x49,
//         }
//     }
//
//     let packed = StructUnpacked::default().pack();
//     assert_eq!(packed.0, [0u8, 0x49]);
// }
