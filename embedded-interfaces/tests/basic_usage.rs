use embedded_interfaces::codegen::registers;

// FIXME: ensure error on overlap or bit reuse
// FIXME: ensure error if size mismatch
// FIXME: allow _: [2..4] short reserved syntax
// FIXME: allow x: u8[7..=0] reverse ranges
// FIXME: allow read write annotatation per field? then dont generate write_f1 read_f1 with_f1 ?? not sure

use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;
type TestSpiCodec = embedded_interfaces::registers::spi::codecs::standard_codec::StandardCodec<1, 6, 0, 7, false, 0>;

registers! {
    defaults {
        i2c_codec = OneByteRegAddrCodec,
        spi_codec = TestSpiCodec,
    }

    devices [
        INA228,
    ]

    // /// Multi-byte register with single field
    // Temperature(addr = 0x6, mode = r, size = 2) {
    //     /// Internal die temperature. Resolution 7.8125 m°C/LSB
    //     raw_value: i16 = 0 {
    //         quantity: ThermodynamicTemperature,
    //         unit: degree_celsius,
    //         scale: 1 / 128,
    //     },
    // }

    Range(addr = 0x0, mode = rw, size = 6) {
        // Single bit
        f0: u8[0],
        // Exclusive range
        f1: u8[1..4],
        // Inclusive range
        f2: u8[4..=7],
        // Bit pattern mixed with ranges
        f3: u8[8, 9, 10, 11..=14, 15],
    }

    Reserved(addr = 0x0, mode = rw, size = 4) {
        // Reserved, no default implies Default::default()
        _: u8,
        // Reserved again, with default
        _: u8 = 0,
        // Named reserved field 0
        _reserved0: u8,
        // Named reserved field 1
        _reserved1: u8,
    }

    /// Register with mixed field types
    AdcConfiguration(addr = 0x1, mode = rw, size = 2) {
        /// Operating mode
        operating_mode: OperatingMode[0..1] = OperatingMode::Continuous,
        /// Enable temperature conversion
        enable_temperature: bool[1] = true,
        /// Enable shunt voltage conversion
        enable_shunt: bool[2] = true,
        /// Enable bus voltage conversion
        enable_bus: bool[3] = true,
        /// Bus conversion time
        bus_conversion_time: ConversionTime[4..7] = ConversionTime::T_1052,
        /// Shunt conversion time
        shunt_conversion_time: ConversionTime[7..10] = ConversionTime::T_1052,
        /// Temperature conversion time
        temperature_conversion_time: ConversionTime[10..13] = ConversionTime::T_1052,
        /// Average count
        average_count: AverageCount[13..16] = AverageCount::X_1,
    }

    /// Large register with 40-bit field
    Energy(addr = 0x9, mode = r, size = 5) {
        /// 40-bit energy accumulation
        raw_value: u64[0..40] = 0,
    }
}

fn main() {
    println!("Register macro parsing successful!");
}
