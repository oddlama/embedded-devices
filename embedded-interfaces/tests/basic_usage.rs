use embedded_interfaces::{codegen::interface_objects, packable::UnsignedPackable};

// FIXME: allow _: _[2..4] short reserved syntax
// FIXME: allow _: _{4} short reserved syntax
// FIXME: allow x: u8[7..=0] reverse ranges
// FIXME: allow read write annotatation per field? then dont generate write_f1 read_f1 with_f1 ?? not sure
// TODO: remove Enum{3} necessity since we know the bit size anyway in the same macro at least,
// same for custom structs

type DummyI2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
type DummySpiCodec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec<()>;

/// Measurement conversion time
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum ConversionTime {
    /// 50µs
    T_50 = 0,
    /// 84μs
    T_84 = 1,
    /// 150μs
    T_150 = 2,
    /// 280μs
    T_280 = 3,
    /// 540μs
    T_540 = 4,
    /// 1052μs
    T_1052 = 5,
    /// 2074μs
    T_2074 = 6,
    /// 4120μs
    T_4120 = 7,
}

/// Conversion averaging counts
#[derive(Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum AverageCount {
    /// Single sample without averaging.
    X_1 = 0,
    /// 4x averaging.
    X_4 = 1,
    /// 16x averaging.
    X_16 = 2,
    /// 64x averaging.
    X_64 = 3,
    /// 128x averaging.
    X_128 = 4,
    /// 256x averaging.
    X_256 = 5,
    /// 512x averaging.
    X_512 = 6,
    /// 1024x averaging.
    X_1024 = 7,
}

impl AverageCount {
    /// Returns the averaging factor
    pub fn factor(&self) -> u16 {
        match self {
            AverageCount::X_1 => 1,
            AverageCount::X_4 => 4,
            AverageCount::X_16 => 16,
            AverageCount::X_64 => 64,
            AverageCount::X_128 => 128,
            AverageCount::X_256 => 256,
            AverageCount::X_512 => 512,
            AverageCount::X_1024 => 1024,
        }
    }
}

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = DummyI2cCodec,
        spi_codec = DummySpiCodec,
    }

    devices [
        INA228,
    ]

    /// Operating mode.
    enum OperatingMode: u8{3} {
        0b000 PowerDown,
        0b101 Continuous,
    }

    // /// Multi-byte register with single field
    // register Temperature(addr = 0x6, mode = r, size = 2) {
    //     /// Internal die temperature. Resolution 7.8125 m°C/LSB
    //     raw_value: i16 = 0 {
    //         quantity: ThermodynamicTemperature,
    //         unit: degree_celsius,
    //         scale: 1 / 128,
    //     },
    // }
    //
    // register Range(addr = 0x0, mode = rw, size = 2) {
    //     // Single bit
    //     f0: u8[0],
    //     // Exclusive range
    //     f1: u8[1..4],
    //     // Inclusive range
    //     f2: u8[4..=7],
    //     // Bit pattern mixed with ranges
    //     f3: u8[8, 9, 10, 11..=14, 15],
    // }
    //
    // register Reserved(addr = 0x0, mode = rw, size = 4) {
    //     // Reserved, no default implies Default::default()
    //     _: u8,
    //     // Reserved again, with default
    //     _: u8 = 0,
    //     // Named reserved field 0
    //     _reserved0: u8,
    //     // Named reserved field 1
    //     _reserved1: u8,
    // }

    /// Register with mixed field types
    register AdcConfiguration(addr = 0x1, mode = rw, size = 2) {
        /// Operating mode
        operating_mode: OperatingMode[0..3] = OperatingMode::Continuous,
        _: u16{13}
        // /// Enable temperature conversion
        // enable_temperature: bool[1] = true,
        // /// Enable shunt voltage conversion
        // enable_shunt: bool[2] = true,
        // /// Enable bus voltage conversion
        // enable_bus: bool[3] = true,
        // /// Bus conversion time
        // bus_conversion_time: ConversionTime[4..7] = ConversionTime::T_1052,
        // /// Shunt conversion time
        // shunt_conversion_time: ConversionTime[7..10] = ConversionTime::T_1052,
        // /// Temperature conversion time
        // temperature_conversion_time: ConversionTime[10..13] = ConversionTime::T_1052,
        // /// Average count
        // average_count: AverageCount[13..16] = AverageCount::X_1,
    }

    // /// Large register with 40-bit field
    // register Energy(addr = 0x9, mode = r, size = 5) {
    //     /// 40-bit energy accumulation
    //     raw_value: u64{40} = 0,
    // }
}
