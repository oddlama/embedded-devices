use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::{register, spi::codecs::standard_codec::StandardCodec};

pub type MAX31865SpiCodec = StandardCodec<1, 6, 0, 7, false, 0>;

/// Conversion mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ConversionMode {
    /// No automatic conversions, 1-shot conversions may be initiated from this mode.
    NormallyOff = 0b0,
    /// Automatic conversion mode, conversions occur continuously at a 50/60Hz rate.
    Automatic = 0b1,
}

/// Wiring mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum WiringMode {
    /// 2- or 4-wire RTD.
    TwoOrFourWire = 0b0,
    /// 3-wire RTD connected to `FORCE+`, `RTDIN+`, `RTDIN-`.
    ThreeWire = 0b1,
}

/// Fault detection cycle.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum FaultDetectionCycle {
    /// Fault detection has finished.
    Finished = 0b00,
    /// Run fault detection with automatic delay.
    Automatic = 0b01,
    /// Run fault detection with manual delay (cycle 1).
    RunManual = 0b10,
    /// Finish fault detection with manual delay (cycle 2).
    FinishManual = 0b11,
}

/// Filter mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum FilterMode {
    /// 60Hz filter
    F_60Hz = 0b0,
    /// 50Hz filter
    F_50Hz = 0b1,
}

/// The device configuration register.
#[device_register(super::MAX31865)]
#[register(address = 0b0000, mode = "rw", spi_codec = "MAX31865SpiCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Configuration {
    /// When no conversions are being performed, VBIAS may be disabled to reduce power
    /// dissipation. Set this bit to `true` to enable VBIAS before beginning a single (1-Shot)
    /// conversion. When automatic (continuous) conversion mode is selected,
    /// VBIAS remains on continuously.
    #[register(default = false)]
    pub enable_bias_voltage: bool,
    /// Conversion mode (operating mode).
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = ConversionMode::NormallyOff)]
    pub conversion_mode: ConversionMode,
    /// When the conversion mode is set to [`ConversionMode::NormallyOff`],
    /// set this flag to start a conversion. You must enable VBIAS before
    /// starting a conversion. Note any filter capacitors at the RTDIN inputs
    /// need to charge before an accurate conversion can be performed.
    /// Therefore, enable VBIAS and wait at least 10.5 time constants of the input RC
    /// network plus an additional 1ms before initiating the conversion. Note that a single
    /// conversion requires approximately 52ms in 60Hz filter mode or 62.5ms in 50Hz
    /// filter mode to complete. This flag will auto-clear after the conversion.
    #[register(default = false)]
    pub oneshot: bool,
    /// The wiring mode of the RTD.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = WiringMode::TwoOrFourWire)]
    pub wiring_mode: WiringMode,
    /// Used to initiate fault detection. This is reset to [`FaultDetectionCycle::Finished`]
    /// when fault detection is done. When writing this register to initiate fault detection,
    /// the bias voltage must be enabled, conversion mode must be [`ConversionMode::NormallyOff`]
    /// and the `clear_fault_status` flag must be off.
    ///
    /// If the external RTD interface circuitry includes an input filter with a time constant
    /// greater than 100µs, the fault detection cycle timing should be controlled in the manual
    /// mode operation. For more information, refer to the datasheet.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FaultDetectionCycle::Finished)]
    pub fault_detection_cycle: FaultDetectionCycle,
    /// Set this flag while `oneshot` is off and writing [`FaultDetectionCycle::Finished`] to
    /// `fault_detection_cycle` to clear the fault status register. This bit will auto-clear.
    #[register(default = false)]
    pub clear_fault_status: bool,
    /// The wiring mode of the RTD.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = FilterMode::F_60Hz)]
    pub filter_mode: FilterMode,
}

/// The RTD resistance data.
#[device_register(super::MAX31865)]
#[register(address = 0b0001, mode = "r", spi_codec = "MAX31865SpiCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Resistance {
    /// The ratio of RTD resistance to reference resistance.
    /// The decimal value is obtained by dividing this value by `2^15`.
    #[bondrewd(bit_length = 15)]
    #[register(default = 0)]
    pub resistance_ratio: u16,
    /// Whether a fault was detected.
    #[register(default = false)]
    pub fault: bool,
}

macro_rules! define_fault_threshold_register {
    ($name:ident, $address:expr, $value_default:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::MAX31865)]
        #[register(address = $address, mode = "rw", spi_codec = "MAX31865SpiCodec")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            /// The ratio of RTD resistance to reference resistance.
            /// The decimal value is obtained by dividing this value by `2^15`.
            #[bondrewd(bit_length = 15)]
            #[register(default = $default_value)]
            pub resistance_ratio: u16,
            #[bondrewd(bit_length = 1, reserve)]
            #[allow(dead_code)]
            pub reserved: u8,
        }
    };
}

define_fault_threshold_register!(
    FaultThresholdHigh,
    0b0011,
    0x7fff,
    r#"
The High Fault Threshold register selects the high trip threshold for RTD fault detection.

The RTD High bit in the Fault Status Register is set if the
RTD resistance register value is greater than or equal to
the value in the High Fault Threshold register.
The POR value of the High Fault Threshold register is FFFFh.
"#
);

define_fault_threshold_register!(
    FaultThresholdLow,
    0b0101,
    0x0000,
    r#"
The Low Fault Threshold register selects the low trip threshold for RTD fault detection.

The RTD Low bit in the Fault Status Register is set if the
RTD resistance value is less than or equal to the value in
the Low Fault Threshold register. The POR value of the
Low Fault Threshold register is 0000h.
"#
);

/// The fault status register.
///
/// Depending on whether 2-, 3- or 4-wire mode is used,
/// these status bits can have different possible causes.
/// Refer to the datasheet for a table of possible causes.
#[device_register(super::MAX31865)]
#[register(address = 0b0111, mode = "r", spi_codec = "MAX31865SpiCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FaultStatus {
    /// Measured resistance greater than High Fault Threshold value.
    #[register(default = false)]
    pub high_threshold: bool,
    /// Measured resistance less than Low Fault Threshold value
    #[register(default = false)]
    pub low_threshold: bool,
    /// `V_REFIN-` > 0.85 * `V_BIAS`.
    /// Detected on demand by initiating fault detection.
    #[register(default = false)]
    pub v_refin_minus_exceeds_85_percent_of_v_bias: bool,
    /// `V_REFIN-` < 0.85 * `V_BIAS`, FORCE- open.
    /// Detected on demand by initiating fault detection.
    #[register(default = false)]
    pub v_refin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool,
    /// `V_RTDIN-` < 0.85 * `V_BIAS`, FORCE- open.
    /// Detected on demand by initiating fault detection.
    #[register(default = false)]
    pub v_rtdin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool,
    /// Over/undervoltage, some protected input voltage exceeds VDD or below GND1
    #[register(default = false)]
    pub over_or_undervoltage: bool,
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}
