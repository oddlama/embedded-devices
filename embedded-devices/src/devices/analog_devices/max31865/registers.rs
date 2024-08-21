use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::num_rational::Rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::thermodynamic_temperature::degree_celsius;

/// Conversion mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum ConversionMode {
    /// No automatic conversions, 1-shot conversions may be initiated from this mode.
    #[default]
    NormallyOff = 0b0,
    /// Automatic conversion mode, conversions occur continuously at a 50/60Hz rate.
    Automatic = 0b1,
}

/// Wiring mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum WiringMode {
    /// 2- or 4-wire RTD.
    #[default]
    TwoOrFourWire = 0b0,
    /// 3-wire RTD connected to `FORCE+`, `RTDIN+`, `RTDIN-`.
    ThreeWire = 0b1,
}

/// Fault detection cycle.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum FaultDetectionCycle {
    /// Fault detection has finished.
    #[default]
    Finished = 0b00,
    /// Run fault detection with automatic delay.
    Automatic = 0b01,
    /// Run fault detection with manual delay (cycle 1).
    RunManual = 0b10,
    /// Finish fault detection with manual delay (cycle 2).
    FinishManual = 0b11,
}

/// Filter mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum FilterMode {
    /// 60Hz filter
    #[default]
    F_60Hz = 0b0,
    /// 50Hz filter
    F_50Hz = 0b1,
}

/// The device configuration register.
#[device_register(super::MAX31865)]
#[register(address = 0b0000, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Configuration {
    /// When no conversions are being performed, VBIAS may be disabled to reduce power
    /// dissipation. Set this bit to `true` to enable VBIAS before beginning a single (1-Shot)
    /// conversion. When automatic (continuous) conversion mode is selected,
    /// VBIAS remains on continuously.
    pub enable_bias_voltage: bool,
    /// Conversion mode (operating mode).
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub conversion_mode: ConversionMode,
    /// When the conversion mode is set to [`ConversionMode::NormallyOff`],
    /// set this flag to start a conversion. You must enable VBIAS before
    /// starting a conversion. Note any filter capacitors at the RTDIN inputs
    /// need to charge before an accurate conversion can be performed.
    /// Therefore, enable VBIAS and wait at least 10.5 time constants of the input RC
    /// network plus an additional 1ms before initiating the conversion. Note that a single
    /// conversion requires approximately 52ms in 60Hz filter mode or 62.5ms in 50Hz
    /// filter mode to complete. This flag will auto-clear after the conversion.
    pub oneshot: bool,
    /// The wiring mode of the RTD.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
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
    pub fault_detection_cycle: FaultDetectionCycle,
    /// Set this flag while `oneshot` is off and writing [`FaultDetectionCycle::Finished`] to
    /// `fault_detection_cycle` to clear the fault status register. This bit will auto-clear.
    pub clear_fault_status: bool,
    /// The wiring mode of the RTD.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub filter_mode: FilterMode,
}

/// The RTD resistance data.
#[device_register(super::MAX31865)]
#[register(address = 0b0001, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Resistance {
    /// The ratio of RTD resistance to reference resistance.
    /// The decimal value is obtained by dividing this value by `2^15`.
    #[bondrewd(bit_length = 15)]
    pub resistance_ratio: u16,
    /// Whether a fault was detected.
    pub fault: bool,
}

macro_rules! define_fault_threshold_register {
    ($name:ident, $address:expr, $doc:expr) => {
        #[doc = $doc]
        #[device_register(super::MAX31865)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            /// The ratio of RTD resistance to reference resistance.
            /// The decimal value is obtained by dividing this value by `2^15`.
            #[bondrewd(bit_length = 15)]
            pub resistance_ratio: u16,
            #[bondrewd(bit_length = 1, reserve)]
            #[allow(dead_code)]
            pub reserved: u8,
        }

        impl $name {
            // /// Reads the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            // pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
            //     ThermodynamicTemperature::new::<degree_celsius>(
            //         Rational32::new_raw(self.read_raw_temperature_limit().into(), 128).reduced(),
            //     )
            // }
            //
            // /// Writes the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            // /// The passed temperature will be truncated (rounded down).
            // pub fn write_temperature_limit(
            //     &mut self,
            //     temperature_limit: ThermodynamicTemperature,
            // ) -> Result<(), core::num::TryFromIntError> {
            //     let temp = temperature_limit.get::<degree_celsius>();
            //     let temp: i16 = (temp * Rational32::from_integer(128)).to_integer().try_into()?;
            //     self.write_raw_temperature_limit(temp);
            //     Ok(())
            // }
            //
            // /// Writes the temperature limit in °C with a resolution of 7.8125m°C/LSB.
            // /// The passed temperature will be truncated (rounded down).
            // pub fn with_temperature_limit(
            //     mut self,
            //     temperature_limit: ThermodynamicTemperature,
            // ) -> Result<Self, core::num::TryFromIntError> {
            //     self.write_temperature_limit(temperature_limit)?;
            //     Ok(self)
            // }
        }
    };
}

define_fault_threshold_register!(
    FaultThresholdHigh,
    0b0011,
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
#[register(address = 0b0111, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FaultStatus {
    /// Measured resistance greater than High Fault Threshold value.
    pub high_threshold: bool,
    /// Measured resistance less than Low Fault Threshold value
    pub low_threshold: bool,
    /// `V_REFIN-` > 0.85 * `V_BIAS`.
    /// Detected on demand by initiating fault detection.
    pub v_refin_minus_exceeds_85_percent_of_v_bias: bool,
    /// `V_REFIN-` < 0.85 * `V_BIAS`, FORCE- open.
    /// Detected on demand by initiating fault detection.
    pub v_refin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool,
    /// `V_RTDIN-` < 0.85 * `V_BIAS`, FORCE- open.
    /// Detected on demand by initiating fault detection.
    pub v_rtdin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool,
    /// Over/undervoltage, some protected input voltage exceeds VDD or below GND1
    pub over_or_undervoltage: bool,
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}