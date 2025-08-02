use embedded_interfaces::codegen::interface_objects;

pub type MAX31865I2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
pub type MAX31865SpiCodec =
    embedded_interfaces::registers::spi::codecs::standard_codec::StandardCodec<1, 6, 0, 7, false, 0>;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = MAX31865I2cCodec,
        spi_codec = MAX31865SpiCodec,
    }

    register_devices [ super::MAX31865 ]

    /// Conversion mode.
    enum ConversionMode: u8{1} {
        /// No automatic conversions, 1-shot conversions may be initiated from this mode.
        0 NormallyOff,
        /// Automatic conversion mode, conversions occur continuously at a 50/60Hz rate.
        1 Automatic,
    }

    /// Wiring mode.
    enum WiringMode: u8{1} {
        /// 2- or 4-wire RTD.
        0 TwoOrFourWire,
        /// 3-wire RTD connected to `FORCE+`, `RTDIN+`, `RTDIN-`.
        1 ThreeWire,
    }

    /// Fault detection cycle.
    enum FaultDetectionCycle: u8{2} {
        /// Fault detection has finished.
        0 Finished,
        /// Run fault detection with automatic delay.
        1 Automatic,
        /// Run fault detection with manual delay (cycle 1).
        2 RunManual,
        /// Finish fault detection with manual delay (cycle 2).
        3 FinishManual,
    }

    /// Filter mode.
    #[allow(non_camel_case_types)]
    enum FilterMode: u8{1} {
        /// 60Hz filter
        0 F_60Hz,
        /// 50Hz filter
        1 F_50Hz,
    }

    /// The device configuration register.
    register Configuration(addr = 0b0000, mode = rw, size = 1) {
        /// When no conversions are being performed, VBIAS may be disabled to reduce power
        /// dissipation. Set this bit to `true` to enable VBIAS before beginning a single (1-Shot)
        /// conversion. When automatic (continuous) conversion mode is selected,
        /// VBIAS remains on continuously.
        enable_bias_voltage: bool = false,
        /// Conversion mode (operating mode).
        conversion_mode: ConversionMode = ConversionMode::NormallyOff,
        /// When the conversion mode is set to [`ConversionMode::NormallyOff`],
        /// set this flag to start a conversion. You must enable VBIAS before
        /// starting a conversion. Note any filter capacitors at the RTDIN inputs
        /// need to charge before an accurate conversion can be performed.
        /// Therefore, enable VBIAS and wait at least 10.5 time constants of the input RC
        /// network plus an additional 1ms before initiating the conversion. Note that a single
        /// conversion requires approximately 52ms in 60Hz filter mode or 62.5ms in 50Hz
        /// filter mode to complete. This flag will auto-clear after the conversion.
        oneshot: bool = false,
        /// The wiring mode of the RTD.
        wiring_mode: WiringMode = WiringMode::TwoOrFourWire,
        /// Used to initiate fault detection. This is reset to [`FaultDetectionCycle::Finished`]
        /// when fault detection is done. When writing this register to initiate fault detection,
        /// the bias voltage must be enabled, conversion mode must be [`ConversionMode::NormallyOff`]
        /// and the `clear_fault_status` flag must be off.
        ///
        /// If the external RTD interface circuitry includes an input filter with a time constant
        /// greater than 100µs, the fault detection cycle timing should be controlled in the manual
        /// mode operation. For more information, refer to the datasheet.
        fault_detection_cycle: FaultDetectionCycle = FaultDetectionCycle::Finished,
        /// Set this flag while `oneshot` is off and writing [`FaultDetectionCycle::Finished`] to
        /// `fault_detection_cycle` to clear the fault status register. This bit will auto-clear.
        clear_fault_status: bool = false,
        /// The filter mode of the RTD.
        filter_mode: FilterMode = FilterMode::F_60Hz,
    }

    /// The RTD resistance data.
    register Resistance(addr = 0b0001, mode = r, size = 2) {
        /// The ratio of RTD resistance to reference resistance.
        /// The decimal value is obtained by dividing this value by `2^15`.
        resistance_ratio: u16{15} = 0,
        /// Whether a fault was detected.
        fault: bool = false,
    }

    /// The High Fault Threshold register selects the high trip threshold for RTD fault detection.
    ///
    /// The RTD High bit in the Fault Status Register is set if the
    /// RTD resistance register value is greater than or equal to
    /// the value in the High Fault Threshold register.
    /// The POR value of the High Fault Threshold register is FFFFh.
    register FaultThresholdHigh(addr = 0b0011, mode = rw, size = 2) {
        /// The ratio of RTD resistance to reference resistance.
        /// The decimal value is obtained by dividing this value by `2^15`.
        resistance_ratio: u16{15} = 0x7fff,
        /// Reserved bit
        _: u8{1},
    }

    /// The Low Fault Threshold register selects the low trip threshold for RTD fault detection.
    ///
    /// The RTD Low bit in the Fault Status Register is set if the
    /// RTD resistance value is less than or equal to the value in
    /// the Low Fault Threshold register. The POR value of the
    /// Low Fault Threshold register is 0000h.
    register FaultThresholdLow(addr = 0b0101, mode = rw, size = 2) {
        /// The ratio of RTD resistance to reference resistance.
        /// The decimal value is obtained by dividing this value by `2^15`.
        resistance_ratio: u16{15} = 0x0000,
        /// Reserved bit
        _: u8{1},
    }

    /// The fault status register.
    ///
    /// Depending on whether 2-, 3- or 4-wire mode is used,
    /// these status bits can have different possible causes.
    /// Refer to the datasheet for a table of possible causes.
    register FaultStatus(addr = 0b0111, mode = r, size = 1) {
        /// Measured resistance greater than High Fault Threshold value.
        high_threshold: bool = false,
        /// Measured resistance less than Low Fault Threshold value
        low_threshold: bool = false,
        /// `V_REFIN-` > 0.85 * `V_BIAS`.
        /// Detected on demand by initiating fault detection.
        v_refin_minus_exceeds_85_percent_of_v_bias: bool = false,
        /// `V_REFIN-` < 0.85 * `V_BIAS`, FORCE- open.
        /// Detected on demand by initiating fault detection.
        v_refin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool = false,
        /// `V_RTDIN-` < 0.85 * `V_BIAS`, FORCE- open.
        /// Detected on demand by initiating fault detection.
        v_rtdin_minus_below_85_percent_of_v_bias_with_force_minus_open: bool = false,
        /// Over/undervoltage, some protected input voltage exceeds VDD or below GND1
        over_or_undervoltage: bool = false,
        /// Reserved bits
        _: u8{2},
    }
}
