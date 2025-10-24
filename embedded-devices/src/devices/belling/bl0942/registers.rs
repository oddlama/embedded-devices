use embedded_interfaces::codegen::interface_objects;
use uom::si::f64::Frequency as QuantFrequency;
use uom::si::frequency::hertz;

// BL0942 uses a custom SPI protocol - you'll need to implement a custom codec
// For now, using UnsupportedCodec as placeholder
pub type BL0942I2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<
    crate::devices::belling::ChecksumError,
>;
pub type BL0942SpiCodec = crate::devices::belling::BellingSpiCodec<0x58, 0xa8>;

interface_objects! {
    register_defaults {
        codec_error = crate::devices::belling::ChecksumError,
        i2c_codec = BL0942I2cCodec,
        spi_codec = BL0942SpiCodec,
    }

    register_devices [ super::BL0942 ]

    // ========================================================================
    // Enumerations
    // ========================================================================

    /// Current channel gain selection
    #[allow(non_camel_case_types)]
    enum CurrentGain: u8{2} {
        /// Gain = 1x
        0b00 X_1,
        /// Gain = 4x
        0b01 X_4,
        /// Gain = 16x
        0b10 X_16,
        /// Gain = 24x
        0b11 X_24,
    }

    /// UART baud rate selection
    enum UartBaudRate: u8{2} {
        /// External pin SCLK_BPS defines 4800/9600 bps
        0b00 ExternalPin,
        /// Reserved
        0b01 Reserved,
        /// 19200 bps
        0b10 Baud19200,
        /// 38400 bps
        0b11 Baud38400,
    }

    /// Output function selection for CF1/CF2/ZX pins
    enum OutputFunction: u8{2} {
        /// Active energy pulse output (CF)
        0b00 ActiveEnergyPulse,
        /// Over-current logic output (O_C)
        0b01 OverCurrent,
        /// Zero-cross voltage (ZX_V)
        0b10 ZeroCrossVoltage,
        /// Zero-cross current (ZX_I)
        0b11 ZeroCrossCurrent,
    }

    /// Line cycle selection for fast RMS measurement
    enum FastRmsCycles: u8{3} {
        /// 0.5 cycles
        0b000 HalfCycle,
        /// 1 cycle
        0b001 OneCycle,
        /// 2 cycles
        0b010 TwoCycles,
        /// 4 cycles
        0b011 FourCycles,
        /// 8 cycles
        0b100 EightCycles,
        /// 8 cycles
        0b101 EightCyclesAlt1,
        /// 8 cycles
        0b110 EightCyclesAlt2,
        /// 8 cycles
        0b111 EightCyclesAlt3,
    }

    /// Number of line cycles until frequency measurement is updated
    #[allow(non_camel_case_types)]
    enum FrequencyUpdateCycles: u8{2} {
        /// 2 cycles
        0b00 N_2,
        /// 4 cycles
        0b01 N_4,
        /// 8 cycles
        0b10 N_8,
        /// 16 cycles
        0b11 N_16,
    }

    /// Power flow direction
    enum PowerDirection: u8{1} {
        /// Forward power flow
        0 Forward,
        /// Reverse power flow
        1 Reverse,
    }

    /// Anti-creep status
    enum CreepStatus: u8{1} {
        /// Anti-creep mechanism inactive
        0 Inactive,
        /// Anti-creep mechanism active
        1 Active,
    }

    /// RMS update rate selection
    #[allow(non_camel_case_types)]
    enum RmsUpdateRate: u8{1} {
        /// 400ms update period
        0 T_400ms,
        /// 800ms update period
        1 T_800ms,
    }

    /// Fast RMS calculation mode
    enum FastRmsMode: u8{1} {
        /// Full-wave RMS calculation
        0 FullWave,
        /// AC wave RMS calculation
        1 AcWave,
    }

    /// AC line frequency selection
    #[allow(non_camel_case_types)]
    enum AcFrequency: u8{1} {
        /// 50 Hz operation
        0 F_50Hz,
        /// 60 Hz operation
        1 F_60Hz,
    }

    /// CF_CNT accumulation mode
    enum AccumulationMode: u8{1} {
        /// Signed accumulation (can be negative)
        0 Signed,
        /// Absolute value accumulation
        1 Absolute,
    }

    /// Reset magic
    enum ResetMagic: u32{24} {
        /// Magic to reset the chip
        0x5a5a5a Reset,
        /// Any other value is ignored
        _ Ignored,
    }

    /// Write protection key
    enum ProtectionKey: u8 {
        /// User mode write is allowed
        0x55 Unlocked,
        /// User-mode writes are locked
        _ Locked,
    }

    // ========================================================================
    // Read-Only Registers
    // ========================================================================

    /// Current waveform data.
    ///
    /// Updated at 7.8 kSPS, 20-bit signed value
    register CurrentWaveform(addr = 0x01, mode = r, size = 3) {
        _: u8{4},
        /// Current waveform data.
        /// Updated at 7.8 kSPS sample rate
        ///
        /// Note: This is instantaneous waveform data, not suitable for direct unit conversion
        value: i32{20} = 0x00000,
    }

    /// Voltage waveform data.
    ///
    /// Updated at 7.8 kSPS, 20-bit signed value
    register VoltageWaveform(addr = 0x02, mode = r, size = 3) {
        /// Reserved bits.
        _: u8{4},
        /// Voltage waveform data.
        /// Updated at 7.8 kSPS sample rate
        ///
        /// Note: This is instantaneous waveform data, not suitable for direct unit conversion
        value: i32{20} = 0x00000,
    }

    /// Current RMS.
    register CurrentRms(addr = 0x03, mode = r, size = 3) {
        /// Current RMS value.
        /// Update frequency depends on the selected update rate in [`UserMode`]
        value: u32{24} = 0,
    }

    /// Voltage RMS.
    register VoltageRms(addr = 0x04, mode = r, size = 3) {
        /// Voltage RMS value.
        /// Update frequency depends on the selected update rate in [`UserMode`]
        value: u32{24} = 0,
    }

    /// Current fast RMS.
    register CurrentFastRms(addr = 0x05, mode = r, size = 3) {
        /// Fast RMS current value.
        /// Updated at faster rate for over-current detection
        value: u32{24} = 0,
    }

    /// Active power.
    ///
    /// Equation: WATT = 3537 * I(A) * V(V) * cos(φ) / Vref²
    /// Conversion: P(W) = WATT * Vref² / 3537
    /// Signed value, negative indicates reverse power flow
    register ActivePower(addr = 0x06, mode = r, size = 3) {
        /// Active power value.
        /// Negative value indicates reverse power flow
        value: i32{24} = 0,
    }

    /// Active energy pulse counter.
    ///
    /// Integrates active power over time.
    register EnergyCounter(addr = 0x07, mode = r, size = 3) {
        /// Active energy pulse counter.
        /// Accumulates energy pulses, can be auto-cleared after read depending on user mode
        /// register configuration. Unit conversion requires calibration based on CF pulse configuration.
        ///
        /// Value in active power register determines pulse time as WATT = (1638.4 * 256) / t_CF
        value: u32{24} = 0,
    }

    /// Line voltage frequency.
    ///
    /// The FREQ register is updated once every FREQ_CYC cycle.
    register Frequency(addr = 0x08, mode = r, size = 3) {
        /// Reserved bits.
        _: u8{8},
        /// Frequency measurement
        raw_frequency: u16{16} = 0x4e20 => {
            quantity: QuantFrequency,
            unit: hertz,
            from_raw: |x| 1_000_000f64 / (x as f64),
            into_raw: |x| 1_000_000f64 / x,
        },
    }

    /// System status register
    register Status(addr = 0x09, mode = r, size = 3) {
        /// Reserved bits.
        _: u16{14},
        /// Voltage zero-cross detection validity
        voltage_zx_invalid: bool = false,
        /// Current zero-cross detection validity
        current_zx_invalid: bool = false,
        /// Reserved bits
        _: u8{6},
        /// Anti-creep mechanism status
        creep_status: CreepStatus = CreepStatus::Inactive,
        /// Power flow direction
        power_direction: PowerDirection = PowerDirection::Forward,
    }

    // ========================================================================
    // Read-Write Registers
    // ========================================================================

    /// Current RMS offset.
    ///
    /// Used to compensate for DC offset in current measurement
    register CurrentRmsOffset(addr = 0x12, mode = rw, size = 3) {
        /// Reserved bits.
        _: u16{16},
        /// Current RMS offset value.
        /// Applies offset correction to measured current RMS
        value: u8 = 0x00,
    }

    /// Active power no-load threshold.
    ///
    /// Anti-creep threshold to suppress noise when no load
    /// Equation: WA_CREEP = WATT * (256 / 3125)
    register NoLoadThreshold(addr = 0x14, mode = rw, size = 3) {
        /// Reserved bits.
        _: u16{16},
        /// No-load threshold value.
        /// When active power < threshold, forces output to zero
        threshold: u8 = 0x0b,
    }

    /// Current fast RMS threshold.
    ///
    /// Threshold for over-current detection
    register OverCurrentThreshold(addr = 0x15, mode = rw, size = 3) {
        /// Reserved bits.
        _: u8{8},
        /// Over-current threshold value.
        /// Compares against upper 16 bits of I_FAST_RMS
        threshold: u16 = 0xffff,
    }

    /// Line cycle for fast RMS measurement.
    register FastRmsCycleConfig(addr = 0x16, mode = rw, size = 3) {
        /// Reserved bits
        _: u32{21},
        /// Number of line cycles for fast RMS averaging
        cycles: FastRmsCycles = FastRmsCycles::OneCycle,
    }

    /// Line cycle for frequency measurement.
    register FreqMeasurementConfig(addr = 0x17, mode = rw, size = 3) {
        /// Reserved bits
        _: u32{22},
        /// Number of line cycles until frequency measurement is updated
        update_cycles: FrequencyUpdateCycles = FrequencyUpdateCycles::N_16,
    }

    /// Logic output configuration.
    ///
    /// Configures what signals are output on CF1, CF2, and ZX pins
    register OutputConfig(addr = 0x18, mode = rw, size = 3) {
        /// Reserved bits
        _: u32{18},
        /// ZX pin output function selection
        zx_function: OutputFunction = OutputFunction::ZeroCrossVoltage,
        /// CF2 pin output function selection
        cf2_function: OutputFunction = OutputFunction::OverCurrent,
        /// CF1 pin output function selection
        cf1_function: OutputFunction = OutputFunction::ActiveEnergyPulse,
    }

    /// User mode selection.
    register UserMode(addr = 0x19, mode = rw, size = 3) {
        /// Reserved bits
        _: u16{14},
        /// UART baud rate selection
        uart_baud_rate: UartBaudRate = UartBaudRate::ExternalPin,
        /// CF_CNT accumulation mode
        accumulation_mode: AccumulationMode = AccumulationMode::Absolute,
        /// Auto-clear CF_CNT after read
        cf_cnt_auto_clear: bool = false,
        /// AC frequency selection
        ac_frequency: AcFrequency = AcFrequency::F_50Hz,
        /// Fast RMS calculation mode
        fast_rms_mode: FastRmsMode = FastRmsMode::FullWave,
        /// RMS update rate selection
        rms_update_rate: RmsUpdateRate = RmsUpdateRate::T_400ms,
        /// Enables active energy pulse output
        cf_enable: bool = true,
        /// Reserved bits
        _: u8{2} = 0b11,
    }

    /// Current channel gain.
    register CurrentGainConfig(addr = 0x1A, mode = rw, size = 3) {
        /// Reserved bits
        _: u32{22},
        /// Current channel gain setting
        /// Adjusts sensitivity of current measurement
        gain: CurrentGain = CurrentGain::X_16,
    }

    /// Software reset.
    register SoftReset(addr = 0x1C, mode = rw, size = 3) {
        /// Reset command: write 0x5A5A5A to trigger reset
        magic: ResetMagic = ResetMagic::Ignored,
    }

    /// User write protection.
    register WriteProtection(addr = 0x1D, mode = rw, size = 3) {
        /// Reserved bits
        _: u16{16} = 0,
        key: ProtectionKey = ProtectionKey::Locked,
    }
}

impl RmsUpdateRate {
    pub fn as_us(&self) -> u32 {
        match self {
            RmsUpdateRate::T_400ms => 400_000,
            RmsUpdateRate::T_800ms => 800_000,
        }
    }
}

impl CurrentGain {
    pub fn factor(&self) -> u8 {
        match self {
            CurrentGain::X_1 => 1,
            CurrentGain::X_4 => 4,
            CurrentGain::X_16 => 16,
            CurrentGain::X_24 => 24,
        }
    }
}
