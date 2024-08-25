use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;

/// Valid bus voltage ranges.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum BusVoltageRange {
    U_16V = 0,
    #[default]
    U_32V = 1,
}

/// PGA settings
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum PgaGain {
    /// Gain 1, range ±40mV
    DIV_1 = 0b00,
    /// Gain 1/2, range ±80mV
    DIV_2 = 0b01,
    /// Gain 1/4, range ±160mV
    DIV_4 = 0b10,
    /// Gain 1/8, range ±320mV
    #[default]
    DIV_8 = 0b11,
}

/// Bus/Shunt ADC settings (resolution and averaging)
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum AdcResolution {
    /// 9bit resolution, 1 sample, 84µs conversion time
    B_9 = 0b0000,
    /// Secondary representation for [`Self::B_9`]
    B_9_a = 0b0100,
    /// 10 bit resolution, 1 sample, 148 μs conversion time
    B_10 = 0b0001,
    /// Secondary representation for [`Self::B_10`]
    B_10_a = 0b0101,
    /// 11 bit resolution, 1 sample, 276 μs conversion time
    B_11 = 0b0010,
    /// Secondary representation for [`Self::B_11`]
    B_11_a = 0b0110,
    /// 12 bit resolution, 1 sample, 532 μs conversion time
    #[default]
    B_12 = 0b0011,
    /// Secondary representation for [`Self::B_12`]
    B_12_a = 0b0111,
    /// Tertiary representation for [`Self::B_12`]
    B_12_b = 0b1000,
    /// 12 bit resolution, 2 sample, 1.06 ms conversion time
    B_12_X_2 = 0b1001,
    /// 12 bit resolution, 4 sample, 2.13 ms conversion time
    B_12_X_4 = 0b1010,
    /// 12 bit resolution, 8 sample, 4.26 ms conversion time
    B_12_X_8 = 0b1011,
    /// 12 bit resolution, 16 sample, 8.51 ms conversion time
    B_12_X_16 = 0b1100,
    /// 12 bit resolution, 32 sample, 17.02 ms conversion time
    B_12_X_32 = 0b1101,
    /// 12 bit resolution, 64 sample, 34.05 ms conversion time
    B_12_X_64 = 0b1110,
    /// 12 bit resolution, 128 sample, 68.10 ms conversion time
    B_12_X_128 = 0b1111,
}

/// Operating mode.
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum OperatingMode {
    PowerDown = 0b000,
    ShuntTriggered = 0b001,
    BusTriggered = 0b010,
    ShuntAndBusTriggered = 0b011,
    AdcDisable = 0b100,
    ShuntContinuous = 0b101,
    BusContinuous = 0b110,
    #[default]
    ShuntAndBusContinuous = 0b111,
}

/// All-register reset, settings for bus voltage range, PGA Gain, ADC resolution/averaging
#[device_register(super::INA219)]
#[register(address = 0b000, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    /// Setting this flag generates a system reset that is the same
    /// as power-on reset. Resets all registers to default values.
    /// This bit self-clears.
    pub reset: bool,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// The range for the bus voltage measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    pub bus_voltage_range: BusVoltageRange,
    /// Sets PGA gain and range. Note that the PGA defaults to ÷8 (320mV range).
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub pga_gain: PgaGain,
    /// Bus ADC Resolution/Averaging setting
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    pub bus_adc_resolution: AdcResolution,
    /// Shunt ADC Resolution/Averaging setting
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    pub shunt_adc_resolution: AdcResolution,
    /// Selects continuous, triggered, or power-down mode of operation.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub operating_mode: OperatingMode,
}

/// Shunt voltage measurement data
#[device_register(super::INA219)]
#[register(address = 0b001, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ShuntVoltage {
    /// The raw voltage measurement with 0.01mV/LSB resolution
    pub raw_value: i16,
}

/// Bus voltage measurement data
#[device_register(super::INA219)]
#[register(address = 0b010, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct BusVoltage {
    /// The raw voltage measurement with 0.04mV/LSB resolution
    #[bondrewd(bit_length = 13)]
    pub raw_value: u16,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Although the data from the last conversion can be read at any time,
    /// this flag indicates when data from a conversion is available in the
    /// data output registers. The CNVR bit is set after all conversions,
    /// averaging, and multiplications are complete.
    ///
    /// It will auto-clear when reading the [`Power`] register or
    /// when writing a new mode into the Operating Mode bits in the
    /// [`Configuration`] register (except for PowerDown or Disable).
    pub conversion_ready: bool,
    /// This flag is set when the Power or Current calculations are
    /// out of range. It indicates that current and power data may be meaningless.
    pub overflow: bool,
}


/// Power measurement data
#[device_register(super::INA219)]
#[register(address = 0b011, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Power {
    /// The raw power measurement
    // FIXME: unclear whether this is signed or not
    // FIXME: range unclear
    pub raw_value: i16,
}

/// Contains the value of the current flowing through the shunt resistor
#[device_register(super::INA219)]
#[register(address = 0b100, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Current {
    /// The raw current measurement
    // FIXME: range unclear
    pub raw_value: i16,
}

/// Sets full-scale range and LSB of current and power measurements
/// This register sets the current that corresponds to a full-scale drop across
/// the shunt. Full-scale range and the LSB of the current and power measurement
/// depend on the value entered in this register.
#[device_register(super::INA219)]
#[register(address = 0b101, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Calibration {
    /// The raw calibration value
    #[bondrewd(bit_length = 15)]
    pub raw_value: u16,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}
