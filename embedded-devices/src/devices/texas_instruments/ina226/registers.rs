use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::f64;
use uom::si::f64::{ElectricCurrent, ElectricPotential};
use uom::si::power::watt;

/// Conversion averaging counts
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum AverageCount {
    /// Single sample without averaging.
    X_1 = 0b000,
    /// 4x averaging.
    X_4 = 0b001,
    /// 16x averaging.
    X_16 = 0b010,
    /// 64x averaging.
    X_64 = 0b011,
    /// 128x averaging.
    X_128 = 0b100,
    /// 256x averaging.
    X_256 = 0b101,
    /// 512x averaging.
    X_512 = 0b110,
    /// 1024x averaging.
    X_1024 = 0b111,
}

impl AverageCount {
    /// Returns the number of samples used for averaging
    pub fn count(&self) -> u32 {
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

/// Measurement conversion time
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum ConversionTime {
    /// 140 μs
    T_140 = 0b000,
    /// 204 μs
    T_204 = 0b001,
    /// 332 μs
    T_332 = 0b010,
    /// 588 μs
    T_588 = 0b011,
    /// 1.1 ms
    T_1100 = 0b100,
    /// 2.116 ms
    T_2116 = 0b101,
    /// 4.156 ms
    T_4156 = 0b110,
    /// 8.244 ms
    T_8244 = 0b111,
}

impl ConversionTime {
    /// Returns the associated time in µs
    pub fn us(&self) -> u32 {
        match self {
            ConversionTime::T_140 => 140,
            ConversionTime::T_204 => 204,
            ConversionTime::T_332 => 332,
            ConversionTime::T_588 => 588,
            ConversionTime::T_1100 => 1100,
            ConversionTime::T_2116 => 2116,
            ConversionTime::T_4156 => 4156,
            ConversionTime::T_8244 => 8244,
        }
    }
}

/// Operating mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum OperatingMode {
    PowerDown = 0b000,
    ShuntTriggered = 0b001,
    BusTriggered = 0b010,
    ShuntAndBusTriggered = 0b011,
    PowerDown2 = 0b100,
    ShuntContinuous = 0b101,
    BusContinuous = 0b110,
    ShuntAndBusContinuous = 0b111,
}

/// Device configuration register
#[device_register(super::INA226)]
#[register(address = 0x00, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    /// Setting this flag generates a system reset that is the same
    /// as power-on reset. Resets all registers to default values.
    /// This bit self-clears.
    #[register(default = false)]
    pub reset: bool,
    #[bondrewd(bit_length = 3, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// The count of ADC samples to average. Applies to all measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = AverageCount::X_1)]
    pub average_count: AverageCount,
    /// The conversion time of the bus voltage measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionTime::T_1100)]
    pub bus_conversion_time: ConversionTime,
    /// The conversion time of the shunt voltage measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionTime::T_1100)]
    pub shunt_conversion_time: ConversionTime,
    /// The operating mode of the device.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = OperatingMode::ShuntAndBusContinuous)]
    pub operating_mode: OperatingMode,
}

impl Configuration {
    /// Calculate the total conversion time based on configured operating mode, average count and conversion times.
    pub fn total_conversion_time_us(&self) -> u32 {
        let sample_measure_time = match self.read_operating_mode() {
            OperatingMode::PowerDown | OperatingMode::PowerDown2 => return 0,
            OperatingMode::BusContinuous | OperatingMode::BusTriggered => self.read_bus_conversion_time().us(),
            OperatingMode::ShuntContinuous | OperatingMode::ShuntTriggered => self.read_shunt_conversion_time().us(),
            OperatingMode::ShuntAndBusContinuous | OperatingMode::ShuntAndBusTriggered => {
                self.read_bus_conversion_time().us() + self.read_shunt_conversion_time().us()
            }
        };

        sample_measure_time * self.read_average_count().count()
    }
}

/// Shunt voltage measurement data
#[device_register(super::INA226)]
#[register(address = 0x01, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ShuntVoltage {
    /// The raw voltage measurement with 2.5µV/LSB resolution
    #[register(default = 0)]
    pub raw_value: i16,
}

impl ShuntVoltage {
    /// Read the shunt voltage
    pub fn read_voltage(&self) -> ElectricPotential {
        // 2.5µV/LSB
        ElectricPotential::new::<volt>(self.read_raw_value() as f64 / 400_000.0)
    }
}

/// Shunt voltage measurement data
#[device_register(super::INA226)]
#[register(address = 0x02, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct BusVoltage {
    /// The raw voltage measurement with 1.25mV/LSB resolution
    #[register(default = 0)]
    pub raw_value: i16,
}

impl BusVoltage {
    /// Read the shunt voltage
    pub fn read_voltage(&self) -> ElectricPotential {
        // 1.25mV/LSB
        ElectricPotential::new::<volt>(self.read_raw_value() as f64 / 800.0)
    }
}

/// Power measurement data
#[device_register(super::INA226)]
#[register(address = 0x03, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Power {
    /// The raw power measurement, W/LSB is determined by calibration register
    #[register(default = 0)]
    pub raw_value: u16,
}

impl Power {
    /// Read the power, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the power register
    pub fn read_power(&self, current_lsb_na: i64) -> f64::Power {
        // nW/LSB = 25 * nA/LSB
        f64::Power::new::<watt>((self.read_raw_value() as i64 * current_lsb_na * 25) as f64 / 1_000_000_000f64)
    }
}

/// Contains the amount of current flowing through the shunt resistor
#[device_register(super::INA226)]
#[register(address = 0x04, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Current {
    /// The raw current measurement, A/LSB is determined by calibration register
    #[register(default = 0)]
    pub raw_value: i16,
}

impl Current {
    /// Read the current, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_current(&self, current_lsb_na: i64) -> ElectricCurrent {
        ElectricCurrent::new::<ampere>((self.read_raw_value() as i64 * current_lsb_na) as f64 / 1_000_000_000f64)
    }
}

#[device_register(super::INA226)]
#[register(address = 0x05, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Calibration {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// The raw calibration value
    #[bondrewd(bit_length = 15)]
    #[register(default = 0)]
    pub raw_value: u16,
}

#[device_register(super::INA226)]
#[register(address = 0x06, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct MaskEnable {
    /// Alert pin shunt voltage over voltage
    #[register(default = false)]
    pub shunt_over_voltage: bool,
    /// Alert pin shunt voltage under voltage
    #[register(default = false)]
    pub shunt_under_voltage: bool,
    /// Alert pin bus voltage over voltage
    #[register(default = false)]
    pub bus_over_voltage: bool,
    /// Alert pin bus under voltage
    #[register(default = false)]
    pub bus_under_voltage: bool,
    /// Alert pin power over limit
    #[register(default = false)]
    pub power_over_limit: bool,
    /// Alert pin conversion ready
    #[register(default = false)]
    pub conversion_ready: bool,
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Alert function flag
    #[register(default = false)]
    pub alert_function_flag: bool,
    /// Conversion ready flag
    #[register(default = false)]
    pub conversion_ready_flag: bool,
    /// Math overflow flag
    #[register(default = false)]
    pub math_overflow_flag: bool,
    /// Alert pin polarity
    #[register(default = false)]
    pub alert_polarity: bool,
    /// Alert pin latch
    #[register(default = false)]
    pub alert_latch: bool,
}

#[device_register(super::INA226)]
#[register(address = 0x07, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AlertLimit {
    /// The value used to compare against the other register as configured in
    /// the MaskEnable register to determine if a limit has been exceeded.
    #[register(default = 0)]
    pub raw_value: u16,
}

#[device_register(super::INA226)]
#[register(address = 0xFE, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ManufacturerId {
    /// Manufacturer Id. Reads `"TI"` in ASCII.
    #[register(default = 0x5449)]
    pub id: u16,
}

#[device_register(super::INA226)]
#[register(address = 0xFF, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DieId {
    /// Die Id.
    #[bondrewd(bit_length = 12)]
    #[register(default = 0b001000100110)]
    pub id: u16,
    /// Revision.
    #[bondrewd(bit_length = 4)]
    #[register(default = 0b0000)]
    pub revision: u8,
}
