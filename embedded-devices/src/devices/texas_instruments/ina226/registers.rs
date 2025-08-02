use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::f64;
use uom::si::f64::{ElectricCurrent, ElectricPotential};
use uom::si::power::watt;

pub type INA226I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = INA226I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ super::INA226 ]

    /// Conversion averaging counts
    #[allow(non_camel_case_types)]
    enum AverageCount: u8{3} {
        /// Single sample without averaging.
        0b000 X_1,
        /// 4x averaging.
        0b001 X_4,
        /// 16x averaging.
        0b010 X_16,
        /// 64x averaging.
        0b011 X_64,
        /// 128x averaging.
        0b100 X_128,
        /// 256x averaging.
        0b101 X_256,
        /// 512x averaging.
        0b110 X_512,
        /// 1024x averaging.
        0b111 X_1024,
    }

    /// Measurement conversion time
    #[allow(non_camel_case_types)]
    enum ConversionTime: u8{3} {
        /// 140 μs
        0b000 T_140,
        /// 204 μs
        0b001 T_204,
        /// 332 μs
        0b010 T_332,
        /// 588 μs
        0b011 T_588,
        /// 1.1 ms
        0b100 T_1100,
        /// 2.116 ms
        0b101 T_2116,
        /// 4.156 ms
        0b110 T_4156,
        /// 8.244 ms
        0b111 T_8244,
    }

    /// Operating mode.
    enum OperatingMode: u8{3} {
        0b000 PowerDown,
        0b001 ShuntTriggered,
        0b010 BusTriggered,
        0b011 ShuntAndBusTriggered,
        0b100 PowerDown2,
        0b101 ShuntContinuous,
        0b110 BusContinuous,
        0b111 ShuntAndBusContinuous,
    }

    /// Device configuration register
    register Configuration(addr = 0x00, mode = rw, size = 2) {
        /// Setting this flag generates a system reset that is the same
        /// as power-on reset. Resets all registers to default values.
        /// This bit self-clears.
        reset: bool = false,
        /// Reserved bits
        _: u8{3},
        /// The count of ADC samples to average. Applies to all measurements.
        average_count: AverageCount = AverageCount::X_1,
        /// The conversion time of the bus voltage measurement
        bus_conversion_time: ConversionTime = ConversionTime::T_1100,
        /// The conversion time of the shunt voltage measurement
        shunt_conversion_time: ConversionTime = ConversionTime::T_1100,
        /// The operating mode of the device.
        operating_mode: OperatingMode = OperatingMode::ShuntAndBusContinuous,
    }

    /// Shunt voltage measurement data
    register ShuntVoltage(addr = 0x01, mode = r, size = 2) {
        /// The raw voltage measurement with 2.5µV/LSB resolution
        raw_value: i16 = 0 => {
            quantity: ElectricPotential,
            unit: volt,
            lsb: 1f64 / 400000f64,
        },
    }

    /// Bus voltage measurement data
    register BusVoltage(addr = 0x02, mode = r, size = 2) {
        /// The raw voltage measurement with 1.25mV/LSB resolution
        raw_value: i16 = 0 => {
            quantity: ElectricPotential,
            unit: volt,
            lsb: 1f64 / 800f64,
        },
    }

    /// Power measurement data
    register Power(addr = 0x03, mode = r, size = 2) {
        /// The raw power measurement, W/LSB is determined by calibration register
        raw_value: u16 = 0,
    }

    /// Contains the amount of current flowing through the shunt resistor
    register Current(addr = 0x04, mode = r, size = 2) {
        /// The raw current measurement, A/LSB is determined by calibration register
        raw_value: i16 = 0,
    }

    /// Calibration register
    register Calibration(addr = 0x05, mode = rw, size = 2) {
        /// Reserved bit
        _: u8{1},
        /// The raw calibration value
        raw_value: u16{15} = 0,
    }

    /// Mask/Enable register
    register MaskEnable(addr = 0x06, mode = rw, size = 2) {
        /// Alert pin shunt voltage over voltage
        shunt_over_voltage: bool = false,
        /// Alert pin shunt voltage under voltage
        shunt_under_voltage: bool = false,
        /// Alert pin bus voltage over voltage
        bus_over_voltage: bool = false,
        /// Alert pin bus under voltage
        bus_under_voltage: bool = false,
        /// Alert pin power over limit
        power_over_limit: bool = false,
        /// Alert pin conversion ready
        conversion_ready: bool = false,
        /// Reserved bits
        _: u8{5},
        /// Alert function flag
        alert_function_flag: bool = false,
        /// Conversion ready flag
        conversion_ready_flag: bool = false,
        /// Math overflow flag
        math_overflow_flag: bool = false,
        /// Alert pin polarity
        alert_polarity: bool = false,
        /// Alert pin latch
        alert_latch: bool = false,
    }

    /// Alert limit register
    register AlertLimit(addr = 0x07, mode = rw, size = 2) {
        /// The value used to compare against the other register as configured in
        /// the MaskEnable register to determine if a limit has been exceeded.
        raw_value: u16 = 0,
    }

    /// Manufacturer ID register
    register ManufacturerId(addr = 0xFE, mode = r, size = 2) {
        /// Manufacturer Id. Reads `"TI"` in ASCII.
        id: u16 = 0x5449,
    }

    /// Die ID register
    register DieId(addr = 0xFF, mode = r, size = 2) {
        /// Die Id.
        id: u16{12} = 0b001000100110,
        /// Revision.
        revision: u8{4} = 0b0000,
    }
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

impl Power {
    /// Read the power, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the power register
    pub fn read_power(&self, current_lsb_na: i64) -> f64::Power {
        // nW/LSB = 25 * nA/LSB
        f64::Power::new::<watt>((self.read_raw_value() as i64 * current_lsb_na * 25) as f64 / 1_000_000_000f64)
    }
}

impl Current {
    /// Read the current, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_current(&self, current_lsb_na: i64) -> ElectricCurrent {
        ElectricCurrent::new::<ampere>((self.read_raw_value() as i64 * current_lsb_na) as f64 / 1_000_000_000f64)
    }
}
