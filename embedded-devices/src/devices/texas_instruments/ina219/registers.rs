use crate::devices::texas_instruments::ina219::INA219Register;
use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::f64::{ElectricCurrent, ElectricPotential};
use uom::si::power::watt;

pub type INA219I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = INA219I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ INA219 ]

    /// Valid bus voltage ranges.
    #[allow(non_camel_case_types)]
    enum BusVoltageRange: u8{1} {
        0 U_16V,
        1 U_32V,
    }

    /// PGA settings
    #[allow(non_camel_case_types)]
    enum PgaGain: u8{2} {
        /// Gain 1, range ±40mV
        0b00 DIV_1,
        /// Gain 1/2, range ±80mV
        0b01 DIV_2,
        /// Gain 1/4, range ±160mV
        0b10 DIV_4,
        /// Gain 1/8, range ±320mV
        0b11 DIV_8,
    }

    /// Bus/Shunt ADC settings (resolution and averaging)
    #[allow(non_camel_case_types)]
    enum AdcResolution: u8{4} {
        /// 9bit resolution, 1 sample, 84 µs conversion time
        0b0000|0b0100 B_9,
        /// 10 bit resolution, 1 sample, 148 μs conversion time
        0b0001|0b0101 B_10,
        /// 11 bit resolution, 1 sample, 276 μs conversion time
        0b0010|0b0110 B_11,
        /// 12 bit resolution, 1 sample, 532 μs conversion time
        0b0011|0b0111|0b1000 B_12,
        /// 12 bit resolution, 2 samples, 1.06 ms conversion time
        0b1001 B_12_X_2,
        /// 12 bit resolution, 4 samples, 2.13 ms conversion time
        0b1010 B_12_X_4,
        /// 12 bit resolution, 8 samples, 4.26 ms conversion time
        0b1011 B_12_X_8,
        /// 12 bit resolution, 16 samples, 8.51 ms conversion time
        0b1100 B_12_X_16,
        /// 12 bit resolution, 32 samples, 17.02 ms conversion time
        0b1101 B_12_X_32,
        /// 12 bit resolution, 64 samples, 34.05 ms conversion time
        0b1110 B_12_X_64,
        /// 12 bit resolution, 128 samples, 68.10 ms conversion time
        0b1111 B_12_X_128,
    }

    /// Operating mode.
    enum OperatingMode: u8{3} {
        0b000 PowerDown,
        0b001 ShuntTriggered,
        0b010 BusTriggered,
        0b011 ShuntAndBusTriggered,
        0b100 AdcDisable,
        0b101 ShuntContinuous,
        0b110 BusContinuous,
        0b111 ShuntAndBusContinuous,
    }

    /// All-register reset, settings for bus voltage range, PGA Gain, ADC resolution/averaging
    register Configuration(addr = 0b000, mode = rw, size = 2) {
        /// Setting this flag generates a system reset that is the same
        /// as power-on reset. Resets all registers to default values.
        /// This bit self-clears.
        reset: bool = false,
        /// Reserved bit
        _: u8{1},
        /// The range for the bus voltage measurement
        bus_voltage_range: BusVoltageRange = BusVoltageRange::U_32V,
        /// Sets PGA gain and range. Note that the PGA defaults to ÷8 (320mV range).
        pga_gain: PgaGain = PgaGain::DIV_8,
        /// Bus ADC Resolution/Averaging setting
        bus_adc_resolution: AdcResolution = AdcResolution::B_12,
        /// Shunt ADC Resolution/Averaging setting
        shunt_adc_resolution: AdcResolution = AdcResolution::B_12,
        /// Selects continuous, triggered, or power-down mode of operation.
        operating_mode: OperatingMode = OperatingMode::ShuntAndBusContinuous,
    }

    /// Shunt voltage measurement data
    register ShuntVoltage(addr = 0b001, mode = r, size = 2) {
        /// The raw voltage measurement with 10µV/LSB resolution
        raw_value: i16 = 0 {
            quantity: ElectricPotential,
            unit: volt,
            lsb: 1f64 / 10_000f64,
        },
    }

    /// Bus voltage measurement data
    register BusVoltage(addr = 0b010, mode = r, size = 2) {
        /// The raw voltage measurement with 4mV/LSB resolution
        raw_value: u16{13} = 0 {
            quantity: ElectricPotential,
            unit: volt,
            lsb: 4f64 / 1000f64,
        },
        /// Reserved bit
        _: u8{1},
        /// Although the data from the last conversion can be read at any time,
        /// this flag indicates when data from a conversion is available in the
        /// data output registers. This flag bit is set after all conversions,
        /// averaging, and multiplications are complete.
        ///
        /// It will auto-clear when reading the [`Power`] register or
        /// when writing a new mode into the Operating Mode bits in the
        /// [`Configuration`] register (except for PowerDown or Disable).
        conversion_ready: bool = false,
        /// This flag is set when the Power or Current calculations are
        /// out of range. It indicates that current and power data may be meaningless.
        overflow: bool = false,
    }

    /// Power measurement data
    register Power(addr = 0b011, mode = r, size = 2) {
        /// The raw power measurement, W/LSB is determined by calibration register
        raw_value: i16 = 0,
    }

    /// Contains the amount of current flowing through the shunt resistor
    register Current(addr = 0b100, mode = r, size = 2) {
        /// The raw current measurement, A/LSB is determined by calibration register
        raw_value: i16 = 0,
    }

    /// Sets full-scale range and LSB of current and power measurements
    /// This register sets the current that corresponds to a full-scale drop across
    /// the shunt. Full-scale range and the LSB of the current and power measurement
    /// depend on the value entered in this register.
    register Calibration(addr = 0b101, mode = rw, size = 2) {
        /// The raw calibration value
        raw_value: u16{15} = 0,
        /// Reserved bit
        _: u8{1},
    }
}

impl AdcResolution {
    /// Returns the associated conversion time
    pub fn conversion_time_us(&self) -> u32 {
        match self {
            AdcResolution::B_9 => 84,
            AdcResolution::B_10 => 148,
            AdcResolution::B_11 => 276,
            AdcResolution::B_12 => 532,
            AdcResolution::B_12_X_2 => 1_060,
            AdcResolution::B_12_X_4 => 2_130,
            AdcResolution::B_12_X_8 => 4_260,
            AdcResolution::B_12_X_16 => 8_510,
            AdcResolution::B_12_X_32 => 17_020,
            AdcResolution::B_12_X_64 => 34_050,
            AdcResolution::B_12_X_128 => 68_100,
        }
    }
}

impl Power {
    /// Read the power, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the power register
    pub fn read_power(&self, current_lsb_na: u32) -> uom::si::f64::Power {
        // nW/LSB = 20 * nA/LSB, we divide by 50 instead to have the result in µW
        // which fits better in a u32.
        uom::si::f64::Power::new::<watt>(
            (self.read_raw_value() as i32 * current_lsb_na as i32 / 50) as f64 / 1_000_000.0,
        )
    }
}

impl Current {
    /// Read the current, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_current(&self, current_lsb_na: u32) -> ElectricCurrent {
        // We pre-divide by 1000 to have the result in µA which fits better in a u32.
        ElectricCurrent::new::<ampere>(
            (self.read_raw_value() as i32 * current_lsb_na as i32 / 1000) as f64 / 1_000_000.0,
        )
    }
}
