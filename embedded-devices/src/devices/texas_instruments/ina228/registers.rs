use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;
use uom::num_rational::{Rational32, Rational64};
use uom::si::electric_charge::coulomb;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::energy::joule;
use uom::si::power::watt;
use uom::si::rational32;
use uom::si::rational32::ThermodynamicTemperature;
use uom::si::rational64;
use uom::si::rational64::{ElectricCharge, ElectricCurrent, ElectricPotential};
use uom::si::thermodynamic_temperature::degree_celsius;

/// Available shunt full-scale ranges
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AdcRange {
    /// ±163.84 mV
    Div4 = 0,
    /// ±40.96 mV
    Div1 = 1,
}

impl AdcRange {
    /// Returns the factor
    pub fn factor(&self) -> i8 {
        match self {
            AdcRange::Div4 => 4,
            AdcRange::Div1 => 1,
        }
    }
}

/// Device configuration register
#[device_register(super::INA228)]
#[register(address = 0x0, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    /// Setting this flag generates a system reset that is the same
    /// as power-on reset. Resets all registers to default values.
    /// This bit self-clears.
    #[register(default = false)]
    pub reset: bool,
    /// Setting this flag resets the energy and charge accumulation registers to 0.
    #[register(default = false)]
    pub reset_accumulators: bool,
    /// The delay for initial ADC conversion in steps of 2 ms (0 = 0ms, 255 = 510ms).
    /// Default: 0 (0ms).
    #[bondrewd(bit_length = 8)]
    #[register(default = 0)]
    pub conversion_delay: u8,
    /// Enables temperature compensation of an external shunt.
    /// Default: false.
    #[register(default = false)]
    pub temperature_compensation: bool,
    /// Enables temperature compensation of an external shunt.
    /// Default: false.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AdcRange::Div4)]
    pub adc_range: AdcRange,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

/// Operating mode
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum OperatingMode {
    Triggered = 0,
    Continuous = 1,
}

/// Measurement conversion time
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
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

impl ConversionTime {
    /// Returns the associated time in µs
    pub fn us(&self) -> u32 {
        match self {
            ConversionTime::T_50 => 50,
            ConversionTime::T_84 => 84,
            ConversionTime::T_150 => 150,
            ConversionTime::T_280 => 280,
            ConversionTime::T_540 => 540,
            ConversionTime::T_1052 => 1052,
            ConversionTime::T_2074 => 2074,
            ConversionTime::T_4120 => 4120,
        }
    }
}

/// Conversion averaging counts
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
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

/// ADC configuration register.
#[device_register(super::INA228)]
#[register(address = 0x1, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AdcConfiguration {
    /// The operating mode of the device. If all of the temperature, bus or shunt conversion
    /// are disabled, the device will sleep.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = OperatingMode::Continuous)]
    pub operating_mode: OperatingMode,
    /// Whether to enable temperature conversion
    #[register(default = true)]
    pub enable_temperature: bool,
    /// Whether to enable shunt voltage conversion
    #[register(default = true)]
    pub enable_shunt: bool,
    /// Whether to enable bus voltage conversion
    #[register(default = true)]
    pub enable_bus: bool,
    /// The conversion time of the bus voltage measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionTime::T_1052)]
    pub bus_conversion_time: ConversionTime,
    /// The conversion time of the shunt voltage measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionTime::T_1052)]
    pub shunt_conversion_time: ConversionTime,
    /// The conversion time of the temperature measurement
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = ConversionTime::T_1052)]
    pub temperature_conversion_time: ConversionTime,
    /// The count of ADC samples to average. Applies to all measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = AverageCount::X_1)]
    pub average_count: AverageCount,
}

/// The register provides the device with a conversion constant value that
/// represents shunt resistance used to calculate current value in Amperes.
/// This also sets the resolution for the Current register.
#[device_register(super::INA228)]
#[register(address = 0x2, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ShuntCalibration {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// The raw calibration value
    #[bondrewd(bit_length = 15)]
    #[register(default = 0x1000)]
    pub raw_value: u16,
}

/// Shunt temperature coefficient register
#[device_register(super::INA228)]
#[register(address = 0x3, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ShuntTemperatureCoefficient {
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Temperature coefficient of the shunt for temperature compensation
    /// correction. Calculated with respect to +25 °C.
    /// The full scale value of the register is 16383 ppm/°C.
    /// The field has a resolution of 1ppm/°C/LSB.
    #[bondrewd(bit_length = 14)]
    #[register(default = 0)]
    pub coefficient: u16,
}

/// Shunt voltage measurement register
#[device_register(super::INA228)]
#[register(address = 0x4, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct ShuntVoltage {
    /// Differential voltage measured across the shunt output.
    /// Resolution is 312.5 nV/LSB with `AdcRange::Div4` and 78.125 nV/LSB with `AdcRange::Div1`.
    #[bondrewd(bit_length = 20)]
    #[register(default = 0)]
    pub raw_value: i32,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

impl ShuntVoltage {
    /// Read the shunt voltage.
    /// Resolution is 312.5 nV/LSB with `AdcRange::Div4` and 78.125 nV/LSB with `AdcRange::Div1`.
    pub fn read_voltage(&self, adc_range: AdcRange) -> ElectricPotential {
        ElectricPotential::new::<volt>(Rational64::new(
            self.read_raw_value() as i64 * adc_range.factor() as i64,
            12_800_000,
        ))
    }
}

/// Bus voltage measurement register
#[device_register(super::INA228)]
#[register(address = 0x5, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct BusVoltage {
    /// Bus voltage output. Two's complement value, however always positive.
    /// Resolution is 195.3125 μV/LSB.
    #[bondrewd(bit_length = 20)]
    #[register(default = 0)]
    pub raw_value: i32,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

impl BusVoltage {
    /// Read the bus voltage
    pub fn read_voltage(&self) -> ElectricPotential {
        ElectricPotential::new::<volt>(Rational64::new(self.read_raw_value() as i64, 5120))
    }
}

/// Die temperature measurement register
#[device_register(super::INA228)]
#[register(address = 0x6, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Temperature {
    /// Internal die temperature measurement. Two's complement value.
    /// Resolution is 7.8125 m°C/LSB.
    #[register(default = 0)]
    pub raw_value: i16,
}

impl Temperature {
    /// Read the die temperature
    pub fn read_temperature(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new(self.read_raw_value() as i32, 128))
    }
}

/// Current calculation register
#[device_register(super::INA228)]
#[register(address = 0x7, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct Current {
    /// The raw calculated current. Two's complement value.
    /// Resolution in A/LSB is determined by the shunt calibration register.
    #[bondrewd(bit_length = 20)]
    #[register(default = 0)]
    pub raw_value: i32,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

impl Current {
    /// Read the current, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_current(&self, current_lsb_na: i64) -> ElectricCurrent {
        ElectricCurrent::new::<ampere>(Rational64::new(
            self.read_raw_value() as i64 * current_lsb_na,
            1_000_000_000,
        ))
    }
}

/// Power calculation register
#[device_register(super::INA228)]
#[register(address = 0x8, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct Power {
    /// The raw calculated power. Positive unsigned value. W/LSB is determined by calibration
    /// register.
    #[bondrewd(bit_length = 24)]
    #[register(default = 0)]
    pub raw_value: u32,
}

impl Power {
    /// Read the power, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the power register
    pub fn read_power(&self, current_lsb_na: i64) -> rational64::Power {
        rational64::Power::new::<watt>(Rational64::new(
            self.read_raw_value() as i64 * current_lsb_na * 32,
            1_000_000_000 * 10,
        ))
    }
}

/// Energy accumulation register. Only contains a valid value when operating in continuous mode.
#[device_register(super::INA228)]
#[register(address = 0x9, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 5)]
pub struct Energy {
    /// The raw accumulated energy. Positive unsigned value. W/LSB is determined by calibration
    /// register.
    #[bondrewd(bit_length = 40)]
    #[register(default = 0)]
    pub raw_value: u64,
}

impl Energy {
    /// Read the energy, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the energy register
    pub fn read_energy(&self, current_lsb_na: i64) -> rational64::Energy {
        rational64::Energy::new::<joule>(Rational64::new(
            self.read_raw_value() as i64 * current_lsb_na * 512,
            1_000_000_000 * 10,
        ))
    }
}

/// Charge accumulation register. Only contains a valid value when operating in continuous mode.
#[device_register(super::INA228)]
#[register(address = 0xa, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 5)]
pub struct Charge {
    /// The raw accumulated charge. Two's complement value.
    /// Resolution in A/LSB is determined by the shunt calibration register.
    #[bondrewd(bit_length = 40)]
    #[register(default = 0)]
    pub raw_value: i64,
}

impl Charge {
    /// Read the charge, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_charge(&self, current_lsb_na: i64) -> ElectricCharge {
        ElectricCharge::new::<coulomb>(Rational64::new(self.read_raw_value() * current_lsb_na, 1_000_000_000))
    }
}

/// Alert pin behavior
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertBehavior {
    /// Alert pin and flag bit reset to the idle state when the fault has been cleared
    Transparent = 0,
    /// Alert pin and flag bit remain active following a fault until the `DiagnosticsAndAlert` Register has been read.
    Latched = 1,
}

/// Alert pin polarity
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum AlertPolarity {
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Diagnostics and alert register
#[device_register(super::INA228)]
#[register(address = 0xb, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DiagnosticsAndAlert {
    /// Determines whether the alert pin resets automatically or latches.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertBehavior::Transparent)]
    pub alert_latch: AlertBehavior,
    /// If set, configures the alert pin to be asserted when the `conversion_ready` flag is
    /// asserted, indicating that a conversion cycle has completed.
    #[register(default = false)]
    pub alert_conversion_ready: bool,
    /// When enabled, alert assertions will be delayed until averaging completes.
    #[register(default = false)]
    pub alert_slow: bool,
    /// Determines the polarity of the alert pin
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = AlertPolarity::ActiveLow)]
    pub alert_polarity: AlertPolarity,

    /// Indicates whether the energy accumulation register has overflowed.
    /// This bit clears when the `Energy` register is read.
    #[register(default = false)]
    pub energy_overflow: bool,
    /// Indicates whether the charge accumulation register has overflowed.
    /// This bit clears when the `Charge` register is read.
    #[register(default = false)]
    pub charge_overflow: bool,
    /// Indicates whether a internal arithmetic operation has overflowed when calculating current
    /// or power measurements. Must be manually cleared by triggering another conversion or by
    /// clearing the accumulators by setting `Configuration::reset_accumulators`.
    #[register(default = false)]
    pub math_overflow: bool,
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// This flag is set when the temperature measurement exceeds the threshold limit in the temperature over-limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub temperature_over_limit: bool,
    /// This flag is set when the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub shunt_over_limit: bool,
    /// This flag is set when the shunt voltage measurement falls below the threshold limit in the shunt under-limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub shunt_under_limit: bool,
    /// This flag is set when the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub bus_over_limit: bool,
    /// This flag is set when the bus voltage measurement falls below the threshold limit in the bus under-limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub bus_under_limit: bool,
    /// This flag is set when the power measurement exceeds the threshold limit in the power limit register.
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub power_over_limit: bool,
    /// This flag is set when the conversion is completed or when writing to the `Configuration` register (except when configuring shutdown).
    /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
    #[register(default = false)]
    pub conversion_ready: bool,
    /// This flag is set to false if a checksum error is detected in the device trim memory space.
    #[register(default = true)]
    pub memory_ok: bool,
}

macro_rules! define_shunt_voltage_threshold_register {
    ($name:ident, desc = $desc:expr, address = $address:expr, value_default = $value_default:expr, resolution = $resolution_factor:expr, value_doc = $value_doc:expr) => {
        #[doc = $desc]
        #[device_register(super::INA228)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            #[doc = "The"]
            #[doc = $value_doc]
            #[register(default = $value_default)]
            pub raw_value: i16,
        }

        impl $name {
            #[doc = "Reads the"]
            #[doc = $value_doc]
            pub fn read_voltage_threshold(&self, adc_range: AdcRange) -> rational32::ElectricPotential {
                rational32::ElectricPotential::new::<volt>(Rational32::new(
                    self.read_raw_value() as i32 * adc_range.factor() as i32,
                    $resolution_factor,
                ))
            }

            #[doc = "Writes the"]
            #[doc = $value_doc]
            pub fn write_voltage_threshold(
                &mut self,
                voltage_threshold: rational32::ElectricPotential,
                adc_range: AdcRange,
            ) -> Result<(), core::num::TryFromIntError> {
                let value = voltage_threshold.get::<volt>();
                let value = (value * Rational32::new_raw($resolution_factor, adc_range.factor() as i32))
                    .to_integer()
                    .try_into()?;
                self.write_raw_value(value);
                Ok(())
            }

            #[doc = "Writes the"]
            #[doc = $value_doc]
            pub fn with_voltage_threshold(
                mut self,
                voltage_threshold: rational32::ElectricPotential,
                adc_range: AdcRange,
            ) -> Result<Self, core::num::TryFromIntError> {
                self.write_voltage_threshold(voltage_threshold, adc_range)?;
                Ok(self)
            }
        }
    };
}

define_shunt_voltage_threshold_register!(
    ShuntOvervoltageThreshold,
    desc = "Shunt Overvoltage Threshold Register",
    address = 0xc,
    value_default = i16::MAX,
    resolution = 800_000,
    value_doc = r#"
threshold for comparison of the value to detect Shunt Overvoltage (overcurrent
protection). Two's complement value.
Resolution is 5 µV/LSB with `AdcRange::Div4` and 1.25 µV/LSB with `AdcRange::Div1`.

If negative values are entered in this register, then a shunt voltage measurement of 0 V
will trip this alarm. When using negative values for the shunt under and overvoltage
thresholds be aware that the over voltage threshold must be set to the larger (that is,
less negative) of the two values.
"#
);

define_shunt_voltage_threshold_register!(
    ShuntUndervoltageThreshold,
    desc = "Shunt Undervoltage Threshold Register",
    address = 0xd,
    value_default = i16::MIN,
    resolution = 800_000,
    value_doc = r#"
threshold for comparison of the value to detect Shunt Undervoltage (undercurrent
protection). Two's complement value.
Resolution is 5 µV/LSB with `AdcRange::Div4` and 1.25 µV/LSB with `AdcRange::Div1`.
"#
);

macro_rules! define_bus_voltage_threshold_register {
    ($name:ident, desc = $desc:expr, address = $address:expr, value_default = $value_default:expr, resolution = $resolution_factor:expr, value_doc = $value_doc:expr) => {
        #[doc = $desc]
        #[device_register(super::INA228)]
        #[register(address = $address, mode = "rw")]
        #[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
        pub struct $name {
            #[bondrewd(bit_length = 1, reserve)]
            #[allow(dead_code)]
            pub reserved: u8,
            #[doc = "The"]
            #[doc = $value_doc]
            #[bondrewd(bit_length = 15)]
            #[register(default = $value_default)]
            pub raw_value: u16,
        }

        impl $name {
            #[doc = "Reads the"]
            #[doc = $value_doc]
            pub fn read_voltage_threshold(&self) -> rational32::ElectricPotential {
                rational32::ElectricPotential::new::<volt>(Rational32::new(
                    self.read_raw_value() as i32,
                    $resolution_factor,
                ))
            }

            #[doc = "Writes the"]
            #[doc = $value_doc]
            pub fn write_voltage_threshold(
                &mut self,
                voltage_threshold: rational32::ElectricPotential,
            ) -> Result<(), core::num::TryFromIntError> {
                let value = voltage_threshold.get::<volt>();
                let value = (value * Rational32::from_integer($resolution_factor))
                    .to_integer()
                    .try_into()?;
                self.write_raw_value(value);
                Ok(())
            }

            #[doc = "Writes the"]
            #[doc = $value_doc]
            pub fn with_voltage_threshold(
                mut self,
                voltage_threshold: rational32::ElectricPotential,
            ) -> Result<Self, core::num::TryFromIntError> {
                self.write_voltage_threshold(voltage_threshold)?;
                Ok(self)
            }
        }
    };
}

define_bus_voltage_threshold_register!(
    BusOvervoltageThreshold,
    desc = "Bus Overvoltage Threshold Register",
    address = 0xe,
    value_default = u16::MAX,
    resolution = 320,
    value_doc = r#"
threshold for comparison of the value to detect Bus Overvoltage (overvoltage
protection). Unsigned representation, positive value only. Resolution is 3.125 mV/LSB.
"#
);

define_bus_voltage_threshold_register!(
    BusUndervoltageThreshold,
    desc = "Bus Undervoltage Threshold Register",
    address = 0xf,
    value_default = 0,
    resolution = 320,
    value_doc = r#"
threshold for comparison of the value to detect Bus Undervoltage (undervoltage
protection). Unsigned representation, positive value only. Resolution is 3.125 mV/LSB.
"#
);

/// Temperature Over-Limit Threshold Register
#[device_register(super::INA228)]
#[register(address = 0x10, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct TemperatureOverlimitThreshold {
    /// The threshold for comparison of the value to detect over temperature measurements.
    /// Two's complement value. The value entered in this field compares directly against the value
    /// from the `Temperature` register to determine if an over temperature condition exists.
    /// Resolution is 7.8125 m°C/LSB.
    #[register(default = i16::MAX)]
    pub raw_value: i16,
}

impl TemperatureOverlimitThreshold {
    /// Reads the temperature overlimit threshold in °C with a resolution of 7.8125 m°C/LSB.
    pub fn read_temperature_limit(&self) -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(Rational32::new(self.read_raw_value().into(), 128))
    }

    /// Writes the temperature overlimit threshold in °C with a resolution of 7.8125 m°C/LSB.
    /// The passed temperature will be truncated (rounded down) if necessary.
    pub fn write_temperature_limit(
        &mut self,
        temperature_limit: ThermodynamicTemperature,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = temperature_limit.get::<degree_celsius>();
        let value = (value * Rational32::from_integer(128)).to_integer().try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the temperature overlimit threshold in °C with a resolution of 7.8125 m°C/LSB.
    /// The passed temperature will be truncated (rounded down) if necessary.
    pub fn with_temperature_limit(
        mut self,
        temperature_limit: ThermodynamicTemperature,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_temperature_limit(temperature_limit)?;
        Ok(self)
    }
}

/// Power Over-Limit Threshold Register
#[device_register(super::INA228)]
#[register(address = 0x11, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PowerOverlimitThreshold {
    /// The threshold for comparison of the value to detect power over-limit measurements.
    /// Unsigned representation, positive value only. The value entered in this field compares
    /// directly against the value from the `Power` register to determine if an over-power condition
    /// exists. Resolution is 256 * Power LSB.
    #[register(default = u16::MAX)]
    pub raw_value: u16,
}

impl PowerOverlimitThreshold {
    /// Reads the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn read_power_limit(&self, current_lsb_na: i64) -> rational64::Power {
        rational64::Power::new::<watt>(Rational64::new(
            self.read_raw_value() as i64 * current_lsb_na * 32 * 256,
            1_000_000_000 * 10,
        ))
    }

    /// Writes the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// The passed power will be truncated (rounded down) if necessary.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn write_power_limit(
        &mut self,
        power_limit: rational64::Power,
        current_lsb_na: i64,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = power_limit.get::<watt>();
        let value = (value * Rational64::new_raw(1_000_000_000 * 10, current_lsb_na * 32 * 256))
            .to_integer()
            .try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// The passed power will be truncated (rounded down) if necessary.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn with_power_limit(
        mut self,
        power_limit: rational64::Power,
        current_lsb_na: i64,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_power_limit(power_limit, current_lsb_na)?;
        Ok(self)
    }
}

/// Manufacturer Id Register
#[device_register(super::INA228)]
#[register(address = 0x3e, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ManufacturerId {
    /// Manufacturer Id. Reads `"TI"` in ASCII.
    #[register(default = 0x5449)]
    pub id: u16,
}

/// Device Id Register
#[device_register(super::INA228)]
#[register(address = 0x3f, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceId {
    /// Device Id.
    #[bondrewd(bit_length = 12)]
    #[register(default = 0x228)]
    pub id: u16,
    /// Revision.
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x1)]
    pub revision: u8,
}
