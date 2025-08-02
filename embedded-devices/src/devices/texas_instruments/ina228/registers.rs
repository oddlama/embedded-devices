use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

use uom::si::electric_charge::coulomb;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::energy::joule;
use uom::si::f64::{self, ElectricCharge, ElectricCurrent, ElectricPotential, ThermodynamicTemperature};
use uom::si::power::watt;
use uom::si::thermodynamic_temperature::degree_celsius;

pub type INA228I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = INA228I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ super::INA228 ]

    /// Available shunt full-scale ranges
    enum AdcRange: u8{1} {
        /// ±163.84 mV
        0 Div4,
        /// ±40.96 mV
        1 Div1,
    }

    /// Operating mode
    enum OperatingMode: u8{1} {
        0 Triggered,
        1 Continuous,
    }

    /// Measurement conversion time
    #[allow(non_camel_case_types)]
    enum ConversionTime: u8{3} {
        /// 50µs
        0 T_50,
        /// 84μs
        1 T_84,
        /// 150μs
        2 T_150,
        /// 280μs
        3 T_280,
        /// 540μs
        4 T_540,
        /// 1052μs
        5 T_1052,
        /// 2074μs
        6 T_2074,
        /// 4120μs
        7 T_4120,
    }

    /// Conversion averaging counts
    #[allow(non_camel_case_types)]
    enum AverageCount: u8{3} {
        /// Single sample without averaging.
        0 X_1,
        /// 4x averaging.
        1 X_4,
        /// 16x averaging.
        2 X_16,
        /// 64x averaging.
        3 X_64,
        /// 128x averaging.
        4 X_128,
        /// 256x averaging.
        5 X_256,
        /// 512x averaging.
        6 X_512,
        /// 1024x averaging.
        7 X_1024,
    }

    /// Alert pin behavior
    enum AlertBehavior: u8{1} {
        /// Alert pin and flag bit reset to the idle state when the fault has been cleared
        0 Transparent,
        /// Alert pin and flag bit remain active following a fault until the `DiagnosticsAndAlert` Register has been read.
        1 Latched,
    }

    /// Alert pin polarity
    enum AlertPolarity: u8{1} {
        0 ActiveLow,
        1 ActiveHigh,
    }

    /// Device configuration register
    register Configuration(addr = 0x0, mode = rw, size = 2) {
        /// Setting this flag generates a system reset that is the same
        /// as power-on reset. Resets all registers to default values.
        /// This bit self-clears.
        reset: bool = false,
        /// Setting this flag resets the energy and charge accumulation registers to 0.
        reset_accumulators: bool = false,
        /// The delay for initial ADC conversion in steps of 2 ms (0 = 0ms, 255 = 510ms).
        /// Default: 0 (0ms).
        conversion_delay: u8{8} = 0,
        /// Enables temperature compensation of an external shunt.
        /// Default: false.
        temperature_compensation: bool = false,
        /// Enables temperature compensation of an external shunt.
        /// Default: false.
        adc_range: AdcRange = AdcRange::Div4,
        /// Reserved bits
        _: u8{4},
    }

    /// ADC configuration register.
    register AdcConfiguration(addr = 0x1, mode = rw, size = 2) {
        /// The operating mode of the device. If all of the temperature, bus or shunt conversion
        /// are disabled, the device will sleep.
        operating_mode: OperatingMode = OperatingMode::Continuous,
        /// Whether to enable temperature conversion
        enable_temperature: bool = true,
        /// Whether to enable shunt voltage conversion
        enable_shunt: bool = true,
        /// Whether to enable bus voltage conversion
        enable_bus: bool = true,
        /// The conversion time of the bus voltage measurement
        bus_conversion_time: ConversionTime = ConversionTime::T_1052,
        /// The conversion time of the shunt voltage measurement
        shunt_conversion_time: ConversionTime = ConversionTime::T_1052,
        /// The conversion time of the temperature measurement
        temperature_conversion_time: ConversionTime = ConversionTime::T_1052,
        /// The count of ADC samples to average. Applies to all measurements.
        average_count: AverageCount = AverageCount::X_1,
    }

    /// The register provides the device with a conversion constant value that
    /// represents shunt resistance used to calculate current value in Amperes.
    /// This also sets the resolution for the Current register.
    register ShuntCalibration(addr = 0x2, mode = rw, size = 2) {
        /// Reserved bit
        _: u8{1},
        /// The raw calibration value
        raw_value: u16{15} = 0x1000,
    }

    /// Shunt temperature coefficient register
    register ShuntTemperatureCoefficient(addr = 0x3, mode = rw, size = 2) {
        /// Reserved bits
        _: u8{2},
        /// Temperature coefficient of the shunt for temperature compensation
        /// correction. Calculated with respect to +25 °C.
        /// The full scale value of the register is 16383 ppm/°C.
        /// The field has a resolution of 1ppm/°C/LSB.
        coefficient: u16{14} = 0,
    }

    /// Shunt voltage measurement register
    register ShuntVoltage(addr = 0x4, mode = r, size = 3) {
        /// Differential voltage measured across the shunt output.
        /// Resolution is 312.5 nV/LSB with `AdcRange::Div4` and 78.125 nV/LSB with `AdcRange::Div1`.
        raw_value: i32{20} = 0,
        /// Reserved bits
        _: u8{4},
    }

    /// Bus voltage measurement register
    register BusVoltage(addr = 0x5, mode = r, size = 3) {
        /// Bus voltage output. Two's complement value, however always positive.
        /// Resolution is 195.3125 μV/LSB.
        raw_value: i32{20} = 0 => {
            quantity: ElectricPotential,
            unit: volt,
            lsb: 1f64 / 5120f64,
        },
        /// Reserved bits
        _: u8{4},
    }

    /// Die temperature measurement register
    register Temperature(addr = 0x6, mode = r, size = 2) {
        /// Internal die temperature measurement. Two's complement value.
        /// Resolution is 7.8125 m°C/LSB.
        raw_value: i16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// Current calculation register
    register Current(addr = 0x7, mode = r, size = 3) {
        /// The raw calculated current. Two's complement value.
        /// Resolution in A/LSB is determined by the shunt calibration register.
        raw_value: i32{20} = 0,
        /// Reserved bits
        _: u8{4},
    }

    /// Power calculation register
    register Power(addr = 0x8, mode = r, size = 3) {
        /// The raw calculated power. Positive unsigned value. W/LSB is determined by calibration
        /// register.
        raw_value: u32{24} = 0,
    }

    /// Energy accumulation register. Only contains a valid value when operating in continuous mode.
    register Energy(addr = 0x9, mode = r, size = 5) {
        /// The raw accumulated energy. Positive unsigned value. W/LSB is determined by calibration
        /// register.
        raw_value: u64{40} = 0,
    }

    /// Charge accumulation register. Only contains a valid value when operating in continuous mode.
    register Charge(addr = 0xa, mode = r, size = 5) {
        /// The raw accumulated charge. Two's complement value.
        /// Resolution in A/LSB is determined by the shunt calibration register.
        raw_value: i64{40} = 0,
    }

    /// Diagnostics and alert register
    register DiagnosticsAndAlert(addr = 0xb, mode = rw, size = 2) {
        /// Determines whether the alert pin resets automatically or latches.
        alert_latch: AlertBehavior = AlertBehavior::Transparent,
        /// If set, configures the alert pin to be asserted when the `conversion_ready` flag is
        /// asserted, indicating that a conversion cycle has completed.
        alert_conversion_ready: bool = false,
        /// When enabled, alert assertions will be delayed until averaging completes.
        alert_slow: bool = false,
        /// Determines the polarity of the alert pin
        alert_polarity: AlertPolarity = AlertPolarity::ActiveLow,

        /// Indicates whether the energy accumulation register has overflowed.
        /// This bit clears when the `Energy` register is read.
        energy_overflow: bool = false,
        /// Indicates whether the charge accumulation register has overflowed.
        /// This bit clears when the `Charge` register is read.
        charge_overflow: bool = false,
        /// Indicates whether a internal arithmetic operation has overflowed when calculating current
        /// or power measurements. Must be manually cleared by triggering another conversion or by
        /// clearing the accumulators by setting `Configuration::reset_accumulators`.
        math_overflow: bool = false,
        /// Reserved bit
        _: u8{1},
        /// This flag is set when the temperature measurement exceeds the threshold limit in the temperature over-limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        temperature_over_limit: bool = false,
        /// This flag is set when the shunt voltage measurement exceeds the threshold limit in the shunt over-limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        shunt_over_limit: bool = false,
        /// This flag is set when the shunt voltage measurement falls below the threshold limit in the shunt under-limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        shunt_under_limit: bool = false,
        /// This flag is set when the bus voltage measurement exceeds the threshold limit in the bus over-limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        bus_over_limit: bool = false,
        /// This flag is set when the bus voltage measurement falls below the threshold limit in the bus under-limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        bus_under_limit: bool = false,
        /// This flag is set when the power measurement exceeds the threshold limit in the power limit register.
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        power_over_limit: bool = false,
        /// This flag is set when the conversion is completed or when writing to the `Configuration` register (except when configuring shutdown).
        /// When `alert_behavior` is `AlertBehavior::Latched` this flag is cleared by reading this register.
        conversion_ready: bool = false,
        /// This flag is set to false if a checksum error is detected in the device trim memory space.
        memory_ok: bool = true,
    }

    /// Shunt Overvoltage Threshold Register
    register ShuntOvervoltageThreshold(addr = 0xc, mode = rw, size = 2) {
        /// The threshold for comparison of the value to detect Shunt Overvoltage (overcurrent
        /// protection). Two's complement value.
        /// Resolution is 5 µV/LSB with `AdcRange::Div4` and 1.25 µV/LSB with `AdcRange::Div1`.
        ///
        /// If negative values are entered in this register, then a shunt voltage measurement of 0 V
        /// will trip this alarm. When using negative values for the shunt under and overvoltage
        /// thresholds be aware that the over voltage threshold must be set to the larger (that is,
        /// less negative) of the two values.
        raw_value: i16 = 32767, // i16::MAX
    }

    /// Shunt Undervoltage Threshold Register
    register ShuntUndervoltageThreshold(addr = 0xd, mode = rw, size = 2) {
        /// The threshold for comparison of the value to detect Shunt Undervoltage (undercurrent
        /// protection). Two's complement value.
        /// Resolution is 5 µV/LSB with `AdcRange::Div4` and 1.25 µV/LSB with `AdcRange::Div1`.
        raw_value: i16 = -32768, // i16::MIN
    }

    /// Bus Overvoltage Threshold Register
    register BusOvervoltageThreshold(addr = 0xe, mode = rw, size = 2) {
        /// Reserved bit
        _: u8{1},
        /// The threshold for comparison of the value to detect Bus Overvoltage (overvoltage
        /// protection). Unsigned representation, positive value only. Resolution is 3.125 mV/LSB.
        raw_value: u16{15} = 32767, // u16::MAX
    }

    /// Bus Undervoltage Threshold Register
    register BusUndervoltageThreshold(addr = 0xf, mode = rw, size = 2) {
        /// Reserved bit
        _: u8{1},
        /// The threshold for comparison of the value to detect Bus Undervoltage (undervoltage
        /// protection). Unsigned representation, positive value only. Resolution is 3.125 mV/LSB.
        raw_value: u16{15} = 0,
    }

    /// Temperature Over-Limit Threshold Register
    register TemperatureOverlimitThreshold(addr = 0x10, mode = rw, size = 2) {
        /// The threshold for comparison of the value to detect over temperature measurements.
        /// Two's complement value. The value entered in this field compares directly against the value
        /// from the `Temperature` register to determine if an over temperature condition exists.
        /// Resolution is 7.8125 m°C/LSB.
        raw_value: i16 = i16::MAX => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
    }

    /// Power Over-Limit Threshold Register
    register PowerOverlimitThreshold(addr = 0x11, mode = rw, size = 2) {
        /// The threshold for comparison of the value to detect power over-limit measurements.
        /// Unsigned representation, positive value only. The value entered in this field compares
        /// directly against the value from the `Power` register to determine if an over-power condition
        /// exists. Resolution is 256 * Power LSB.
        raw_value: u16 = 65535, // u16::MAX
    }

    /// Manufacturer Id Register
    register ManufacturerId(addr = 0x3e, mode = r, size = 2) {
        /// Manufacturer Id. Reads `"TI"` in ASCII.
        id: u16 = 0x5449,
    }

    /// Device Id Register
    register DeviceId(addr = 0x3f, mode = r, size = 2) {
        /// Device Id.
        id: u16{12} = 0x228,
        /// Revision.
        revision: u8{4} = 0x1,
    }
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

impl ShuntVoltage {
    /// Read the shunt voltage.
    /// Resolution is 312.5 nV/LSB with `AdcRange::Div4` and 78.125 nV/LSB with `AdcRange::Div1`.
    pub fn read_voltage(&self, adc_range: AdcRange) -> ElectricPotential {
        ElectricPotential::new::<volt>((self.read_raw_value() as i64 * adc_range.factor() as i64) as f64 / 12_800_000.0)
    }
}

impl Current {
    /// Read the current, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_current(&self, current_lsb_na: i64) -> ElectricCurrent {
        ElectricCurrent::new::<ampere>((self.read_raw_value() as i64 * current_lsb_na) as f64 / 1_000_000_000.0)
    }
}

impl Power {
    /// Read the power, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the power register
    pub fn read_power(&self, current_lsb_na: i64) -> f64::Power {
        f64::Power::new::<watt>((self.read_raw_value() as i64 * current_lsb_na * 32) as f64 / (1_000_000_000f64 * 10.0))
    }
}

impl Energy {
    /// Read the energy, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register which is used to derive the nW/LSB for the energy register
    pub fn read_energy(&self, current_lsb_na: i64) -> f64::Energy {
        f64::Energy::new::<joule>(
            (self.read_raw_value() as i64 * current_lsb_na * 512) as f64 / (1_000_000_000f64 * 10.0),
        )
    }
}

impl Charge {
    /// Read the charge, current_lsb_na is the calibrated amount of nA/LSB
    /// for the current register.
    pub fn read_charge(&self, current_lsb_na: i64) -> ElectricCharge {
        ElectricCharge::new::<coulomb>((self.read_raw_value() * current_lsb_na) as f64 / 1_000_000_000f64)
    }
}

impl ShuntOvervoltageThreshold {
    /// Reads the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent
    /// protection).
    pub fn read_voltage_threshold(&self, adc_range: AdcRange) -> ElectricPotential {
        ElectricPotential::new::<volt>((self.read_raw_value() as i32 * adc_range.factor() as i32) as f64 / 800_000f64)
    }

    /// Writes the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent
    /// protection).
    pub fn write_voltage_threshold(
        &mut self,
        voltage_threshold: ElectricPotential,
        adc_range: AdcRange,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = voltage_threshold.get::<volt>();
        let value = ((value * 800_000f64 / adc_range.factor() as f64) as i32).try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent
    /// protection).
    pub fn with_voltage_threshold(
        mut self,
        voltage_threshold: ElectricPotential,
        adc_range: AdcRange,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_voltage_threshold(voltage_threshold, adc_range)?;
        Ok(self)
    }
}

impl ShuntUndervoltageThreshold {
    /// Reads the threshold for comparison of the value to detect Shunt Undervoltage (undercurrent
    /// protection).
    pub fn read_voltage_threshold(&self, adc_range: AdcRange) -> ElectricPotential {
        ElectricPotential::new::<volt>((self.read_raw_value() as i32 * adc_range.factor() as i32) as f64 / 800_000f64)
    }

    /// Writes the threshold for comparison of the value to detect Shunt Undervoltage (undercurrent
    /// protection).
    pub fn write_voltage_threshold(
        &mut self,
        voltage_threshold: ElectricPotential,
        adc_range: AdcRange,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = voltage_threshold.get::<volt>();
        let value = ((value * 800_000f64 / adc_range.factor() as f64) as i32).try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the threshold for comparison of the value to detect Shunt Undervoltage (undercurrent
    /// protection).
    pub fn with_voltage_threshold(
        mut self,
        voltage_threshold: ElectricPotential,
        adc_range: AdcRange,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_voltage_threshold(voltage_threshold, adc_range)?;
        Ok(self)
    }
}

impl BusOvervoltageThreshold {
    /// Reads the threshold for comparison of the value to detect Bus Overvoltage (overvoltage
    /// protection).
    pub fn read_voltage_threshold(&self) -> ElectricPotential {
        ElectricPotential::new::<volt>(self.read_raw_value() as f64 / 320f64)
    }

    /// Writes the threshold for comparison of the value to detect Bus Overvoltage (overvoltage
    /// protection).
    pub fn write_voltage_threshold(
        &mut self,
        voltage_threshold: ElectricPotential,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = voltage_threshold.get::<volt>();
        let value = ((value * 320f64) as i32).try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the threshold for comparison of the value to detect Bus Overvoltage (overvoltage
    /// protection).
    pub fn with_voltage_threshold(
        mut self,
        voltage_threshold: ElectricPotential,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_voltage_threshold(voltage_threshold)?;
        Ok(self)
    }
}

impl BusUndervoltageThreshold {
    /// Reads the threshold for comparison of the value to detect Bus Undervoltage (undervoltage
    /// protection).
    pub fn read_voltage_threshold(&self) -> ElectricPotential {
        ElectricPotential::new::<volt>(self.read_raw_value() as f64 / 320f64)
    }

    /// Writes the threshold for comparison of the value to detect Bus Undervoltage (undervoltage
    /// protection).
    pub fn write_voltage_threshold(
        &mut self,
        voltage_threshold: ElectricPotential,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = voltage_threshold.get::<volt>();
        let value = ((value * 320f64) as i32).try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the threshold for comparison of the value to detect Bus Undervoltage (undervoltage
    /// protection).
    pub fn with_voltage_threshold(
        mut self,
        voltage_threshold: ElectricPotential,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_voltage_threshold(voltage_threshold)?;
        Ok(self)
    }
}

impl PowerOverlimitThreshold {
    /// Reads the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn read_power_limit(&self, current_lsb_na: i64) -> f64::Power {
        f64::Power::new::<watt>(
            (self.read_raw_value() as i64 * current_lsb_na * 32 * 256) as f64 / (1_000_000_000f64 * 10.0),
        )
    }

    /// Writes the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// The passed power will be truncated (rounded down) if necessary.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn write_power_limit(
        &mut self,
        power_limit: f64::Power,
        current_lsb_na: i64,
    ) -> Result<(), core::num::TryFromIntError> {
        let value = power_limit.get::<watt>();
        let value = ((value * 1_000_000_000f64 * 10.0 / (current_lsb_na * 32 * 256) as f64) as i32).try_into()?;
        self.write_raw_value(value);
        Ok(())
    }

    /// Writes the power overlimit threshold in Watt with a resolution of 256 * Power LSB.
    /// The passed power will be truncated (rounded down) if necessary.
    /// current_lsb_na is the calibrated amount of nA/LSB for the current register which is used to
    /// derive the nW/LSB for the power register
    pub fn with_power_limit(
        mut self,
        power_limit: f64::Power,
        current_lsb_na: i64,
    ) -> Result<Self, core::num::TryFromIntError> {
        self.write_power_limit(power_limit, current_lsb_na)?;
        Ok(self)
    }
}
