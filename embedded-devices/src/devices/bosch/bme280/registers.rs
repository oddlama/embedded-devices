use bondrewd::{BitfieldEnum, Bitfields};
use embedded_devices_derive::device_register;
use embedded_registers::{i2c::codecs::OneByteRegAddrCodec, register, spi::codecs::simple_codec::SimpleCodec};

pub type BME280SpiCodec = SimpleCodec<1, 6, 0, 7, true, 0>;
pub type BME280I2cCodec = OneByteRegAddrCodec;

/// Known chip ids
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Chip {
    /// A BMP280 production sample
    BMP280Sample1 = 0x56,
    /// A BMP280 production sample
    BMP280Sample2 = 0x57,
    /// A mass-produced BMP280
    BMP280 = 0x58,
    /// A mass-produced BME280
    BME280 = 0x60,
    /// Unknown chip id
    Invalid(u8),
}

/// The chip identification number. This number can
/// be read as soon as the device finished the power-on-reset.
#[device_register(super::BME280Common)]
#[register(
    address = 0xd0,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Id {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Chip::Invalid(0))]
    pub chip: Chip,
}

#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum ResetMagic {
    /// Magic value to reset the device
    Reset = 0xb6,
    /// Invalid reset magic
    Invalid(u8),
}

/// The reset register. If the value 0xB6 is written to the register,
/// the device is reset using the complete power-on-reset procedure.
/// Writing other values than 0xB6 has no effect.
#[device_register(super::BME280Common)]
#[register(
    address = 0xe0,
    mode = "w",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Reset {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = ResetMagic::Reset)]
    pub magic: ResetMagic,
}

/// Oversampling settings for temperature, pressure, and humidity measurements.
/// See sections 3.4ff of the manual for measurement flow and recommended values.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Oversampling {
    /// Disables this output. Measurement data will be returned as 0x80000.
    Disabled = 0b000,
    /// Disables oversampling.
    /// Without IIR filtering, this sets the resolution of temperature and pressure measurements
    /// to 16 bits.
    X_1 = 0b001,
    /// Configures 2x oversampling.
    /// This increases the resolution of temperature and pressure measurements to 17 bits without
    /// IIR filtering.
    X_2 = 0b010,
    /// Configures 4x oversampling.
    /// This increases the resolution of temperature and pressure measurements to 18 bits without
    /// IIR filtering.
    X_4 = 0b011,
    /// Configures 8x oversampling.
    /// This increases the resolution of temperature and pressure measurements to 19 bits without
    /// IIR filtering.
    X_8 = 0b100,
    /// Configures 16x oversampling.
    /// This increases the resolution of temperature and pressure measurements to 20 bits,
    /// regardless of IIR filtering.
    X_16 = 0b101,
    /// Unknown oversampling setting.
    Invalid(u8),
}

impl Oversampling {
    /// Returns the oversampling factor (1 for X_1, 16 for X_16)
    pub fn factor(&self) -> u32 {
        match self {
            Oversampling::Disabled => 0,
            Oversampling::X_1 => 1,
            Oversampling::X_2 => 2,
            Oversampling::X_4 => 4,
            Oversampling::X_8 => 8,
            Oversampling::X_16 => 16,
            Oversampling::Invalid(_) => 0,
        }
    }
}

/// The humidity control register. Changes to this register only become effective
/// after a write to the ControlMeasurement register!
#[device_register(super::BME280Common)]
#[register(
    address = 0xf2,
    mode = "rw",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ControlHumidity {
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Controls oversampling of humidity data.
    /// The default is 1x, i.e., no oversampling.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X_1)]
    pub oversampling: Oversampling,
}

/// The status register.
#[device_register(super::BME280Common)]
#[register(
    address = 0xf3,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Status {
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Automatically set to `1` whenever a conversion is running and back to `0` when the results have been transferred to the data registers.
    #[register(default = false)]
    pub measuring: bool,

    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,

    /// Automatically set to `1` when the NVM data is being copied to image registers and back to `0` when the
    /// copying is done. The data are copied at power-on-reset and before every conversion.
    #[register(default = false)]
    pub update: bool,
}

/// Sensor operating mode
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum SensorMode {
    /// Sleep mode is entered by default after power on reset. In sleep mode, no measurements are
    /// performed and power consumption is at a minimum. All registers are accessible.
    /// There are no special restrictions on interface timings.
    Sleep = 0b00,
    /// In forced mode, a single measurement is performed in accordance to the selected measurement and
    /// filter options. When the measurement is finished, the sensor returns to sleep mode and the
    /// measurement results can be obtained from the data registers. For a next measurement, forced mode
    /// needs to be selected again. Using forced mode is recommended
    /// for applications which require low sampling rate or host-based synchronization.
    Forced = 0b01,
    /// Normal mode comprises an automated perpetual cycling between an (active) measurement period
    /// and an (inactive) standby period. The measurements are performed in accordance to the selected
    /// measurement and filter options. The standby time is determined by the [`StandbyTime`] setting
    /// in the [`Config`] register and can be set to between 0.5 and 1000 ms.
    Normal = 0b11,
}

/// The measurement control register sets the pressure and temperature
/// data acquisition options of the device. The register needs to be written
/// after changing [`ControlHumidity`] for those changes to become effective.
#[device_register(super::BME280Common)]
#[register(
    address = 0xf4,
    mode = "rw",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ControlMeasurement {
    /// Controls oversampling of temperature data.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::Disabled)]
    pub temperature_oversampling: Oversampling,
    /// Controls oversampling of pressure data.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::Disabled)]
    pub pressure_oversampling: Oversampling,
    /// Controls operating mode of the sensor.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = SensorMode::Sleep)]
    pub sensor_mode: SensorMode,
}

/// The standby time between measurements in [`SensorMode::Normal`].
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum StandbyTime {
    /// 0.5ms
    T_0_5 = 0b000,
    /// 62.5ms
    T_62_5 = 0b001,
    /// 125ms
    T_125 = 0b010,
    /// 250ms
    T_250 = 0b011,
    /// 500ms
    T_500 = 0b100,
    /// 1000ms
    T_1000 = 0b101,
    /// 10ms (BME280 only)
    T_10 = 0b110,
    /// 20ms (BME280 only)
    T_20 = 0b111,
    //TODO split this up into a bmx280 crate for common stuff.
    //TODO /// 2000ms (BMP280 only)
    //TODO T_2000 = 0b110,
    //TODO /// 4000ms (BMP280 only)
    //TODO T_4000 = 0b111,
}

/// Lowpass filter settings for pressure and temperature values.
/// Enabling any filter option increases the resolution of the
/// respective measured quantity to 20 bits.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IIRFilter {
    /// Disables the IIR filter (default).
    /// The resolution of pressure and temperature measurements is dictated by their respective
    /// oversampling settings.
    Disabled = 0b000,
    /// Sets the IIR filter coefficient to 2.
    Coefficient2 = 0b001,
    /// Sets the IIR filter coefficient to 4.
    Coefficient4 = 0b010,
    /// Sets the IIR filter coefficient to 8.
    Coefficient8 = 0b011,
    /// Sets the IIR filter coefficient to 16.
    Coefficient16 = 0b100,
}

/// The config register sets the rate, filter and interface options of the device.
/// Writes to this register in [`SensorMode::Normal`] may be ignored.
/// In [`SensorMode::Sleep`] writes are not ignored.
#[device_register(super::BME280Common)]
#[register(
    address = 0xf4,
    mode = "rw",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Config {
    /// Controls inactive duration t_standby in [`SensorMode::Normal`].
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = StandbyTime::T_0_5)]
    pub standby_time: StandbyTime,
    /// Controls the time constant of the IIR filter.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = IIRFilter::Disabled)]
    pub filter: IIRFilter,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved0: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    /// Whether to enable the SPI 3-wire interface.
    #[register(default = false)]
    pub spi_3wire: bool,
}

/// Device-internal calibration registers (section 1)
#[device_register(super::BME280Common)]
#[register(
    address = 0x88,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 26)]
pub struct TrimmingParameters1 {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,
    pub dig_h1: u8,
}

/// Device-internal calibration registers (section 2)
#[device_register(super::BME280Common)]
#[register(
    address = 0xe1,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 7)]
pub struct TrimmingParameters2 {
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4_msb: i8,
    pub dig_h5_lsn_h4_lsn: i8,
    pub dig_h5_msb: i8,
    pub dig_h6: i8,
}

/// This register contains the raw pressure measurement
#[device_register(super::BME280Common)]
#[register(
    address = 0xf7,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct Pressure {
    /// The raw pressure measurement
    #[bondrewd(bit_length = 20)]
    #[register(default = 1 << 19)]
    pub pressure: u32,

    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,
}

/// This register contains the raw temperature measurement
#[device_register(super::BME280Common)]
#[register(
    address = 0xfa,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 3)]
pub struct Temperature {
    /// The raw temperature measurement
    #[bondrewd(bit_length = 20)]
    #[register(default = 1 << 19)]
    pub temperature: u32,

    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,
}

/// This register contains the raw humidity measurement
#[device_register(super::BME280Common)]
#[register(
    address = 0xfd,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Humidity {
    /// The raw humidity measurement
    #[register(default = 1 << 15)]
    pub humidity: u16,
}

/// Burst register read of pressure and temperature
#[device_register(super::BME280Common)]
#[register(
    address = 0xf7,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct BurstMeasurementsPT {
    #[bondrewd(struct_size = 3)]
    pub pressure: PressureBitfield,
    #[bondrewd(struct_size = 3)]
    pub temperature: TemperatureBitfield,
}

/// Burst register read of pressure, temperature and humidity
#[device_register(super::BME280Common)]
#[register(
    address = 0xf7,
    mode = "r",
    i2c_codec = "BME280I2cCodec",
    spi_codec = "BME280SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 8)]
pub struct BurstMeasurementsPTH {
    #[bondrewd(struct_size = 3)]
    pub pressure: PressureBitfield,
    #[bondrewd(struct_size = 3)]
    pub temperature: TemperatureBitfield,
    #[bondrewd(struct_size = 2)]
    pub humidity: HumidityBitfield,
}
