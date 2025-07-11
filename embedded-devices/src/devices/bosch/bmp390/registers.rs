use bondrewd::{BitfieldEnum, Bitfields};
use embedded_devices_derive::device_register;
use embedded_registers::{i2c::codecs::OneByteRegAddrCodec, register, spi::codecs::standard_codec::StandardCodec};

pub type BMP390SpiCodec = StandardCodec<1, 6, 0, 7, true, 1>;

/// Known chip ids
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Chip {
    BME390 = 0x60,
    /// Unknown chip id
    Invalid(u8),
}

/// The chip identification register.
#[device_register(super::BMP390)]
#[register(
    address = 0x00,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ChipId {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Chip::BME390)]
    pub chip: Chip,
}

/// The chip revision register.
#[device_register(super::BMP390)]
#[register(
    address = 0x01,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct RevId {
    #[register(default = 0x01)]
    pub revision: u8,
}

/// The error condition register.
#[device_register(super::BMP390)]
#[register(
    address = 0x02,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Error {
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Sensor configuration error detected (only working in normal mode). Cleared on read.
    #[register(default = false)]
    pub conf_err: bool,
    /// Command execution failed. Cleared on read.
    #[register(default = false)]
    pub cmd_err: bool,
    /// Fatal error
    #[register(default = false)]
    pub fatal_err: bool,
}

/// The status flag register
#[device_register(super::BMP390)]
#[register(
    address = 0x03,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Status {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Data ready for temperature sensor. It gets reset, when one temperature DATA register is read out.
    #[register(default = false)]
    pub temperature_data_ready: bool,
    /// Data ready for pressure. It gets reset, when one pressure DATA register is read out.
    #[register(default = false)]
    pub pressure_data_ready: bool,
    /// CMD decoder status. True if the command decoder is ready to accept a new command.
    #[register(default = false)]
    pub command_ready: bool,

    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
}

/// This register contains the raw pressure measurement
#[device_register(super::BMP390)]
#[register(
    address = 0x04,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Pressure {
    /// The raw pressure measurement
    #[bondrewd(bit_length = 24)]
    #[register(default = 1 << 23)]
    pub pressure: u32,
}

/// This register contains the raw temperature measurement
#[device_register(super::BMP390)]
#[register(
    address = 0x07,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Temperature {
    /// The raw temperature measurement
    #[bondrewd(bit_length = 24)]
    #[register(default = 1 << 23)]
    pub temperature: u32,
}

/// This register contains the raw sensor time
#[device_register(super::BMP390)]
#[register(
    address = 0x0c,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct SensorTime {
    /// The raw sensor time
    #[bondrewd(bit_length = 24)]
    #[register(default = 0)]
    pub time: u32,
}

/// The event register. Cleared on read.
#[device_register(super::BMP390)]
#[register(
    address = 0x10,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Event {
    #[bondrewd(bit_length = 6, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Set when a serial interface transaction occurs during a pressure or temperature conversion.
    /// Cleared on read.
    #[register(default = false)]
    pub measurement_while_transaction: bool,
    /// Set after device powerup or soft reset.
    /// Cleared on read.
    #[register(default = true)]
    pub power_or_reset: bool,
}

/// The interrupt status register.
/// Cleared on read.
#[device_register(super::BMP390)]
#[register(
    address = 0x11,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptStatus {
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Set when data is ready.
    /// Cleared on read.
    #[register(default = false)]
    pub data_ready: bool,

    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,

    /// Set when the FIFO full interrupt is triggered.
    /// Cleared on read.
    #[register(default = false)]
    pub fifo_full: bool,
    /// Set when the FIFO watermark interrupt is triggered.
    /// Cleared on read.
    #[register(default = false)]
    pub fifo_watermark: bool,
}

// TODO FIXME wrong: https://github.com/Devlyn-Nelson/Bondrewd/issues/12
/// This register contains the raw sensor time
#[device_register(super::BMP390)]
#[register(
    address = 0x12,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct FifoLength {
    #[bondrewd(bit_length = 7, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// The fifo length
    #[bondrewd(bit_length = 9)]
    #[register(default = 0)]
    pub length: u16,
}

/// This register contains the fifo data
#[device_register(super::BMP390)]
#[register(
    address = 0x14,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoData {
    /// The data
    #[register(default = 0)]
    pub data: u8,
}

// TODO FIXME wrong: https://github.com/Devlyn-Nelson/Bondrewd/issues/12
/// This register contains the fifo watermark
#[device_register(super::BMP390)]
#[register(
    address = 0x15,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct FifoWatermark {
    #[bondrewd(bit_length = 7, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// The fifo watermark. Defaults to `0x01` after power-on-reset.
    #[bondrewd(bit_length = 9)]
    #[register(default = 0x01)]
    pub watermark: u16,
}

/// The first fifo config register.
#[device_register(super::BMP390)]
#[register(
    address = 0x17,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoConfig1 {
    #[bondrewd(bit_length = 3, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Store temperature data in FIFO. Defaults to false.
    #[register(default = false)]
    pub temperature_enable: bool,
    /// Store pressure data in FIFO. Defaults to false.
    #[register(default = false)]
    pub pressure_enable: bool,
    /// Store sensortime frame after the last valid data frame. Defaults to false.
    #[register(default = false)]
    pub time_enable: bool,
    /// Stop writing samples into FIFO when FIFO is full. Defaults to true after power-on-reset.
    #[register(default = true)]
    pub stop_on_full: bool,
    /// Enables or disables the fifo. Defaults to false.
    #[register(default = false)]
    pub enable: bool,
}

/// FIFO data source.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum DataSource {
    /// Unfiltered data
    Unfiltered = 0b00,
    /// Filtered data
    Filtered = 0b01,
    /// Unknown data source.
    Invalid(u8),
}

/// The second fifo config register.
#[device_register(super::BMP390)]
#[register(
    address = 0x18,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoConfig2 {
    #[bondrewd(bit_length = 3, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// The data source
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = DataSource::Unfiltered)]
    pub data_source: DataSource,
    /// FIFO downsampling selection for pressure and temperature data, factor is $2^{fifo_subsampling}$.
    #[bondrewd(bit_length = 3)]
    #[register(default = 0x02)]
    pub subsampling: u8,
}

/// The interrupt output mode bit.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum InterruptOutputMode {
    PushPull = 0,
    OpenDrain = 1,
}

/// The interrupt output polarity "level" bit.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum InterruptPolarity {
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// The interrupt control register.
#[device_register(super::BMP390)]
#[register(
    address = 0x19,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptControl {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Enable temperature/pressure ready interrupt for INT pin and INT_STATUS register.
    #[register(default = false)]
    pub data_ready: bool,
    /// Not explained further in datasheet. false = "low", true = "high".
    #[register(default = false)]
    pub ds: bool,
    /// Enable FIFO full interrupt for INT pin and INT_STATUS register.
    #[register(default = false)]
    pub full: bool,
    /// Enable FIFO watermark reached interrupt for INT pin and INT_STATUS register.
    #[register(default = false)]
    pub watermark: bool,
    /// Latching of interrupts for INT pin and INT_STATUS register
    #[register(default = false)]
    pub latching: bool,
    /// Interrupt output polarity
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = InterruptPolarity::ActiveHigh)]
    pub polarity: InterruptPolarity,
    /// Interrupt output mode
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = InterruptOutputMode::PushPull)]
    pub output: InterruptOutputMode,
}

/// The interrupt output polarity "level" bit.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[allow(non_camel_case_types)]
pub enum WatchdogTimerPeriod {
    /// 1.25ms
    T_1_25 = 0,
    /// 40ms
    T_40 = 1,
}

/// The interface control register.
#[device_register(super::BMP390)]
#[register(
    address = 0x1a,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterfaceConfig {
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// The i2c watchdog timer period (backed by NVM).
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = WatchdogTimerPeriod::T_1_25)]
    pub i2c_watchdog_timer_period: WatchdogTimerPeriod,
    /// Whether to enable the i2c watchdog timer (backed by NVM).
    #[register(default = false)]
    pub i2c_watchdog_timer: bool,
    /// Whether to enable the SPI 3-wire interface.
    #[register(default = false)]
    pub spi_3wire: bool,
}

/// Sensor operating mode
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum SensorMode {
    /// Sleep mode is entered by default after power on reset.
    /// In sleep mode, no measurements are performed and power consumption is at a minimum.
    /// All registers are accessible.
    Sleep = 0b00,
    /// In forced mode, a single measurement is performed in accordance to the selected measurement and
    /// filter options. When the measurement is finished, the sensor returns to sleep mode and the
    /// measurement results can be obtained from the data registers. For a next measurement, forced mode
    /// needs to be selected again. Using forced mode is recommended
    /// for applications which require low sampling rate or host-based synchronization.
    Forced = 0b01,
    /// Normal mode comprises an automated perpetual cycling between an (active) measurement period
    /// and an (inactive) standby period. The measurements are performed in accordance to the selected
    /// measurement and filter options.
    Normal = 0b11,
}

/// The power control register.
#[device_register(super::BMP390)]
#[register(
    address = 0x1b,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct PowerControl {
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Controls operating mode of the sensor.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = SensorMode::Sleep)]
    pub sensor_mode: SensorMode,

    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,

    /// Whether to enable the temperature sensor.
    #[register(default = false)]
    pub temperature_enable: bool,
    /// Whether to enable the pressure sensor.
    #[register(default = false)]
    pub pressure_enable: bool,
}

/// Oversampling settings for temperature and pressure measurements.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Oversampling {
    /// Disables oversampling.
    X_1 = 0b000,
    /// Configures 2x oversampling.
    X_2 = 0b001,
    /// Configures 4x oversampling.
    X_4 = 0b010,
    /// Configures 8x oversampling.
    X_8 = 0b011,
    /// Configures 16x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X_16 = 0b100,
    /// Configures 32x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X_32 = 0b101,
    /// Unknown oversampling setting.
    Invalid(u8),
}

impl Oversampling {
    /// Returns the oversampling factor (1 for X_1, 32 for X_32)
    pub fn factor(&self) -> u32 {
        match self {
            Oversampling::X_1 => 1,
            Oversampling::X_2 => 2,
            Oversampling::X_4 => 4,
            Oversampling::X_8 => 8,
            Oversampling::X_16 => 16,
            Oversampling::X_32 => 32,
            Oversampling::Invalid(_) => 0,
        }
    }
}

/// The oversampling control register.
#[device_register(super::BMP390)]
#[register(
    address = 0x1c,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OversamplingControl {
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Controls oversampling of temperature data.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X_1)]
    pub temperature_oversampling: Oversampling,
    /// Controls oversampling of pressure data.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X_4)]
    pub pressure_oversampling: Oversampling,
}

/// Output data rate
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum DataRate {
    /// prescaler = 1, data rate = 200Hz, sampling period = 5 ms
    F_200 = 0x00,
    /// prescaler = 2, data rate = 100Hz, sampling period = 10 ms
    F_100 = 0x01,
    /// prescaler = 4, data rate = 50Hz, sampling period = 20 ms
    F_50 = 0x02,
    /// prescaler = 8, data rate = 25Hz, sampling period = 40 ms
    F_25 = 0x03,
    /// prescaler = 16, data rate = 25/2Hz, sampling period = 80 ms
    F_12_5 = 0x04,
    /// prescaler = 32, data rate = 25/4Hz, sampling period = 160 ms
    F_6_25 = 0x05,
    /// prescaler = 64, data rate = 25/8Hz, sampling period = 320 ms
    F_3_1 = 0x06,
    /// prescaler = 127, data rate = 25/16Hz, sampling period = 640 ms
    F_1_5 = 0x07,
    /// prescaler = 256, data rate = 25/32Hz, sampling period = 1.280 s
    F_0_78 = 0x08,
    /// prescaler = 512, data rate = 25/64Hz, sampling period = 2.560 s
    F_0_39 = 0x09,
    /// prescaler = 1024, data rate = 25/128Hz, sampling period = 5.120 s
    F_0_2 = 0x0A,
    /// prescaler = 2048, data rate = 25/256Hz, sampling period = 10.24 s
    F_0_1 = 0x0B,
    /// prescaler = 4096, data rate = 25/512Hz, sampling period = 20.48 s
    F_0_05 = 0x0C,
    /// prescaler = 8192, data rate = 25/1024Hz, sampling period = 40.96 s
    F_0_02 = 0x0D,
    /// prescaler = 16384, data rate = 25/2048Hz, sampling period = 81.92 s
    F_0_01 = 0x0E,
    /// prescaler = 32768, data rate = 25/4096Hz, sampling period = 163.84 s
    F_0_006 = 0x0F,
    /// prescaler = 65536, data rate = 25/8192Hz, sampling period = 327.68 s
    F_0_003 = 0x10,
    /// prescaler = 131072, data rate = 25/16384Hz, sampling period = 655.36 s
    F_0_0015 = 0x11,
    /// Unknown oversampling setting.
    Invalid(u8),
}

impl DataRate {
    /// Returns the interval of this data rate in microseconds.
    pub fn interval_us(&self) -> u32 {
        match self {
            DataRate::F_200 => 5_000,
            DataRate::F_100 => 10_000,
            DataRate::F_50 => 20_000,
            DataRate::F_25 => 40_000,
            DataRate::F_12_5 => 80_000,
            DataRate::F_6_25 => 160_000,
            DataRate::F_3_1 => 320_000,
            DataRate::F_1_5 => 640_000,
            DataRate::F_0_78 => 1_280_000,
            DataRate::F_0_39 => 2_560_000,
            DataRate::F_0_2 => 5_120_000,
            DataRate::F_0_1 => 10_240_000,
            DataRate::F_0_05 => 20_480_000,
            DataRate::F_0_02 => 40_960_000,
            DataRate::F_0_01 => 81_920_000,
            DataRate::F_0_006 => 163_840_000,
            DataRate::F_0_003 => 327_680_000,
            DataRate::F_0_0015 => 655_360_000,
            DataRate::Invalid(_) => panic!("Cannot get interval of invalid DataRate"),
        }
    }
}

/// The data rate control register.
#[device_register(super::BMP390)]
#[register(
    address = 0x1d,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct DataRateControl {
    #[bondrewd(bit_length = 3, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Controls the output data rate
    #[bondrewd(enum_primitive = "u8", bit_length = 5)]
    #[register(default = DataRate::F_200)]
    pub data_rate: DataRate,
}

/// Lowpass filter settings for pressure and temperature values.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IIRFilter {
    /// Disables the IIR filter (default).
    Disabled = 0b000,
    /// Sets the IIR filter coefficient to 1.
    Coefficient1 = 0b001,
    /// Sets the IIR filter coefficient to 3.
    Coefficient3 = 0b010,
    /// Sets the IIR filter coefficient to 7.
    Coefficient7 = 0b011,
    /// Sets the IIR filter coefficient to 15.
    Coefficient15 = 0b100,
    /// Sets the IIR filter coefficient to 31.
    Coefficient31 = 0b101,
    /// Sets the IIR filter coefficient to 63.
    Coefficient63 = 0b110,
    /// Sets the IIR filter coefficient to 127.
    Coefficient127 = 0b111,
}

/// The general config register.
#[device_register(super::BMP390)]
#[register(
    address = 0x1f,
    mode = "rw",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Config {
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// Controls the output data rate
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = IIRFilter::Disabled)]
    pub iir_filter: IIRFilter,

    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
}

/// Device-internal trimming coefficients (calibration registers)
#[device_register(super::BMP390)]
#[register(
    address = 0x31,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 21)]
pub struct TrimmingCoefficients {
    pub par_t1: u16,
    pub par_t2: u16,
    pub par_t3: i8,
    pub par_p1: i16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i8,
    pub par_p5: u16,
    pub par_p6: u16,
    pub par_p7: i8,
    pub par_p8: i8,
    pub par_p9: i16,
    pub par_p10: i8,
    pub par_p11: i8,
}

/// Available commands
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Cmd {
    /// Does nothing.
    Nop = 0x00,
    /// Clears all data in the FIFO, does not change FIFO_CONFIG registers.
    FlushFifo = 0xb0,
    /// Triggers a reset, all user configuration settings are overwritten with their default state.
    Reset = 0xb6,
    /// Invalid command
    Invalid(u8),
}

/// The command register.
#[device_register(super::BMP390)]
#[register(
    address = 0x7e,
    mode = "w",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Command {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Cmd::Nop)]
    pub command: Cmd,
}

/// Burst register read of pressure and temperature
#[device_register(super::BMP390)]
#[register(
    address = 0x04,
    mode = "r",
    i2c_codec = "OneByteRegAddrCodec",
    spi_codec = "BMP390SpiCodec"
)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct BurstMeasurements {
    #[bondrewd(struct_size = 3)]
    pub pressure: PressureBitfield,
    #[bondrewd(struct_size = 3)]
    pub temperature: TemperatureBitfield,
}
