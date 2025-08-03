use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::{i2c::codecs::OneByteRegAddrCodec, spi::codecs::standard_codec::StandardCodec};

pub type BMP390SpiCodec = StandardCodec<1, 6, 0, 7, true, 1>;
pub type BMP390I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = BMP390I2cCodec,
        spi_codec = BMP390SpiCodec,
    }

    register_devices [ super::BMP390 ]

    /// Known chip ids
    enum Chip: u8{8} {
        /// BME390 chip
        0x60 BME390,
        /// Unknown chip id
        _ Invalid(u8),
    }

    /// FIFO data source
    enum DataSource: u8{2} {
        /// Unfiltered data
        0b00 Unfiltered,
        /// Filtered data
        0b01 Filtered,
        /// Unknown data source
        _ Invalid(u8),
    }

    /// The interrupt output mode bit
    enum InterruptOutputMode: u8{1} {
        0 PushPull,
        1 OpenDrain,
    }

    /// The interrupt output polarity "level" bit
    enum InterruptPolarity: u8{1} {
        0 ActiveLow,
        1 ActiveHigh,
    }

    /// Watchdog timer period
    #[allow(non_camel_case_types)]
    enum WatchdogTimerPeriod: u8{1} {
        /// 1.25ms
        0 T_1_25,
        /// 40ms
        1 T_40,
    }

    /// Sensor operating mode
    enum SensorMode: u8{2} {
        /// Sleep mode is entered by default after power on reset.
        /// In sleep mode, no measurements are performed and power consumption is at a minimum.
        /// All registers are accessible.
        0b00 Sleep,
        /// In forced mode, a single measurement is performed in accordance to the selected measurement and
        /// filter options. When the measurement is finished, the sensor returns to sleep mode and the
        /// measurement results can be obtained from the data registers. For a next measurement, forced mode
        /// needs to be selected again. Using forced mode is recommended
        /// for applications which require low sampling rate or host-based synchronization.
        0b01 Forced,
        /// Normal mode comprises an automated perpetual cycling between an (active) measurement period
        /// and an (inactive) standby period. The measurements are performed in accordance to the selected
        /// measurement and filter options.
        0b11 Normal,
        /// Invalid sensor mode
        _ Invalid(u8),
    }

    /// Oversampling settings for temperature and pressure measurements
    #[allow(non_camel_case_types)]
    enum Oversampling: u8{3} {
        /// Disables oversampling
        0b000 X_1,
        /// Configures 2x oversampling
        0b001 X_2,
        /// Configures 4x oversampling
        0b010 X_4,
        /// Configures 8x oversampling
        0b011 X_8,
        /// Configures 16x oversampling.
        /// If this is used as the pressure oversampling rate, it is recommended to also use
        /// at least 2x temperature oversampling to get accurate compensation.
        0b100 X_16,
        /// Configures 32x oversampling.
        /// If this is used as the pressure oversampling rate, it is recommended to also use
        /// at least 2x temperature oversampling to get accurate compensation.
        0b101 X_32,
        /// Unknown oversampling setting
        _ Invalid(u8),
    }

    /// Output data rate
    #[allow(non_camel_case_types)]
    enum DataRate: u8{5} {
        /// prescaler = 1, data rate = 200Hz, sampling period = 5 ms
        0x00 F_200,
        /// prescaler = 2, data rate = 100Hz, sampling period = 10 ms
        0x01 F_100,
        /// prescaler = 4, data rate = 50Hz, sampling period = 20 ms
        0x02 F_50,
        /// prescaler = 8, data rate = 25Hz, sampling period = 40 ms
        0x03 F_25,
        /// prescaler = 16, data rate = 25/2Hz, sampling period = 80 ms
        0x04 F_12_5,
        /// prescaler = 32, data rate = 25/4Hz, sampling period = 160 ms
        0x05 F_6_25,
        /// prescaler = 64, data rate = 25/8Hz, sampling period = 320 ms
        0x06 F_3_1,
        /// prescaler = 127, data rate = 25/16Hz, sampling period = 640 ms
        0x07 F_1_5,
        /// prescaler = 256, data rate = 25/32Hz, sampling period = 1.280 s
        0x08 F_0_78,
        /// prescaler = 512, data rate = 25/64Hz, sampling period = 2.560 s
        0x09 F_0_39,
        /// prescaler = 1024, data rate = 25/128Hz, sampling period = 5.120 s
        0x0A F_0_2,
        /// prescaler = 2048, data rate = 25/256Hz, sampling period = 10.24 s
        0x0B F_0_1,
        /// prescaler = 4096, data rate = 25/512Hz, sampling period = 20.48 s
        0x0C F_0_05,
        /// prescaler = 8192, data rate = 25/1024Hz, sampling period = 40.96 s
        0x0D F_0_02,
        /// prescaler = 16384, data rate = 25/2048Hz, sampling period = 81.92 s
        0x0E F_0_01,
        /// prescaler = 32768, data rate = 25/4096Hz, sampling period = 163.84 s
        0x0F F_0_006,
        /// prescaler = 65536, data rate = 25/8192Hz, sampling period = 327.68 s
        0x10 F_0_003,
        /// prescaler = 131072, data rate = 25/16384Hz, sampling period = 655.36 s
        0x11 F_0_001_5,
        /// Unknown data rate setting
        _ Invalid(u8),
    }

    /// Lowpass filter settings for pressure and temperature values
    enum IIRFilter: u8{3} {
        /// Disables the IIR filter (default)
        0b000 Disabled,
        /// Sets the IIR filter coefficient to 1
        0b001 Coefficient1,
        /// Sets the IIR filter coefficient to 3
        0b010 Coefficient3,
        /// Sets the IIR filter coefficient to 7
        0b011 Coefficient7,
        /// Sets the IIR filter coefficient to 15
        0b100 Coefficient15,
        /// Sets the IIR filter coefficient to 31
        0b101 Coefficient31,
        /// Sets the IIR filter coefficient to 63
        0b110 Coefficient63,
        /// Sets the IIR filter coefficient to 127
        0b111 Coefficient127,
    }

    /// Available commands
    enum Cmd: u8{8} {
        /// Does nothing
        0x00 Nop,
        /// Clears all data in the FIFO, does not change FIFO_CONFIG registers
        0xb0 FlushFifo,
        /// Triggers a reset, all user configuration settings are overwritten with their default state
        0xb6 Reset,
        /// Invalid command
        _ Invalid(u8),
    }

    /// The chip identification register
    register ChipId(addr = 0x00, mode = r, size = 1) {
        chip: Chip = Chip::BME390,
    }

    /// The chip revision register
    register RevId(addr = 0x01, mode = r, size = 1) {
        revision: u8 = 0x01,
    }

    /// The error condition register
    register Error(addr = 0x02, mode = r, size = 1) {
        /// Reserved bits
        _: u8{5},
        /// Sensor configuration error detected (only working in normal mode). Cleared on read.
        conf_err: bool = false,
        /// Command execution failed. Cleared on read.
        cmd_err: bool = false,
        /// Fatal error
        fatal_err: bool = false,
    }

    /// The status flag register
    register Status(addr = 0x03, mode = r, size = 1) {
        /// Reserved bit
        _: u8{1},
        /// Data ready for temperature sensor. It gets reset, when one temperature DATA register is read out.
        temperature_data_ready: bool = false,
        /// Data ready for pressure. It gets reset, when one pressure DATA register is read out.
        pressure_data_ready: bool = false,
        /// CMD decoder status. True if the command decoder is ready to accept a new command.
        command_ready: bool = false,
        /// Reserved bits
        _: u8{4},
    }

    /// This register contains the raw pressure measurement
    register Pressure(addr = 0x04, mode = r, size = 3) {
        /// The raw pressure measurement
        value: u32{24,le} = 1 << 23,
    }

    /// This register contains the raw temperature measurement
    register Temperature(addr = 0x07, mode = r, size = 3) {
        /// The raw temperature measurement
        value: u32{24,le} = 1 << 23,
    }

    /// This register contains the raw sensor time
    register SensorTime(addr = 0x0c, mode = r, size = 3) {
        /// The raw sensor time
        time: u32{24,le} = 0,
    }

    /// The event register. Cleared on read.
    register Event(addr = 0x10, mode = r, size = 1) {
        /// Reserved bits
        _: u8{6},
        /// Set when a serial interface transaction occurs during a pressure or temperature conversion.
        /// Cleared on read.
        measurement_while_transaction: bool = false,
        /// Set after device powerup or soft reset.
        /// Cleared on read.
        power_or_reset: bool = true,
    }

    /// The interrupt status register.
    /// Cleared on read.
    register InterruptStatus(addr = 0x11, mode = r, size = 1) {
        /// Reserved bits
        _: u8{4},
        /// Set when data is ready.
        /// Cleared on read.
        data_ready: bool = false,
        /// Reserved bit
        _: u8{1},
        /// Set when the FIFO full interrupt is triggered.
        /// Cleared on read.
        fifo_full: bool = false,
        /// Set when the FIFO watermark interrupt is triggered.
        /// Cleared on read.
        fifo_watermark: bool = false,
    }

    /// This register contains the fifo length
    register FifoLength(addr = 0x12, mode = r, size = 2) {
        /// The fifo length
        length: u16[15, 0..8] = 0,
        /// Reserved bits
        _: u8[8..15],
    }

    /// This register contains the fifo data
    register FifoData(addr = 0x14, mode = r, size = 1) {
        /// The data
        data: u8 = 0,
    }

    /// This register contains the fifo watermark
    register FifoWatermark(addr = 0x15, mode = rw, size = 2) {
        /// The fifo watermark.
        watermark: u16[15, 0..8] = 0x1,
        /// Reserved bits
        _: u8[8..15],
    }

    /// The first fifo config register
    register FifoConfig1(addr = 0x17, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{3},
        /// Store temperature data in FIFO.
        temperature_enable: bool = false,
        /// Store pressure data in FIFO.
        pressure_enable: bool = false,
        /// Store sensortime frame after the last valid data frame.
        time_enable: bool = false,
        /// Stop writing samples into FIFO when FIFO is full.
        stop_on_full: bool = true,
        /// Enables or disables the fifo.
        enable: bool = false,
    }

    /// The second fifo config register
    register FifoConfig2(addr = 0x18, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{3},
        /// The data source
        data_source: DataSource = DataSource::Unfiltered,
        /// FIFO downsampling selection for pressure and temperature data, factor is 2^fifo_subsampling.
        subsampling: u8{3} = 0x02,
    }

    /// The interrupt control register
    register InterruptControl(addr = 0x19, mode = rw, size = 1) {
        /// Reserved bit
        _: u8{1},
        /// Enable temperature/pressure ready interrupt for INT pin and INT_STATUS register.
        data_ready: bool = false,
        /// Not explained further in datasheet. false = "low", true = "high".
        ds: bool = false,
        /// Enable FIFO full interrupt for INT pin and INT_STATUS register.
        full: bool = false,
        /// Enable FIFO watermark reached interrupt for INT pin and INT_STATUS register.
        watermark: bool = false,
        /// Latching of interrupts for INT pin and INT_STATUS register
        latching: bool = false,
        /// Interrupt output polarity
        polarity: InterruptPolarity = InterruptPolarity::ActiveHigh,
        /// Interrupt output mode
        output: InterruptOutputMode = InterruptOutputMode::PushPull,
    }

    /// The interface control register
    register InterfaceConfig(addr = 0x1a, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{5},
        /// The i2c watchdog timer period (backed by NVM).
        i2c_watchdog_timer_period: WatchdogTimerPeriod = WatchdogTimerPeriod::T_1_25,
        /// Whether to enable the i2c watchdog timer (backed by NVM).
        i2c_watchdog_timer: bool = false,
        /// Whether to enable the SPI 3-wire interface.
        spi_3wire: bool = false,
    }

    /// The power control register
    register PowerControl(addr = 0x1b, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{2},
        /// Controls operating mode of the sensor.
        sensor_mode: SensorMode = SensorMode::Sleep,
        /// Reserved bits
        _: u8{2},
        /// Whether to enable the temperature sensor.
        temperature_enable: bool = false,
        /// Whether to enable the pressure sensor.
        pressure_enable: bool = false,
    }

    /// The oversampling control register
    register OversamplingControl(addr = 0x1c, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{2},
        /// Controls oversampling of temperature data.
        temperature_oversampling: Oversampling = Oversampling::X_1,
        /// Controls oversampling of pressure data.
        pressure_oversampling: Oversampling = Oversampling::X_4,
    }

    /// The data rate control register
    register DataRateControl(addr = 0x1d, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{3},
        /// Controls the output data rate
        data_rate: DataRate = DataRate::F_200,
    }

    /// The general config register
    register Config(addr = 0x1f, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{4},
        /// Controls the IIR filter
        iir_filter: IIRFilter = IIRFilter::Disabled,
        /// Reserved bit
        _: u8{1},
    }

    /// Device-internal trimming coefficients (calibration registers)
    register TrimmingCoefficients(addr = 0x31, mode = r, size = 21) {
        par_t1: u16{le},
        par_t2: u16{le},
        par_t3: i8,
        par_p1: i16{le},
        par_p2: i16{le},
        par_p3: i8,
        par_p4: i8,
        par_p5: u16{le},
        par_p6: u16{le},
        par_p7: i8,
        par_p8: i8,
        par_p9: i16{le},
        par_p10: i8,
        par_p11: i8,
    }

    /// The command register
    register Command(addr = 0x7e, mode = w, size = 1) {
        command: Cmd = Cmd::Nop,
    }

    /// Burst register read of pressure and temperature
    register BurstMeasurements(addr = 0x04, mode = r, size = 6) {
        pressure: PressureUnpacked,
        temperature: TemperatureUnpacked,
    }
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

impl DataRate {
    /// Returns the interval of this data rate in microseconds.
    pub fn interval_us(&self) -> u32 {
        match self {
            DataRate::Invalid(_) => 0,
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
            DataRate::F_0_001_5 => 655_360_000,
        }
    }
}
