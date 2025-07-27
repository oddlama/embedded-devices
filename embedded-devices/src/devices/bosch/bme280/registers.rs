use crate::devices::bosch::bme280::BME280CommonRegister;
use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::{i2c::codecs::OneByteRegAddrCodec, spi::codecs::standard_codec::StandardCodec};

pub type BME280SpiCodec = StandardCodec<1, 6, 0, 7, true, 0>;
pub type BME280I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = BME280I2cCodec,
        spi_codec = BME280SpiCodec,
    }

    register_devices [ BME280Common ]

    /// Known chip ids
    enum Chip: u8{8} {
        /// A BMP280 production sample
        0x56 BMP280Sample1,
        /// A BMP280 production sample
        0x57 BMP280Sample2,
        /// A mass-produced BMP280
        0x58 BMP280,
        /// A mass-produced BME280
        0x60 BME280,
        /// Unknown chip id
        _ Invalid(u8),
    }

    /// Reset magic values
    enum ResetMagic: u8{8} {
        /// Magic value to reset the device
        0xb6 Reset,
        /// Invalid reset magic
        _ Invalid(u8),
    }

    /// Oversampling settings for temperature, pressure, and humidity measurements.
    /// See sections 3.4ff of the manual for measurement flow and recommended values.
    #[allow(non_camel_case_types)]
    enum Oversampling: u8{3} {
        /// Disables this output. Measurement data will be returned as 0x80000.
        0b000 Disabled,
        /// Disables oversampling.
        /// Without IIR filtering, this sets the resolution of temperature and pressure measurements
        /// to 16 bits.
        0b001 X_1,
        /// Configures 2x oversampling.
        /// This increases the resolution of temperature and pressure measurements to 17 bits without
        /// IIR filtering.
        0b010 X_2,
        /// Configures 4x oversampling.
        /// This increases the resolution of temperature and pressure measurements to 18 bits without
        /// IIR filtering.
        0b011 X_4,
        /// Configures 8x oversampling.
        /// This increases the resolution of temperature and pressure measurements to 19 bits without
        /// IIR filtering.
        0b100 X_8,
        /// Configures 16x oversampling.
        /// This increases the resolution of temperature and pressure measurements to 20 bits,
        /// regardless of IIR filtering.
        0b101 X_16,
        /// Unknown oversampling setting.
        _ Invalid(u8),
    }

    /// Sensor operating mode
    enum SensorMode: u8{2} {
        /// Sleep mode is entered by default after power on reset. In sleep mode, no measurements are
        /// performed and power consumption is at a minimum. All registers are accessible.
        /// There are no special restrictions on interface timings.
        0b00 Sleep,
        /// In forced mode, a single measurement is performed in accordance to the selected measurement and
        /// filter options. When the measurement is finished, the sensor returns to sleep mode and the
        /// measurement results can be obtained from the data registers. For a next measurement, forced mode
        /// needs to be selected again. Using forced mode is recommended
        /// for applications which require low sampling rate or host-based synchronization.
        0b01 Forced,
        /// Normal mode comprises an automated perpetual cycling between an (active) measurement period
        /// and an (inactive) standby period. The measurements are performed in accordance to the selected
        /// measurement and filter options. The standby time is determined by the standby_time setting
        /// in the Config register and can be set to between 0.5 and 1000 ms.
        0b11 Normal,
        /// Invalid sensor mode
        _ Invalid(u8),
    }

    /// The standby time between measurements in Normal sensor mode.
    #[allow(non_camel_case_types)]
    enum StandbyTime: u8{3} {
        /// 0.5ms
        0b000 T_0_5,
        /// 62.5ms
        0b001 T_62_5,
        /// 125ms
        0b010 T_125,
        /// 250ms
        0b011 T_250,
        /// 500ms
        0b100 T_500,
        /// 1000ms
        0b101 T_1000,
        /// 10ms (BME280 only)
        0b110 T_10,
        /// 20ms (BME280 only)
        0b111 T_20,
    }

    /// Lowpass filter settings for pressure and temperature values.
    /// Enabling any filter option increases the resolution of the
    /// respective measured quantity to 20 bits.
    enum IIRFilter: u8{3} {
        /// Disables the IIR filter (default).
        /// The resolution of pressure and temperature measurements is dictated by their respective
        /// oversampling settings.
        0b000 Disabled,
        /// Sets the IIR filter coefficient to 2.
        0b001 Coefficient2,
        /// Sets the IIR filter coefficient to 4.
        0b010 Coefficient4,
        /// Sets the IIR filter coefficient to 8.
        0b011 Coefficient8,
        /// Sets the IIR filter coefficient to 16.
        0b100 Coefficient16,
        /// Invalid coefficient
        _ Invalid(u8),
    }

    /// The chip identification number. This number can
    /// be read as soon as the device finished the power-on-reset.
    register Id(addr = 0xd0, mode = r, size = 1) {
        chip: Chip = Chip::Invalid(0),
    }

    /// The reset register. If the value 0xB6 is written to the register,
    /// the device is reset using the complete power-on-reset procedure.
    /// Writing other values than 0xB6 has no effect.
    register Reset(addr = 0xe0, mode = w, size = 1) {
        magic: ResetMagic = ResetMagic::Reset,
    }

    /// The humidity control register. Changes to this register only become effective
    /// after a write to the ControlMeasurement register!
    register ControlHumidity(addr = 0xf2, mode = rw, size = 1) {
        /// Reserved bits
        _: u8{5},
        /// Controls oversampling of humidity data.
        /// The default is 1x, i.e., no oversampling.
        oversampling: Oversampling = Oversampling::X_1,
    }

    /// The status register.
    register Status(addr = 0xf3, mode = r, size = 1) {
        /// Reserved bits
        _: u8{4},
        /// Automatically set to `1` whenever a conversion is running and back to `0` when the results have been transferred to the data registers.
        measuring: bool = false,
        /// Reserved bits
        _: u8{2},
        /// Automatically set to `1` when the NVM data is being copied to image registers and back to `0` when the
        /// copying is done. The data are copied at power-on-reset and before every conversion.
        update: bool = false,
    }

    /// The measurement control register sets the pressure and temperature
    /// data acquisition options of the device. The register needs to be written
    /// after changing ControlHumidity for those changes to become effective.
    register ControlMeasurement(addr = 0xf4, mode = rw, size = 1) {
        /// Controls oversampling of temperature data.
        temperature_oversampling: Oversampling = Oversampling::Disabled,
        /// Controls oversampling of pressure data.
        pressure_oversampling: Oversampling = Oversampling::Disabled,
        /// Controls operating mode of the sensor.
        sensor_mode: SensorMode = SensorMode::Sleep,
    }

    /// The config register sets the rate, filter and interface options of the device.
    /// Writes to this register in Normal mode may be ignored.
    /// In Sleep mode writes are not ignored.
    register Config(addr = 0xf5, mode = rw, size = 1) {
        /// Controls inactive duration t_standby in Normal sensor mode.
        standby_time: StandbyTime = StandbyTime::T_0_5,
        /// Controls the time constant of the IIR filter.
        filter: IIRFilter = IIRFilter::Disabled,
        /// Reserved bit
        _: bool,
        /// Whether to enable the SPI 3-wire interface.
        spi_3wire: bool = false,
    }

    /// Device-internal calibration registers (section 1)
    register TrimmingParameters1(addr = 0x88, mode = r, size = 26) {
        dig_t1: u16,
        dig_t2: i16,
        dig_t3: i16,
        dig_p1: u16,
        dig_p2: i16,
        dig_p3: i16,
        dig_p4: i16,
        dig_p5: i16,
        dig_p6: i16,
        dig_p7: i16,
        dig_p8: i16,
        dig_p9: i16,
        /// Reserved byte
        _: u8,
        dig_h1: u8,
    }

    /// Device-internal calibration registers (section 2)
    register TrimmingParameters2(addr = 0xe1, mode = r, size = 7) {
        dig_h2: i16,
        dig_h3: u8,
        dig_h4_msb: i8,
        dig_h5_lsn_h4_lsn: i8,
        dig_h5_msb: i8,
        dig_h6: i8,
    }

    /// This register contains the raw pressure measurement
    register Pressure(addr = 0xf7, mode = r, size = 3) {
        /// The raw pressure measurement
        value: u32{20} = 1 << 19,
        /// Reserved bits
        _: u8{4},
    }

    /// This register contains the raw temperature measurement
    register Temperature(addr = 0xfa, mode = r, size = 3) {
        /// The raw temperature measurement
        value: u32{20} = 1 << 19,
        /// Reserved bits
        _: u8{4},
    }

    /// This register contains the raw humidity measurement
    register Humidity(addr = 0xfd, mode = r, size = 2) {
        /// The raw humidity measurement
        value: u16 = 1 << 15,
    }

    /// Burst register read of pressure and temperature
    register BurstMeasurementsPT(addr = 0xf7, mode = r, size = 6) {
        pressure: PressureUnpacked,
        temperature: TemperatureUnpacked,
    }

    /// Burst register read of pressure, temperature and humidity
    register BurstMeasurementsPTH(addr = 0xf7, mode = r, size = 8) {
        pressure: PressureUnpacked,
        temperature: TemperatureUnpacked,
        humidity: HumidityUnpacked,
    }
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

impl StandbyTime {
    /// Standby time in microseconds.
    pub fn time_us(&self) -> u32 {
        match self {
            StandbyTime::T_0_5 => 500,
            StandbyTime::T_62_5 => 62_500,
            StandbyTime::T_125 => 125_000,
            StandbyTime::T_250 => 250_000,
            StandbyTime::T_500 => 500_000,
            StandbyTime::T_1000 => 1_000_000,
            StandbyTime::T_10 => 10_000,
            StandbyTime::T_20 => 20_000,
        }
    }
}
