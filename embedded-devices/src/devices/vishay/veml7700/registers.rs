use embedded_interfaces::codegen::interface_objects;
use embedded_interfaces::registers::i2c::codecs::OneByteRegAddrCodec;

pub type VEML7700I2cCodec = OneByteRegAddrCodec;

interface_objects! {
    register_defaults {
        codec_error = (),
        i2c_codec = VEML7700I2cCodec,
        spi_codec = embedded_interfaces::registers::spi::codecs::unsupported_codec::UnsupportedCodec::<()>,
    }

    register_devices [ super::VEML7700 ]

    /// Measurement gain
    #[allow(non_camel_case_types)]
    enum Gain: u8{2} {
        /// x1 gain
        0b00 X_1,
        /// x2 gain
        0b01 X_2,
        /// x(1/8) gain
        0b10 X_1_8,
        /// x(1/4) gain
        0b11 X_1_4,
    }

    /// Measurement integration time
    #[allow(non_camel_case_types)]
    enum IntegrationTime: u8{4} {
        /// 25 ms
        0b1100 T_25,
        /// 50 ms
        0b1000 T_50,
        /// 100 ms
        0b0000 T_100,
        /// 200 ms
        0b0001 T_200,
        /// 400 ms
        0b0010 T_400,
        /// 800 ms
        0b0011 T_800,
        _ Invalid(u8),
    }

    /// Number of consecutive out-of-threshold measurements
    /// necessary to generate an interrupt
    #[allow(non_camel_case_types)]
    enum InterruptThresholdCount: u8{2} {
        0b00 One,
        0b01 Two,
        0b10 Four,
        0b11 Eight,
    }

    /// Configuration register
    register Configuration(addr = 0x00, mode = rw, size = 2) {
        /// ALS shutdown
        shutdown: bool[7] = true,
        /// ALS interrupt enable
        interrupt_enable: bool[6] = false,
        /// Reserved bits
        _: u8[4,5],
        /// ALS persistence protect number
        interrupt_threshold_count: InterruptThresholdCount[2,3] = InterruptThresholdCount::One,
        /// ALS integration time
        integration_time: IntegrationTime[14,15,0,1] = IntegrationTime::T_100,
        /// Reserved bit
        _: u8[13],
        /// ALS gain
        gain: Gain[11,12] = Gain::X_1_8,
        /// Reserved bits
        _: u8[8,9,10],
    }

    /// High threshold window register
    register HighThresholdWindow(addr = 0x01, mode = rw, size = 2) {
        /// ALS high threshold window setting
        als_high_threshold: u16{le},
    }

    /// Low threshold window register
    register LowThresholdWindow(addr = 0x02, mode = rw, size = 2) {
        /// ALS low threshold window setting
        als_low_threshold: u16{le},
    }

    /// Power saving modes
    enum PowerSavingMode: u8{2} {
        /// 500 ms extra delay between measurements
        0b00 Mode1,
        /// 1000 ms extra delay between measurements
        0b01 Mode2,
        /// 2000 ms extra delay between measurements
        0b10 Mode3,
        /// 4000 ms extra delay between measurements
        0b11 Mode4,
    }

    /// Power saving register
    register PowerSaving(addr = 0x03, mode = rw, size = 2) {
        /// Power saving enable
        power_saving_enable: bool[7] = false,
        /// Power saving mode
        power_saving_mode: PowerSavingMode[5,6] = PowerSavingMode::Mode1,
        /// Reserved bits
        _: u16[0..5,8..16],
    }

    /// ALS channel measurement data
    register ALSData(addr = 0x04, mode = r, size = 2) {
        /// ALS channel data
        als_data: u16{le},
    }

    /// White channel measurement data
    register WhiteData(addr = 0x05, mode = r, size = 2) {
        /// White channel data
        white_data: u16{le},
    }

    /// Interrupt status register
    register InterruptStatus(addr = 0x06, mode = r, size = 2) {
        /// Reserved bits
        _: u8,
        int_threshold_low: bool = false,
        int_threshold_high: bool = false,
        /// Reserved bits
        _: u8{6},
    }

    /// Known address options
    enum AddressOption: u8 {
        0xC4 AddressA,
        0xD4 AddressB,
        _ Invalid(u8),
    }

    /// Known device ids
    enum DeviceIDCode: u8 {
        0x81 VEML7700,
        _ Invalid(u8),
    }

    /// Device ID register
    register DeviceID(addr = 0x07, mode = r, size = 2) {
        device_id: DeviceIDCode = DeviceIDCode::Invalid(0),
        address_option_code: AddressOption = AddressOption::Invalid(0),
    }
}

impl PowerSavingMode {
    pub const fn ms(&self) -> u32 {
        match self {
            PowerSavingMode::Mode1 => 500,
            PowerSavingMode::Mode2 => 1000,
            PowerSavingMode::Mode3 => 2000,
            PowerSavingMode::Mode4 => 4000,
        }
    }
}

impl PowerSaving {
    pub fn ms(&self) -> u32 {
        if self.read_power_saving_enable() {
            self.read_power_saving_mode().ms()
        } else {
            0
        }
    }
}

impl IntegrationTime {
    pub const fn ms(&self) -> u32 {
        match self {
            IntegrationTime::T_25 => 25,
            IntegrationTime::T_50 => 50,
            IntegrationTime::T_100 => 100,
            IntegrationTime::T_200 => 200,
            IntegrationTime::T_400 => 400,
            IntegrationTime::T_800 => 800,
            IntegrationTime::Invalid(_) => 0,
        }
    }

    pub(super) const fn k_hz(&self) -> f32 {
        match self {
            IntegrationTime::T_25 => 0.04,
            IntegrationTime::T_50 => 0.02,
            IntegrationTime::T_100 => 0.01,
            IntegrationTime::T_200 => 0.005,
            IntegrationTime::T_400 => 0.0025,
            IntegrationTime::T_800 => 0.00125,
            IntegrationTime::Invalid(_) => 0.0,
        }
    }

    pub(super) const fn increase_by_one(&self) -> Option<Self> {
        let res = match self {
            IntegrationTime::T_25 => IntegrationTime::T_50,
            IntegrationTime::T_50 => IntegrationTime::T_100,
            IntegrationTime::T_100 => IntegrationTime::T_200,
            IntegrationTime::T_200 => IntegrationTime::T_400,
            IntegrationTime::T_400 => IntegrationTime::T_800,
            IntegrationTime::T_800 | IntegrationTime::Invalid(_) => return None,
        };

        Some(res)
    }

    pub(super) const fn decrease_by_one(&self) -> Option<Self> {
        let res = match self {
            IntegrationTime::T_800 => IntegrationTime::T_400,
            IntegrationTime::T_400 => IntegrationTime::T_200,
            IntegrationTime::T_200 => IntegrationTime::T_100,
            IntegrationTime::T_100 => IntegrationTime::T_50,
            IntegrationTime::T_50 => IntegrationTime::T_25,
            IntegrationTime::T_25 | IntegrationTime::Invalid(_) => return None,
        };

        Some(res)
    }
}

impl Gain {
    pub const fn factor(&self) -> f32 {
        match self {
            Gain::X_1 => 1.0,
            Gain::X_2 => 2.0,
            Gain::X_1_8 => 0.125,
            Gain::X_1_4 => 0.25,
        }
    }

    pub(super) const fn inv_factor(&self) -> f32 {
        match self {
            Gain::X_1_8 => 8.0,
            Gain::X_1_4 => 4.0,
            Gain::X_1 => 1.0,
            Gain::X_2 => 0.5,
        }
    }

    pub(super) const fn increase_by_one(&self) -> Option<Self> {
        let res = match self {
            Gain::X_1_8 => Gain::X_1_4,
            Gain::X_1_4 => Gain::X_1,
            Gain::X_1 => Gain::X_2,
            Gain::X_2 => return None,
        };

        Some(res)
    }

    pub(super) const fn decrease_by_one(&self) -> Option<Self> {
        let res = match self {
            Gain::X_2 => Gain::X_1,
            Gain::X_1 => Gain::X_1_4,
            Gain::X_1_4 => Gain::X_1_8,
            Gain::X_1_8 => return None,
        };

        Some(res)
    }
}

impl Configuration {
    /// Calculate the lux/count resolution based on
    /// the current gain and integration time settings
    pub fn resolution(&self) -> f32 {
        const BASE_RES: f32 = 6.72;

        let gain = self.read_gain().inv_factor();
        let it = self.read_integration_time().k_hz();

        BASE_RES * it * gain
    }
}
