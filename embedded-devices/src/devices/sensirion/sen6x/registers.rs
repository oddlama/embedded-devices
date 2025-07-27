use crate::devices::sensirion::{SensirionI2cCodec, SensirionI2cCodecConsecutiveFetch};

/// See [`StartContinuousMeasurement`](super::commands::StartContinuousMeasurement) command.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x0021, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartContinuousMeasurement {}

/// Stops the measurement and returns to idle mode.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x0104, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StopMeasurement {}

/// This command can be used to check if new measurement results are ready to read. The data ready
/// flag is automatically reset after reading the measurement values
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x0202, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DataReady {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
    /// True if data is ready, false if not. When no measurement is running, false will be
    /// returned.
    #[register(default = false)]
    pub data_ready: bool,
}

/// Returns the measured number concentration values. The command [`DataReady`] can be used to
/// check if new data is available since the last read operation. If no new data is available, the
/// previous values will be returned. If no data is available at all (e.g. measurement not running
/// for at least one  second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for
/// i16).
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x0316, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 10)]
pub struct NumberConcentrationValues {
    /// PM0.5 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_concentration_pm0_5: u16,
    /// PM1 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_concentration_pm1: u16,
    /// PM2.5 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_concentration_pm2_5: u16,
    /// PM4 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_concentration_pm4: u16,
    /// PM10 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_concentration_pm10: u16,
}

/// This command allows to compensate temperature effects of the design-in at customer side by
/// applying custom temperature offsets to the ambient temperature. The compensated ambient
/// temperature is calculated as follows:
///
/// ```text
/// T_Ambient_Compensated = T_Ambient + (slope * T_Ambient) + offset
/// ```
///
/// Where slope and offset are the values set with this command, smoothed with the specified time
/// constant. All temperatures (`T_Ambient_Compensated`, `T_Ambient` and `offset`) are represented
/// in °C. There are 5 temperature offset slots available that all contribute additively to
/// `T_Ambient_Compensated`. The default values for the temperature offset parameters are all zero,
/// meaning that `T_Ambient_Compensated` is equal to `T_Ambient` by default.
///
/// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
/// value of zero after a device reset.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x60B2, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 8)]
pub struct TemperatureOffsetParameters {
    /// Temperature offset, LSB = 0.005°C
    #[register(default = 0)]
    pub offset: i16,
    /// Normalized temperature offset slope factor, LSB = 0.0001
    #[register(default = 0)]
    pub slope: i16,
    /// The time constant determines how fast the new slope and offset will be applied. After the
    /// specified value in seconds, 63% of the new slope and offset are applied. A time constant of
    /// zero means the new values will be applied immediately (within the next measure interval of
    /// 1 second). LSB = 1 second.
    #[register(default = 0)]
    pub time_constant: u16,
    /// The temperature offset slot to be modified. Valid values are 0 to 4. If the value is
    /// outside this range, the parameters will not be applied.
    #[register(default = 0)]
    pub slot: u16,
}

/// This command allows to set custom temperature acceleration parameters of the RH/T engine. It
/// overwrites the default temperature acceleration parameters of the RH/T engine with custom
/// values.
///
/// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
/// values after a device reset.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x6100, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 8)]
pub struct TemperatureAccelerationParameters {
    /// Filter constant K, LSB = 0.1
    pub k: u16,
    /// Filter constant P, LSB = 0.1
    pub p: u16,
    /// Filter constant T1, LSB = 0.1 second
    pub t1: u16,
    /// Filter constant T2, LSB = 0.1 second
    pub t2: u16,
}

/// Gets the product name from the device.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0xD014, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 32)]
pub struct ProductName {
    /// Null-terminated ASCII string containing the product name.
    pub name: [u8; 32],
}

/// Gets the serial number from the device.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0xD033, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 32)]
pub struct SerialNumber {
    /// Null-terminated ASCII string containing the serial number.
    pub serial_number: [u8; 32],
}

/// Executes a reset on the device. This has the same effect as a power cycle.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0xD304, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct DeviceReset {}

/// This command triggers fan cleaning. The fan is set to the maximum speed for 10 seconds and then
/// automatically stopped. Wait at least 10s after this command before starting a measurement.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x5607, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartFanCleaning {}

/// This command allows you to use the inbuilt heater in SHT sensor to reverse creep at high
/// humidity. This command activates the SHT sensor heater with 200mW for 1s. The heater is then
/// automatically deactivated again.
///
/// Note: Wait at least 20s after this command before starting a measurement to get coherent
/// temperature values (heating consequence to disappear).
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x6765, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct ActivateSHTHeater {}

/// Gets/sets the parameters to customize the VOC algorithm. Writing has no effect if at least one
/// parameter is outside the specified range.
///
/// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
/// values after a device reset.
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x60D0, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 12)]
pub struct VOCAlgorithmTuningParameters {
    /// VOC index representing typical (average) conditions. Allowed values are in range 1-250.
    /// The default value is 100.
    #[register(default = 100)]
    pub index_offset: i16,
    /// Time constant to estimate the VOC algorithm offset from the history in hours. Past events
    /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
    /// The default value is 12 hours.
    #[register(default = 12)]
    pub learning_time_offset_hours: i16,
    /// Time constant to estimate the VOC algorithm gain from the history in hours. Past events
    /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
    /// The default value is 12 hours.
    #[register(default = 12)]
    pub learning_time_gain_hours: i16,
    /// Maximum duration of gating in minutes (freeze of estimator during high VOC index signal).
    /// Set to zero to disable the gating. Allowed values are in range 0-3000. The default value
    /// is 180 minutes.
    #[register(default = 180)]
    pub gating_max_duration_minutes: i16,
    /// Initial estimate for standard deviation. Lower value boosts events during initial learning
    /// period but may result in larger device-to-device variations. Allowed values are in range
    /// 10-5000. The default value is 50.
    #[register(default = 50)]
    pub std_initial: i16,
    /// Gain factor to amplify or to attenuate the VOC index output. Allowed values are in range
    /// 1-1000. The default value is 230.
    #[register(default = 230)]
    pub gain_factor: i16,
}

/// Allows backup/restore of the VOC algorithm state to resume operation after a power cycle or
/// device reset, skipping initial learning phase.
///
/// By default, the VOC Engine is reset, and the algorithm state is retained if a measurement is
/// stopped and started again. If the VOC algorithm state shall be reset, a device reset, or a
/// power cycle can be executed.
///
/// Reads or writes the current VOC algorithm state. This data can be used to restore the state
/// with [`VOCAlgorithmState`] command after a short power cycle or device reset.
///
/// This can be read either in measure mode or in idle mode (which will then return the state at
/// the time when the measurement was stopped). In measure mode, the state can be read each measure
/// interval to always have the latest state available, even in case of a sudden power loss.
///
/// Writing is only possible in idle mode and the state will be applied only once when starting the
/// next measurement.
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x6181, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 8)]
pub struct VOCAlgorithmState {
    /// The opaque internal state.
    state: [u8; 8],
}

/// Gets/sets the parameters to customize the NOx algorithm. Writing has no effect if at least one
/// parameter is outside the specified range.
///
/// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
/// values after a device reset.
#[cfg_attr(
    feature = "sensirion-sen65",
    device_register(crate::devices::sensirion::sen65::SEN65)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[cfg_attr(
    feature = "sensirion-sen68",
    device_register(crate::devices::sensirion::sen68::SEN68)
)]
#[register(address = 0x60E1, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 12)]
pub struct NOxAlgorithmTuningParameters {
    /// NOx index representing typical (average) conditions. Allowed values are in range 1-250.
    /// The default value is 1.
    #[register(default = 1)]
    pub index_offset: i16,
    /// Time constant to estimate the NOx algorithm offset from the history in hours. Past events
    /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
    /// The default value is 12 hours.
    #[register(default = 12)]
    pub learning_time_offset_hours: i16,
    /// The time constant to estimate the NOx algorithm gain from the history has no impact for
    /// NOx. This parameter is still in place for consistency reasons with the VOC tuning
    /// parameters command. This parameter must always be set to 12 hours.
    #[register(default = 12)]
    pub learning_time_gain_hours: i16,
    /// Maximum duration of gating in minutes (freeze of estimator during high NOx index signal).
    /// Set to zero to disable the gating. Allowed values are in range 0-3000. The default value
    /// is 720 minutes.
    #[register(default = 720)]
    pub gating_max_duration_minutes: i16,
    /// The initial estimate for standard deviation parameter has no impact for NOx. This parameter
    /// is still in place for consistency reasons with the VOC tuning parameters command. This
    /// parameter must always be set to 50.
    #[register(default = 50)]
    pub std_initial: i16,
    /// Gain factor to amplify or to attenuate the NOx index output. Allowed values are in range
    /// 1-1000. The default value is 230.
    #[register(default = 230)]
    pub gain_factor: i16,
}

/// Execute the forced recalibration (FRC) of the CO2 signal. Refer to the datasheet for additional
/// information on how this is to be used.
///
/// Note: After power-on wait at least 1000 ms and after stopping a measurement 600 ms before
/// sending this command. The recalibration procedure will take about 500 ms to complete, during
/// which time no other functions can be executed.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[register(address = 0x6707, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformForcedCO2Recalibration {
    /// Target co2 concentration of the test setup. LSB = 1 ppm.
    pub target_co2_concentration: u16,
}

/// Continuation of [`PerformForcedCO2Recalibration`]. Only use this directly after the wait period
/// of forced CO2 recalibration.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[register(address = 0x6707, mode = "r", i2c_codec = "SensirionI2cCodecConsecutiveFetch")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformForcedCO2RecalibrationResult {
    /// Correction value as received from the SCD in ppm. To get the FRC correction in ppm,
    /// subtract `0x8000` from this value.
    ///
    /// If the recalibration has failed this value is 0xFFFF.
    #[register(default = u16::MAX)]
    pub correction: u16,
}

/// Gets/sets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor
/// supports automatic self-calibration (ASC) for long-term stability of the CO2 output. This
/// feature can be enabled or disabled. By default, it is enabled.
///
/// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
/// after a device reset.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[register(address = 0x6711, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct CO2SensorAutomaticSelfCalibration {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
    /// Set to true to enable or false to disable the automatic CO2 measurement self-calibration
    /// feature.
    pub enable: bool,
}

/// Gets/sets the ambient pressure value. The ambient pressure can be used for pressure
/// compensation in the CO2 sensor. Setting an ambient pressure overrides any pressure compensation
/// based on a previously set sensor altitude. Use of this command is recommended for applications
/// experiencing significant ambient pressure changes to ensure CO2 sensor accuracy. Valid input
/// values are between 700 to 1200 hPa. The default value is 1013 hPa.
///
/// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
/// after a device reset.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[register(address = 0x6720, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AmbientPressure {
    /// Ambient pressure in hPa to be used for pressure compensation. LSB = 1 hPa.
    #[register(default = 1013)]
    pub pressure: u16,
}

/// Gets/sets the current sensor altitude. The sensor altitude can be used for pressure
/// compensation in the CO2 sensor. The default sensor altitude value is set to 0 meters above sea
/// level. Valid input values are between 0 and 3000m.
///
/// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
/// after a device reset.
#[cfg_attr(
    feature = "sensirion-sen63c",
    device_register(crate::devices::sensirion::sen63c::SEN63C)
)]
#[cfg_attr(
    feature = "sensirion-sen66",
    device_register(crate::devices::sensirion::sen66::SEN66)
)]
#[register(address = 0x6736, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SensorAltitude {
    /// Sensor altitude, valid input between 0 and 3000m. LSB = 1 meter.
    pub altitude: u16,
}
