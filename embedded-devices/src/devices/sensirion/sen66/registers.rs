use bondrewd::Bitfields;

use crate::{
    devices::sensirion::{SensirionI2cCodec, sensirion_command},
    utils::reexport_registers,
};

// Re-export registers that also belong to this device
reexport_registers! {
    crate::devices::sensirion::sen6x::registers, {
        ActivateSHTHeater,
        AmbientPressure,
        CO2SensorAutomaticSelfCalibration,
        DataReady,
        DeviceReset,
        NOxAlgorithmTuningParameters,
        NumberConcentrationValues,
        PerformForcedCO2Recalibration,
        PerformForcedCO2RecalibrationResult,
        ProductName,
        SensorAltitude,
        SerialNumber,
        StartContinuousMeasurement,
        StartFanCleaning,
        StopMeasurement,
        TemperatureAccelerationParameters,
        TemperatureOffsetParameters,
        VOCAlgorithmState,
        VOCAlgorithmTuningParameters,
    }
}

/// Returns the measured values. The command [`DataReady`] can be used to check if new data is
/// available since the last read operation. If no new data is available, the previous values will
/// be returned. If no data is available at all (e.g. measurement not running for at least one
/// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
#[device_register(super::SEN66)]
#[register(address = 0x0300, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 18)]
pub struct MeasuredValues {
    /// PM1 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm1: u16,
    /// PM2.5 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm2_5: u16,
    /// PM4 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm4: u16,
    /// PM10 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm10: u16,
    /// Ambient relative humidity, LSB = 0.01%
    #[register(default = i16::MAX)]
    pub relative_humidity: i16,
    /// Ambient temperature, LSB = 0.005°C
    #[register(default = i16::MAX)]
    pub temperature: i16,
    /// VOC index, LSB = 0.1
    #[register(default = i16::MAX)]
    pub voc_index: i16,
    /// NOx index, LSB = 0.1
    /// Will be i16::MAX for the first 10-11 seconds after startup or reset.
    #[register(default = i16::MAX)]
    pub nox_index: i16,
    /// CO2 concentration, LSB = 1 ppm
    /// Will be u16::MAX for the first 5-6 seconds after startup or reset.
    #[register(default = u16::MAX)]
    pub co2_concentration: u16,
}
sensirion_command!(MeasuredValues, 20, true);

/// Returns the measured raw values. The command [`DataReady`] can be used to check if new data
/// is available since the last read operation. If no new data is available, the previous values
/// will be returned. If no data is available at all (e.g. measurement not running for at least one
/// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
#[device_register(super::SEN66)]
#[register(address = 0x0405, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 10)]
pub struct MeasuredRawValues {
    /// Raw relative humidity, LSB = 0.01%
    #[register(default = i16::MAX)]
    pub relative_humidity: i16,
    /// Raw temperature, LSB = 0.005°C
    #[register(default = i16::MAX)]
    pub temperature: i16,
    /// VOC ticks, LSB = 1
    #[register(default = u16::MAX)]
    pub voc_ticks: u16,
    /// NOx ticks, LSB = 1
    /// Will be i16::MAX for the first 10-11 seconds after startup or reset.
    #[register(default = u16::MAX)]
    pub nox_ticks: u16,
    /// CO2 concentration, not interpolated, LSB = 1 ppm
    /// Will be u16::MAX for the first 5-6 seconds after startup or reset.
    #[register(default = u16::MAX)]
    pub co2_concentration: u16,
}
sensirion_command!(MeasuredRawValues, 20, true);

/// Reads the current device status.
///
/// Note: The status flags of type `Error` are sticky, i.e. they are not cleared automatically even
/// if the error condition no longer exists. So, they can only be cleared manually with
/// [`ReadAndClearDeviceStatus`] or through a reset, either by calling [`DeviceReset`] or through a
/// power cycle. All other flags are not sticky, i.e. they are cleared automatically if the trigger
/// condition disappears.
#[device_register(super::SEN66)]
#[register(address = 0xD206, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 4)]
pub struct DeviceStatus {
    #[bondrewd(bit_length = 10, reserve)]
    #[allow(dead_code)]
    pub reserved0: u16,
    /// Fan is switched on, but its speed is more than 10% off the target speed for multiple
    /// consecutive measurement intervals. During the first 10 seconds after starting the
    /// measurement, the fan speed is not checked (settling time). Very low or very high ambient
    /// temperature could trigger this warning during startup. If this flag is set constantly, it
    /// might indicate a problem with the power supply or with the fan, and the measured PM values
    /// might be wrong. This flag is automatically cleared as soon as the measured speed is within
    /// 10% of the target speed or when leaving the measure mode.
    ///
    /// Can occur only in measurement mode.
    pub fan_speed_warning: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved_co2_error1: bool,
    /// Error related to the PM sensor. The particulate matter values might be unknown or wrong if
    /// this flag is set, relative humidity and temperature values might be out of specs due to
    /// compensation algorithms depending on PM sensor state.
    ///
    /// Can occur only in measurement mode.
    pub pm_error: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved_hcho_error: bool,
    /// Error related to the CO2 sensor. The CO2 values might be unknown or wrong if this flag is
    /// set, relative humidity and temperature values might be out of specs due to compensation
    /// algorithms depending on CO2 sensor state.
    ///
    /// Can occur only in measurement mode.
    pub co2_error: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved2: bool,
    /// Error related to the gas sensor. The VOC index and NOx index might be unknown or wrong if
    /// this flag is set, relative humidity and temperature values might be out of specs due to
    /// compensation algorithms depending on gas sensor state.
    ///
    /// Can occur only in measurement mode.
    pub gas_error: bool,
    /// Error related to the RH&T sensor. The temperature and humidity values might be unknown or
    /// wrong if this flag is set, and other measured values might be out of specs due compensation
    /// algorithms depending on RH&T sensor values.
    ///
    /// Can occur only in measurement mode.
    pub rh_t_error: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved3: bool,
    /// Fan is switched on, but 0 RPM is measured for multiple consecutive measurement intervals.
    /// This can occur if the fan is mechanically blocked or broken. Note that the measured values
    /// are most likely wrong if this error is reported.
    ///
    /// Can occur only in measurement mode.
    pub fan_error: bool,
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved4: u8,
}
sensirion_command!(DeviceStatus, 20, true);

/// Reads the current device status (like command [`DeviceStatus`]) and afterwards clears all
/// flags.
#[device_register(super::SEN66)]
#[register(address = 0xD210, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 4)]
pub struct ReadAndClearDeviceStatus {
    /// The current device status.
    #[bondrewd(struct_size = 4)]
    status: DeviceStatusBitfield,
}
sensirion_command!(ReadAndClearDeviceStatus, 20, true);
