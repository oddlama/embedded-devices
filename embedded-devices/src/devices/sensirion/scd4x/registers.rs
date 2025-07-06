use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;

use crate::devices::sensirion::{SensirionI2cCodec, SensirionI2cCodecConsecutiveFetch, sensirion_command};

/// Starts the periodic measurement mode. The signal update interval is 5 seconds.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x21b1, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartPeriodicMeasurement {}
sensirion_command!(StartPeriodicMeasurement, 0, false);

/// Reads the sensor output. The measurement data can only be read out once per signal update
/// interval as the buffer is emptied upon read-out. If no data is available in the buffer, the
/// sensor returns a NACK. To avoid a NACK response, [`DataReady`] can be issued to check
/// data status. The I2C master can abort the read transfer with a NACK followed by a STOP
/// condition after any data byte if the user is not interested in subsequent data.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0xec05, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct ReadMeasurement {
    /// CO2 concentration, LSB = 1 ppm
    pub co2_concentration: u16,
    /// Ambient temperature, T(°C) = -45 + 175 / (2^16 - 1) * value
    pub temperature: u16,
    /// Ambient relative humidity, RH(%) = 100 / (2^16 - 1) * value
    pub relative_humidity: u16,
}
sensirion_command!(ReadMeasurement, 1, true);

/// Command returns a sensor running in periodic measurement mode or low power periodic measurement
/// mode back to the idle state, e.g. to then allow changing the sensor configuration or to save
/// power.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3f86, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StopPeriodicMeasurement {}
sensirion_command!(StopPeriodicMeasurement, 500, true);

/// Setting the temperature offset of the SCD4x inside the customer device allows the user to
/// optimize the RH and T output signal. The temperature offset can depend on several factors such
/// as the SCD4x measurement mode, self-heating of close components, the ambient temperature and
/// air flow. Thus, the SCD4x temperature offset should be determined after integration into the
/// final device and under its typical operating conditions (including the operation mode to be
/// used in the application) in thermal equilibrium. By default, the temperature offset is set to 4
/// °C. To save the setting to the EEPROM, the [`PersistSettings`] command may be issued.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x241d, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetTemperatureOffset {
    /// Offset temperature, value = T_offset[°C] * (2^16 - 1) / 175
    pub temperature: u16,
}
sensirion_command!(SetTemperatureOffset, 1, false);

/// See [`SetTemperatureOffset`] for details.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2318, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetTemperatureOffset {
    /// Offset temperature, T_offset[°C] = value * 175 / (2^16 - 1)
    pub temperature: u16,
}
sensirion_command!(GetTemperatureOffset, 1, false);

/// Reading and writing the sensor altitude must be done while the SCD4x is in idle mode.
/// Typically, the sensor altitude is set once after device installation. To save the setting to
/// the EEPROM, the [`PersistSettings`] command must be issued. The default sensor altitude value
/// is set to 0 meters above sea level. Valid input values are between 0 - 3000 m.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2427, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetSensorAltitude {
    /// Sensor altitude, valid input between 0 and 3000m. LSB = 1 meter.
    pub altitude: u16,
}
sensirion_command!(SetSensorAltitude, 1, false);

/// See [`SetSensorAltitude`] for details.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2322, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetSensorAltitude {
    /// Sensor altitude, between 0 and 3000m. LSB = 1 meter.
    pub altitude: u16,
}
sensirion_command!(GetSensorAltitude, 1, false);

/// Gets/sets the ambient pressure value. The ambient pressure can be used for pressure
/// compensation in the CO2 sensor. Setting an ambient pressure overrides any pressure compensation
/// based on a previously set sensor altitude. Use of this command is recommended for applications
/// experiencing significant ambient pressure changes to ensure CO2 sensor accuracy. Valid input
/// values are between 700 to 1200 hPa. The default value is 1013 hPa.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0xe000, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AmbientPressure {
    /// Ambient pressure in hPa to be used for pressure compensation. LSB = 1 hPa.
    #[register(default = 1013)]
    pub pressure: u16,
}
sensirion_command!(AmbientPressure, 1, true);

/// Execute the forced recalibration (FRC) of the CO2 signal. Refer to the datasheet for additional
/// information on how this is to be used.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x362f, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformForcedRecalibration {
    /// Target co2 concentration of the test setup. LSB = 1 ppm.
    pub target_co2_concentration: u16,
}
sensirion_command!(PerformForcedRecalibration, 400, false);

/// Continuation of [`PerformForcedRecalibration`]. Only use this directly after the wait period
/// of forced CO2 recalibration.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x362f, mode = "r", i2c_codec = "SensirionI2cCodecConsecutiveFetch")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformForcedRecalibrationResult {
    /// Correction value as received from the SCD in ppm. To get the FRC correction in ppm,
    /// subtract `0x8000` from this value.
    ///
    /// If the recalibration has failed this value is 0xFFFF.
    #[register(default = u16::MAX)]
    pub correction: u16,
}
sensirion_command!(PerformForcedRecalibrationResult, 0, false);

/// Sets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor supports
/// automatic self-calibration (ASC) for long-term stability of the CO2 output. This feature can be
/// enabled or disabled. By default, it is enabled.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2416, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationEnabled {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
    /// Set to true to enable or false to disable the automatic CO2 measurement self-calibration
    /// feature.
    pub enable: bool,
}
sensirion_command!(SetAutomaticSelfCalibrationEnabled, 1, false);

/// Gets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor supports
/// automatic self-calibration (ASC) for long-term stability of the CO2 output. This feature can be
/// enabled or disabled. By default, it is enabled.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2313, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationEnabled {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
    /// Set to true to enable or false to disable the automatic CO2 measurement self-calibration
    /// feature.
    pub enable: bool,
}
sensirion_command!(GetAutomaticSelfCalibrationEnabled, 1, false);

/// Sets the value of the ASC baseline target, i.e. the CO2 concentration in ppm which the ASC
/// algorithm will assume as lower-bound background to which the SCD4x is exposed to regularly
/// within one ASC period of operation. To save the setting to the EEPROM, the [`PersistSettings`]
/// command must be issued subsequently.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x243a, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationTarget {
    /// ASC target co2 concentration. LSB = 1 ppm.
    pub target_co2_concentration: u16,
}
sensirion_command!(SetAutomaticSelfCalibrationTarget, 1, false);

/// Gets the value of the ASC baseline target. See [`SetAutomaticSelfCalibrationTarget`] for
/// details.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x233f, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationTarget {
    /// ASC target co2 concentration. LSB = 1 ppm.
    pub target_co2_concentration: u16,
}
sensirion_command!(GetAutomaticSelfCalibrationTarget, 1, false);

/// To enable use-cases with a constrained power budget, the SCD4x features a low power periodic
/// measurement mode with a signal update interval of approximately 30 seconds.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x21ac, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartLowPowerPeriodicMeasurement {}
sensirion_command!(StartLowPowerPeriodicMeasurement, 0, false);

/// Polls the sensor for whether data from a periodic or single shot measurement is ready to be
/// read out.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0xe4b8, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DataReady {
    #[bondrewd(bit_length = 14, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
    /// Unequal to 0 if data is ready, 0 if not. When no measurement is running, 0 will be
    /// returned.
    #[register(default = 0)]
    #[bondrewd(bit_length = 2, reserve)]
    pub data_ready: u8,
}
sensirion_command!(DataReady, 1, true);

/// Configuration settings such as the temperature offset, sensor altitude and the ASC
/// enabled/disabled parameters are by default stored in the volatile memory (RAM) only. This
/// command stores the current configuration in the EEPROM of the SCD4x, ensuring the current
/// settings persist after power-cycling. To avoid unnecessary wear of the EEPROM, this command
/// should only be sent following configuration changes whose persistence is required. The EEPROM
/// is guaranteed to withstand at least 2000 write cycles. Note that field calibration history
/// (i.e. FRC and ASC) is automatically stored in a separate EEPROM dimensioned for the specified
/// sensor lifetime when operated continuously in either periodic measurement mode, low power
/// periodic measurement mode or single shot mode with 5 minute measurement interval (SCD41 and
/// SCD43 only).
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3615, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PersistSettings {}
sensirion_command!(PersistSettings, 800, false);

/// Reading out the serial number can be used to identify the chip and to verify the presence of
/// the sensor.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3682, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct GetSerialNumber {
    /// 6-byte serial number
    pub serial_number: [u8; 6],
}
sensirion_command!(GetSerialNumber, 1, false);

/// This command can be used as an end-of-line test to check the sensor functionality.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3639, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformSelfTest {
    /// Unequal to 0 if data is a malfunction was detected, 0 if everything is as expected.
    pub has_malfunction: u16,
}
sensirion_command!(PerformSelfTest, 10_000, false);

/// Resets all configuration settings stored in the EEPROM and erases the FRC and ASC algorithm
/// history.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3632, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PerformFactoryReset {}
sensirion_command!(PerformFactoryReset, 1_200, false);

/// Reinitializes the sensor by reloading user settings from EEPROM. The sensor must be in the idle
/// state before sending the reinit command. If the reinit command does not trigger the desired
/// re-initialization, a power-cycle should be applied to the SCD4x.
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x3646, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct Reinit {}
sensirion_command!(Reinit, 30, false);

/// Known sensor variants.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum SensorVariant {
    SCD40 = 0b000,
    SCD41 = 0b001,
    Invalid010 = 0b010,
    Invalid011 = 0b011,
    Invalid100 = 0b100,
    SCD43 = 0b101,
    Invalid110 = 0b110,
    Invalid111 = 0b111,
}

/// Reads out the SCD4x sensor variant (e.g. SCD40, SCD41 or SCD43).
#[cfg_attr(
    feature = "sensirion-scd40",
    device_register(crate::devices::sensirion::scd40::SCD40)
)]
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x202f, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetSensorVariant {
    /// The sensor variant.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = SensorVariant::Invalid111)]
    pub variant: SensorVariant,
    #[bondrewd(bit_length = 13, reserve)]
    #[allow(dead_code)]
    pub reserved: u16,
}
sensirion_command!(GetSensorVariant, 1, false);

/// On-demand measurement of CO2 concentration, relative humidity and temperature. The sensor
/// output is read out by using the [`ReadMeasurement`] command.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x219d, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct MeasureSingleShot {}
sensirion_command!(
    MeasureSingleShot,
    1, /* NOTE: Adjusted from datasheet which specified 5_000 but we want to poll manually */
    false
);

/// On-demand measurement of relative humidity and temperature only, significantly reduces power
/// consumption. The sensor output is read out by using the read_measurement command. CO2 output is
/// returned as 0 ppm.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2196, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct MeasureSingleShotRhtOnly {}
sensirion_command!(
    MeasureSingleShotRhtOnly,
    1, /* NOTE: Adjusted from datasheet which specified 5_000 but we want to poll manually */
    false
);

/// Put the sensor from idle to sleep to reduce current consumption. Can be used to power down when
/// operating the sensor in power-cycled single shot mode.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x36e0, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PowerDown {}
sensirion_command!(PowerDown, 1, false);

/// Wake up the sensor from sleep mode into idle mode. Note that the SCD4x does not acknowledge the
/// wake_up command. The sensor's idle state after wake up can be verified by reading out the
/// serial number.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x36f6, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct WakeUp {}
sensirion_command!(WakeUp, 30, false);

/// Sets the duration of the initial period for ASC correction (in hours). By default, the initial
/// period for ASC correction is 44 hours. Allowed values are integer multiples of 4 hours. A value
/// of 0 results in an immediate correction. To save the setting to the EEPROM, the
/// [`PersistSettings`] command must be issued.
///
/// Note: For single shot operation, this parameter always assumes a measurement interval of 5
/// minutes, counting the number of single shots to calculate elapsed time. If single shot
/// measurements are taken more / less frequently than once every 5 minutes, this parameter must be
/// scaled accordingly to achieve the intended period in hours (e.g. for a 10-minute measurement
/// interval, the scaled parameter value is obtained by multiplying the intended period in hours by
/// 0.5).
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2445, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationInitialPeriod {
    /// ASC initial period in hours.
    pub period_hours: u16,
}
sensirion_command!(SetAutomaticSelfCalibrationInitialPeriod, 1, false);

/// See [`SetAutomaticSelfCalibrationInitialPeriod`] for details.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x2340, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationInitialPeriod {
    /// ASC initial period in hours.
    pub period_hours: u16,
}
sensirion_command!(GetAutomaticSelfCalibrationInitialPeriod, 1, false);

/// Sets the standard period for ASC correction (in hours). By default, the standard period for ASC
/// correction is 156 hours. Allowed values are integer multiples of 4 hours. A value of 0 results
/// in an immediate correction. To save the setting to the EEPROM, the [`PersistSettings`] command
/// must be issued.
///
/// Note: For single shot operation, this parameter always assumes a measurement interval of 5
/// minutes, counting the number of single shots to calculate elapsed time. If single shot
/// measurements are taken more / less frequently than once every 5 minutes, this parameter must be
/// scaled accordingly to achieve the intended period in hours (e.g. for a 10-minute measurement
/// interval, the scaled parameter value is obtained by multiplying the intended period in hours by
/// 0.5).
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x244e, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationStandardPeriod {
    /// ASC standard period in hours.
    pub period_hours: u16,
}
sensirion_command!(SetAutomaticSelfCalibrationStandardPeriod, 1, false);

/// See [`SetAutomaticSelfCalibrationStandardPeriod`] for details.
#[cfg_attr(
    feature = "sensirion-scd41",
    device_register(crate::devices::sensirion::scd41::SCD41)
)]
#[cfg_attr(
    feature = "sensirion-scd43",
    device_register(crate::devices::sensirion::scd43::SCD43)
)]
#[register(address = 0x234b, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationStandardPeriod {
    /// ASC standard period in hours.
    pub period_hours: u16,
}
sensirion_command!(GetAutomaticSelfCalibrationStandardPeriod, 1, false);
