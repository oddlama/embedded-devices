use crate::devices::sensirion::SensirionI2cCodec;
use embedded_devices_derive::device_register;
use embedded_registers::register;

pub const SENSOR_VARIANT_SCD40: u16 = 0x0000;
pub const SENSOR_VARIANT_SCD41: u16 = 0x1000;
pub const SENSOR_VARIANT_SCD43: u16 = 0x5000;

pub const DATA_READY_MASK: u16 = 0b11111111111;

/// Starts the periodic measurement mode. The signal update interval is 5 second
#[device_register(super::SCD4x)]
#[register(address = 0x21b1, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartPeriodicMeasurement {}

/// Command returns a sensor running in periodic measurement mode or low power periodic measurement
/// mode back to the idle state, e.g. to then allow changing the sensor configuration or to save
/// power. Note that the sensor will only respond to other commands 500 ms after the
/// stop_periodic_measurement command has been issued.
#[device_register(super::SCD4x)]
#[register(address = 0x3f86, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StopPeriodicMeasurement {}

/// Reads the sensor output. The measurement data can only be read out once per signal update
/// interval as the buffer is emptied upon read-out. If no data is available in the buffer, the
/// sensor returns a NACK. To avoid a NACK response, the get_data_ready_status can be issued to
/// check data status. The I2C master can abort the read transfer with a NACK followed by a STOP
/// condition after any data byte if the user is not interested in subsequent data.
#[device_register(super::SCD4x)]
#[register(address = 0xec05, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct ReadMeasurement {
    pub co2: u16,
    pub temperature: u16,
    pub humidity: u16,
}

/// Setting the temperature offset of the SCD4x inside the customer device allows the user to
/// optimize the RH and T output signal. The temperature offset can depend on several factors such
/// as the SCD4x measurement mode, self-heating of close components, the ambient temperature and
/// air flow. Thus, the SCD4x temperature offset should be determined after integration into the
/// final device and under its typical operating conditions (including the operation mode to be
/// used in the application) in thermal equilibrium. By default, the temperature offset is set to 4
/// °C. To save the setting to the EEPROM, the [`PersistSettings`] command may be issued.
///
/// Recommended temperature offset values are between 0°C and 20°C.
#[device_register(super::SCD4x)]
#[register(address = 0x241d, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetTemperatureOffset {
    pub offset: u16,
}

/// Get the current temperature offset value
#[device_register(super::SCD4x)]
#[register(address = 0x1218, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetTemperatureOffset {
    pub offset: u16,
}

/// Reading and writing the sensor altitude must be done while the SCD4x is in idle mode.
/// Typically, the sensor altitude is set once after device installation. To save the setting to
/// the EEPROM, the persist_settings command must be issued. The default sensor altitude value is
/// set to 0 meters above sea level. Valid input values are between 0 - 3,000 m.
#[device_register(super::SCD4x)]
#[register(address = 0x2427, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetSensorAltitude {
    pub altitude: u16,
}

/// The get_sensor_altitude command can be sent while the SCD4x is in idle mode to read out the
/// previously saved sensor altitude value set by the set_sensor_altitude command.
#[device_register(super::SCD4x)]
#[register(address = 0x2322, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetSensorAltitude {
    pub altitude: u16,
}

/// The set_ambient_pressure command can be sent during periodic measurements to enable continuous
/// pressure compensation. Note that setting an ambient pressure overrides any pressure
/// compensation based on a previously set sensor altitude. Use of this command is highly
/// recommended for applications experiencing significant ambient pressure changes to ensure sensor
/// accuracy. Valid input values are between 70,000 - 120,000 Pa. The default value is 101,300 Pa.
#[device_register(super::SCD4x)]
#[register(address = 0xe000, mode = "rw", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct AmbientPressure {
    #[register(default = 1013)]
    pub pressure: u16,
}

/// Sets the current state of the ASC. By default, ASC is enabled. To save the setting to the
/// EEPROM, the persist_settings command must be issued.
#[device_register(super::SCD4x)]
#[register(address = 0x2416, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationEnabled {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved0: u16,
    #[register(default = true)]
    pub enabled: bool,
}

/// Gets the current ASC state
#[device_register(super::SCD4x)]
#[register(address = 0x2313, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationEnabled {
    #[bondrewd(bit_length = 15, reserve)]
    #[allow(dead_code)]
    pub reserved0: u16,
    #[register(default = true)]
    pub enabled: bool,
}

/// Sets the value of the ASC baseline target, i.e. the CO2 concentration in ppm which the ASC
/// algorithm will assume as lower-bound background to which the SCD4x is exposed to regularly
/// within one ASC period of operation. To save the setting to the EEPROM, the persist_settings
/// command must be issued subsequently.
#[device_register(super::SCD4x)]
#[register(address = 0x243a, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationTarget {
    pub target: u16,
}

#[device_register(super::SCD4x)]
#[register(address = 0x233f, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationTarget {
    pub target: u16,
}

/// Starts the low power periodic measurement mode. The signal update interval is approximately 30 seconds.
#[device_register(super::SCD4x)]
#[register(address = 0x21ac, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartLowPowerPeriodicMeasurement {}

/// Polls the sensor for whether data from a periodic or single shot measurement is ready to be read out
#[device_register(super::SCD4x)]
#[register(address = 0xe4b8, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetDataReadyStatus {
    pub status: u16,
}

/// Configuration settings such as the temperature offset, sensor altitude and the ASC
/// enabled/disabled parameters are by default stored in the volatile memory (RAM) only. The
/// persist_settings command stores the current configuration in the EEPROM of the SCD4x, ensuring
/// the current settings persist after power-cycling. To avoid unnecessary wear of the EEPROM, the
/// persist_settings command should only be sent following configuration changes whose persistence
/// is required. The EEPROM is guaranteed to withstand at least 2000 write cycles. Note that field
/// calibration history (i.e. FRC and ASC, see Section 3.8) is automatically stored in a separate
/// EEPROM dimensioned for the specified sensor lifetime when operated continuously in either
/// periodic measurement mode, low power periodic measurement mode (see Section 3.9) or single shot
/// mode with 5 minute measurement interval.
#[device_register(super::SCD4x)]
#[register(address = 0x3615, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PersistSettings {}

/// Reading out the serial number can be used to identify the chip and to verify the presence of
/// the sensor.
#[device_register(super::SCD4x)]
#[register(address = 0x3682, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct GetSerialNumber {
    #[bondrewd(bit_length = 48)]
    pub serial_number: u64,
}

/// The perform_self_test command can be used as an end-of-line test to check the sensor
/// functionality.
#[device_register(super::SCD4x)]
#[register(address = 0x3639, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct PerformSelfTest {
    pub status: u8,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,
}

/// The perform_factory_reset command resets all configuration settings stored in the EEPROM and
/// erases the FRC and ASC algorithm history
#[device_register(super::SCD4x)]
#[register(address = 0x3632, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PerformFactoryReset {}

/// The reinit command reinitializes the sensor by reloading user settings from EEPROM. The sensor
/// must be in the idle state before sending the reinit command. If the reinit command does not
/// trigger the desired re-initialization, a power-cycle should be applied to the SCD4x.
#[device_register(super::SCD4x)]
#[register(address = 0x3646, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct Reinit {}

/// reads out the SCD4x sensor variant (e.g. SCD40 or SCD41).
#[device_register(super::SCD4x)]
#[register(address = 0x202f, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetSensorVariant {
    pub variant: u16,
}

// The following features are only available on the SCD41

/// On-demand measurement of CO2 concentration, relative humidity and temperature. The sensor output is read out
/// by using the read_measurement command
#[device_register(super::SCD4x)]
#[register(address = 0x219d, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct MeasureSingleShot {}

/// On-demand measurement of relative humidity and temperature only, significantly reduces power
/// consumption. The sensor output is read out by using the read_measurement command (Section
/// 3.6.2). CO2 output is returned as 0 ppm
#[device_register(super::SCD4x)]
#[register(address = 0x2196, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct MeasureSingleShotRHTOnly {}

/// Put the sensor from idle to sleep to reduce current consumption. Can be used to power down when
/// operating the sensor in power-cycled single shot mode
#[device_register(super::SCD4x)]
#[register(address = 0x36e0, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct PowerDown {}

/// Wake up the sensor from sleep mode into idle mode. Note that the SCD4x does not acknowledge the
/// wake_up command. The sensor's idle state after wake up can be verified by reading out the
/// serial number.
#[device_register(super::SCD4x)]
#[register(address = 0x36f6, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct WakeUp {}

/// Sets the duration of the initial period for ASC correction (in hours). By default, the initial
/// period for ASC correction is 44 hours. Allowed values are integer multiples of 4 hours. A value
/// of 0 results in an immediate correction. To save the setting to the EEPROM, the
/// persist_settings (see Section 3.10.1) command must be issued. Note: For single shot operation,
/// this parameter always assumes a measurement interval of 5 minutes, counting the number of
/// single shots to calculate elapsed time. If single shot measurements are taken more / less
/// frequently than once every 5 minutes, this parameter must be scaled accordingly to achieve the
/// intended period in hours (e.g. for a 10-minute measurement interval, the scaled parameter value
/// is obtained by multiplying the intended period in hours by 0.5).
#[device_register(super::SCD4x)]
#[register(address = 0x2445, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationInitialPeriod {
    #[register(default = 76)]
    pub hours: u16,
}

#[device_register(super::SCD4x)]
#[register(address = 0x2340, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationInitialPeriod {
    #[register(default = 76)]
    pub hours: u16,
}

/// Sets the standard period for ASC correction (in hours). By default, the standard period for ASC
/// correction is 156 hours. Allowed values are integer multiples of 4 hours. Note: a value of 0
/// results in an immediate correction. To save the setting to the EEPROM, the persist_settings
/// (see Section 3.10.1) command must be issued. Note: For single shot operation, this parameter
/// always assumes a measurement interval of 5 minutes, counting the number of single shots to
/// calculate elapsed time. If single shot measurements are taken more / less frequently than once
/// every 5 minutes, this parameter must be scaled accordingly to achieve the intended period in
/// hours (e.g. for a 10-minute measurement interval, the scaled parameter value is obtained by
/// multiplying the intended period in hours by 0.5).
#[device_register(super::SCD4x)]
#[register(address = 0x2445, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct SetAutomaticSelfCalibrationStandardPeriod {
    #[register(default = 156)]
    pub hours: u16,
}

#[device_register(super::SCD4x)]
#[register(address = 0x2340, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct GetAutomaticSelfCalibrationStandardPeriod {
    #[register(default = 156)]
    pub hours: u16,
}
