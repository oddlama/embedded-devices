use embedded_devices_derive::device_register;
use embedded_registers::register;

use super::registers;
use crate::devices::sensirion::{SensirionI2cCodec, SensirionI2cCodecConsecutiveFetch, sensirion_command};

/// Starts a continuous measurement. After starting the measurement, it takes some time (~1.1s)
/// until the first measurement results are available. You could poll with the command [`DataReady`]
/// to check when the results are ready to be read.
struct StartContinuousMeasurement;
sensirion_command!(send, 50, false, StartContinuousMeasurement);

// sensirion_command!(send, 1000, true, StopMeasurement, registers::StopMeasurement);
// sensirion_command!(read, 20, true, DataReady, registers::DataReady);
// sensirion_command!(
//     read,
//     20,
//     true,
//     NumberConcentrationValues,
//     registers::NumberConcentrationValues
// );
// sensirion_command!(
//     write,
//     20,
//     true,
//     TemperatureOffsetParameters,
//     registers::TemperatureOffsetParameters
// );
// sensirion_command!(
//     write,
//     20,
//     false,
//     TemperatureAccelerationParameters,
//     registers::TemperatureAccelerationParameters
// );
// sensirion_command!(read, 20, true, ProductName, registers::ProductName);
// sensirion_command!(read, 20, true, SerialNumber, registers::SerialNumber);
// sensirion_command!(send, 20, false, DeviceReset, registers::DeviceReset);
// sensirion_command!(send, 20, false, StartFanCleaning, registers::StartFanCleaning);
// sensirion_command!(send, 1300, false, ActivateSHTHeater, registers::ActivateSHTHeater);
// sensirion_command!(
//     read,
//     20,
//     false,
//     VOCAlgorithmTuningParameters,
//     registers::VOCAlgorithmTuningParameters
// );
// sensirion_command!(
//     write,
//     20,
//     false,
//     VOCAlgorithmTuningParameters,
//     registers::VOCAlgorithmTuningParameters
// );
// sensirion_command!(read, 20, true, VOCAlgorithmState, registers::VOCAlgorithmState);
// sensirion_command!(write, 20, true, VOCAlgorithmState, registers::VOCAlgorithmState);
// sensirion_command!(
//     read,
//     20,
//     false,
//     NOxAlgorithmTuningParameters,
//     registers::NOxAlgorithmTuningParameters
// );
// sensirion_command!(
//     write,
//     20,
//     false,
//     NOxAlgorithmTuningParameters,
//     registers::NOxAlgorithmTuningParameters
// );
// sensirion_command!(fetch, 500,  false, PerformForcedCO2Recalibration,     registers::PerformForcedCO2Recalibration     registers::PerformForcedCO2RecalibrationResult);
// sensirion_command!(
//     read,
//     20,
//     false,
//     CO2SensorAutomaticSelfCalibration,
//     registers::CO2SensorAutomaticSelfCalibration
// );
// sensirion_command!(
//     write,
//     20,
//     false,
//     CO2SensorAutomaticSelfCalibration,
//     registers::CO2SensorAutomaticSelfCalibration
// );
// sensirion_command!(read, 20, true, AmbientPressure, registers::AmbientPressure);
// sensirion_command!(write, 20, true, AmbientPressure, registers::AmbientPressure);
// sensirion_command!(read, 20, false, SensorAltitude, registers::SensorAltitude);
// sensirion_command!(write, 20, false, SensorAltitude, registers::SensorAltitude);
