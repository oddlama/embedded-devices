use crate::utils::reexport_structs;

// Re-export structs that also belong to this device
reexport_structs! {
   crate::devices::sensirion::scd4x::commands, {
        Measurement,
        TemperatureOffset,
        SensorAltitude,
        AmbientPressure,
        TargetCo2Concentration,
        Co2Correction,
        AutomaticSelfCalibrationConfig,
        AutomaticSelfCalibrationTarget,
        DataReady,
        SerialNumber,
        SelfTestResult,
        SensorVariantResult,
        AutomaticSelfCalibrationInitialPeriod,
        AutomaticSelfCalibrationStandardPeriod,
   }
}

pub use crate::devices::sensirion::scd4x::commands::{
    FactoryReset, GetAmbientPressure, GetAutomaticSelfCalibrationEnabled, GetAutomaticSelfCalibrationInitialPeriod,
    GetAutomaticSelfCalibrationStandardPeriod, GetAutomaticSelfCalibrationTarget, GetDataReady, GetSensorAltitude,
    GetSensorVariant, GetSerialNumber, GetTemperatureOffset, MeasureSingleShot, MeasureSingleShotRhtOnly,
    PerformForcedRecalibration, PerformSelfTest, PersistSettings, PowerDown, ReadMeasurement, Reinit,
    SetAmbientPressure, SetAutomaticSelfCalibrationEnabled, SetAutomaticSelfCalibrationInitialPeriod,
    SetAutomaticSelfCalibrationStandardPeriod, SetAutomaticSelfCalibrationTarget, SetSensorAltitude,
    SetTemperatureOffset, StartLowPowerPeriodicMeasurement, StartPeriodicMeasurement, StopPeriodicMeasurement, WakeUp,
};
