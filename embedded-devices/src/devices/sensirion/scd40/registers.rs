use crate::utils::reexport_registers;

// Re-export registers that also belong to this device
reexport_registers! {
    crate::devices::sensirion::scd4x::registers, {
        AmbientPressure,
        DataReady,
        GetAutomaticSelfCalibrationEnabled,
        GetAutomaticSelfCalibrationInitialPeriod,
        GetAutomaticSelfCalibrationStandardPeriod,
        GetAutomaticSelfCalibrationTarget,
        GetSensorAltitude,
        GetSensorVariant,
        GetSerialNumber,
        GetTemperatureOffset,
        PerformFactoryReset,
        PerformForcedRecalibration,
        PerformForcedRecalibrationResult,
        PerformSelfTest,
        PersistSettings,
        ReadMeasurement,
        Reinit,
        SetSensorAltitude,
        SetTemperatureOffset,
        StartLowPowerPeriodicMeasurement,
        StartPeriodicMeasurement,
        StopPeriodicMeasurement,
    }
}
