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
        MeasureSingleShot,
        MeasureSingleShotRhtOnly,
        PerformFactoryReset,
        PerformForcedRecalibration,
        PerformForcedRecalibrationResult,
        PerformSelfTest,
        PersistSettings,
        PowerDown,
        ReadMeasurement,
        Reinit,
        SetAutomaticSelfCalibrationEnabled,
        SetAutomaticSelfCalibrationInitialPeriod,
        SetAutomaticSelfCalibrationStandardPeriod,
        SetAutomaticSelfCalibrationTarget,
        SetSensorAltitude,
        SetTemperatureOffset,
        StartLowPowerPeriodicMeasurement,
        StartPeriodicMeasurement,
        StopPeriodicMeasurement,
        WakeUp,
    }
}
