use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{Length, Pressure, Ratio, ThermodynamicTemperature, Time, VolumetricNumberDensity},
    length::meter,
    pressure::hectopascal,
    ratio::part_per_million,
    thermodynamic_temperature::degree_celsius,
    time::second,
    volumetric_number_density::per_cubic_centimeter,
};

interface_objects! {
    /// Whether data is ready to be read out
    enum DataReadyStatus: u8{1} {
        0 NotReady,
        1 Ready,
    }

    struct DataReady(size=2) {
        _: u16{15},
        /// When no measurement is running, [`DataReadyStatus::NotReady`] will be returned.
        data_ready: DataReadyStatus = DataReadyStatus::NotReady,
    }

    struct NumberConcentrationValues(size=10) {
        /// PM0.5 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_concentration_pm0_5: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM1 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_concentration_pm1: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM2.5 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_concentration_pm2_5: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM4 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_concentration_pm4: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM10 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_concentration_pm10: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
    }

    struct TemperatureOffsetParameters(size=8) {
        /// Temperature offset, LSB = 0.005°C
        raw_offset: i16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 200f64,
        },
        /// Normalized temperature offset slope factor, LSB = 0.0001
        slope: i16 = 0,
        /// The time constant determines how fast the new slope and offset will be applied. After the
        /// specified value in seconds, 63% of the new slope and offset are applied. A time constant of
        /// zero means the new values will be applied immediately (within the next measure interval of
        /// 1 second). LSB = 1 second.
        raw_time_constant: u16 = 0 => {
            quantity: Time,
            unit: second,
            lsb: 1f64 / 1f64,
        },
        /// The temperature offset slot to be modified. Valid values are 0 to 4. If the value is
        /// outside this range, the parameters will not be applied.
        slot: u16 = 0,
    }

    struct TemperatureAccelerationParameters(size=8) {
        /// Filter constant K, LSB = 0.1
        k: u16,
        /// Filter constant P, LSB = 0.1
        p: u16,
        /// Filter constant T1, LSB = 0.1 second
        raw_t1: u16 = 0 => {
            quantity: Time,
            unit: second,
            lsb: 1f64 / 10f64,
        },
        /// Filter constant T2, LSB = 0.1 second
        raw_t2: u16 = 0 => {
            quantity: Time,
            unit: second,
            lsb: 1f64 / 10f64,
        },
    }

    struct ProductName(size=32) {
        /// Null-terminated ASCII string containing the product name.
        name: [u8; 32],
    }

    struct SerialNumber(size=32) {
        /// Null-terminated ASCII string containing the serial number.
        serial_number: [u8; 32],
    }

    struct VOCAlgorithmTuningParameters(size=12) {
        /// VOC index representing typical (average) conditions. Allowed values are in range 1-250.
        /// The default value is 100.
        index_offset: i16 = 100,
        /// Time constant to estimate the VOC algorithm offset from the history in hours. Past events
        /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
        /// The default value is 12 hours.
        learning_time_offset_hours: i16 = 12,
        /// Time constant to estimate the VOC algorithm gain from the history in hours. Past events
        /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
        /// The default value is 12 hours.
        learning_time_gain_hours: i16 = 12,
        /// Maximum duration of gating in minutes (freeze of estimator during high VOC index signal).
        /// Set to zero to disable the gating. Allowed values are in range 0-3000. The default value
        /// is 180 minutes.
        gating_max_duration_minutes: i16 = 180,
        /// Initial estimate for standard deviation. Lower value boosts events during initial learning
        /// period but may result in larger device-to-device variations. Allowed values are in range
        /// 10-5000. The default value is 50.
        std_initial: i16 = 50,
        /// Gain factor to amplify or to attenuate the VOC index output. Allowed values are in range
        /// 1-1000. The default value is 230.
        gain_factor: i16 = 230,
    }

    struct VOCAlgorithmState(size=8) {
        /// The opaque internal state.
        state: [u8; 8],
    }

    struct NOxAlgorithmTuningParameters(size=12) {
        /// NOx index representing typical (average) conditions. Allowed values are in range 1-250.
        /// The default value is 1.
        index_offset: i16 = 1,
        /// Time constant to estimate the NOx algorithm offset from the history in hours. Past events
        /// will be forgotten after about twice the learning time. Allowed values are in range 1-1000.
        /// The default value is 12 hours.
        learning_time_offset_hours: i16 = 12,
        /// The time constant to estimate the NOx algorithm gain from the history has no impact for
        /// NOx. This parameter is still in place for consistency reasons with the VOC tuning
        /// parameters command. This parameter must always be set to 12 hours.
        learning_time_gain_hours: i16 = 12,
        /// Maximum duration of gating in minutes (freeze of estimator during high NOx index signal).
        /// Set to zero to disable the gating. Allowed values are in range 0-3000. The default value
        /// is 720 minutes.
        gating_max_duration_minutes: i16 = 720,
        /// The initial estimate for standard deviation parameter has no impact for NOx. This parameter
        /// is still in place for consistency reasons with the VOC tuning parameters command. This
        /// parameter must always be set to 50.
        std_initial: i16 = 50,
        /// Gain factor to amplify or to attenuate the NOx index output. Allowed values are in range
        /// 1-1000. The default value is 230.
        gain_factor: i16 = 230,
    }

    struct TargetCo2Concentration(size=2) {
        /// Target co2 concentration of the test setup. LSB = 1 ppm.
        raw_target_co2_concentration: u16 = 0 => {
            quantity: Ratio,
            unit: part_per_million,
            lsb: 1f64 / 1f64,
        },
    }

    struct Co2Correction(size=2) {
        /// Correction value as received from the SCD in ppm. To get the FRC correction in ppm,
        /// subtract `0x8000` from this value.
        ///
        /// If the recalibration has failed this value is 0xFFFF.
        raw_correction: u16 = u16::MAX => {
            quantity: Ratio,
            unit: part_per_million,
            from_raw: |x| (x - 0x8000) as f64,
            into_raw: |x| x as u16 + 0x8000,
        },
    }

    struct Co2SensorAutomaticSelfCalibration(size=2) {
        _: u16{15},
        /// Set to true to enable or false to disable the automatic CO2 measurement self-calibration
        /// feature.
        enable: bool,
    }

    struct AmbientPressure(size=2) {
        /// Ambient pressure in hPa to be used for pressure compensation. LSB = 1 hPa.
        raw_pressure: u16 = 1013 => {
            quantity: Pressure,
            unit: hectopascal,
            lsb: 1f64 / 1f64,
        },
    }

    struct SensorAltitude(size=2) {
        /// Sensor altitude, valid input between 0 and 3000m. LSB = 1 meter.
        raw_altitude: u16 = 0 => {
            quantity: Length,
            unit: meter,
            lsb: 1f64 / 1f64,
        },
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen63c", crate::devices::sensirion::sen63c::SEN63CCommand),
        ("sensirion-sen65", crate::devices::sensirion::sen65::SEN65Command),
        ("sensirion-sen66", crate::devices::sensirion::sen66::SEN66Command),
        ("sensirion-sen68", crate::devices::sensirion::sen68::SEN68Command),
    ];

    /// Starts a continuous measurement. After starting the measurement, it takes some time (~1.1s)
    /// until the first measurement results are available. You could poll with the command
    /// [`GetDataReady`] to check when the results are ready to be read.
    ///
    /// Cannot be executed during measurement.
    send 0x0021 time_ms=50 StartContinuousMeasurement();

    /// Stops the measurement and returns to idle mode.
    ///
    /// May be executed during measurement.
    send 0x0104 time_ms=1000 StopMeasurement();

    /// This command can be used to check if new measurement results are ready to read. The data ready
    /// flag is automatically reset after reading the measurement values
    ///
    /// May be executed during measurement.
    read 0x0202 time_ms=20 GetDataReady() -> DataReady;

    /// Returns the measured number concentration values. The command [`GetDataReady`] can be used to
    /// check if new data is available since the last read operation. If no new data is available, the
    /// previous values will be returned. If no data is available at all (e.g. measurement not running
    /// for at least one second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for
    /// i16).
    ///
    /// May be executed during measurement.
    read 0x0316 time_ms=20 ReadNumberConcentrationValues() -> NumberConcentrationValues;

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
    ///
    /// May be executed during measurement.
    write 0x60b2 time_ms=20 SetTemperatureOffsetParameters(TemperatureOffsetParameters);

    /// This command allows to set custom temperature acceleration parameters of the RH/T engine. It
    /// overwrites the default temperature acceleration parameters of the RH/T engine with custom
    /// values.
    ///
    /// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
    /// values after a device reset.
    ///
    /// Cannot be executed during measurement.
    write 0x6100 time_ms=20 SetTemperatureAccelerationParameters(TemperatureAccelerationParameters);

    /// Gets the product name from the device.
    ///
    /// May be executed during measurement.
    read 0xd014 time_ms=20 GetProductName() -> ProductName;

    /// Gets the serial number from the device.
    ///
    /// May be executed during measurement.
    read 0xd033 time_ms=20 GetSerialNumber() -> SerialNumber;

    /// Executes a reset on the device. This has the same effect as a power cycle.
    ///
    /// Cannot be executed during measurement.
    send 0xd304 time_ms=20 DeviceReset();

    /// This command triggers fan cleaning. The fan is set to the maximum speed for 10 seconds and then
    /// automatically stopped. Wait at least 10s after this command before starting a measurement.
    ///
    /// Cannot be executed during measurement.
    send 0x5607 time_ms=20 StartFanCleaning();

    /// This command allows you to use the inbuilt heater in SHT sensor to reverse creep at high
    /// humidity. This command activates the SHT sensor heater with 200mW for 1s. The heater is then
    /// automatically deactivated again.
    ///
    /// Note: Wait at least 20s after this command before starting a measurement to get coherent
    /// temperature values (heating consequence to disappear).
    ///
    /// Cannot be executed during measurement.
    send 0x6765 time_ms=1300 ActivateSHTHeater();
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen65", crate::devices::sensirion::sen65::SEN65Command),
        ("sensirion-sen66", crate::devices::sensirion::sen66::SEN66Command),
        ("sensirion-sen68", crate::devices::sensirion::sen68::SEN68Command),
    ];

    /// Gets the parameters to customize the VOC algorithm.
    ///
    /// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
    /// values after a device reset.
    ///
    /// Cannot be executed during measurement.
    read 0x60d0 time_ms=20 GetVOCAlgorithmTuningParameters() -> VOCAlgorithmTuningParameters;

    /// Sets the parameters to customize the VOC algorithm. Writing has no effect if at least one
    /// parameter is outside the specified range.
    ///
    /// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
    /// values after a device reset.
    ///
    /// Cannot be executed during measurement.
    write 0x60d0 time_ms=20 SetVOCAlgorithmTuningParameters(VOCAlgorithmTuningParameters);

    /// Allows backup/restore of the VOC algorithm state to resume operation after a power cycle or
    /// device reset, skipping initial learning phase.
    ///
    /// By default, the VOC Engine is reset, and the algorithm state is retained if a measurement is
    /// stopped and started again. If the VOC algorithm state shall be reset, a device reset, or a
    /// power cycle can be executed.
    ///
    /// Reads the current VOC algorithm state. This data can be used to restore the state
    /// with [`SetVOCAlgorithmState`] command after a short power cycle or device reset.
    ///
    /// This can be read either in measure mode or in idle mode (which will then return the state at
    /// the time when the measurement was stopped). In measure mode, the state can be read each measure
    /// interval to always have the latest state available, even in case of a sudden power loss.
    ///
    /// May be executed during measurement.
    read 0x6181 time_ms=20 GetVOCAlgorithmState() -> VOCAlgorithmState;

    /// Allows backup/restore of the VOC algorithm state to resume operation after a power cycle or
    /// device reset, skipping initial learning phase.
    ///
    /// Writing is only possible in idle mode and the state will be applied only once when starting the
    /// next measurement.
    ///
    /// Cannot be executed during measurement.
    write 0x6181 time_ms=20 SetVOCAlgorithmState(VOCAlgorithmState);

    /// Gets the parameters to customize the NOx algorithm.
    ///
    /// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
    /// values after a device reset.
    ///
    /// Cannot be executed during measurement.
    read 0x60e1 time_ms=20 GetNOxAlgorithmTuningParameters() -> NOxAlgorithmTuningParameters;

    /// Sets the parameters to customize the NOx algorithm. Writing has no effect if at least one
    /// parameter is outside the specified range.
    ///
    /// Note: This configuration is volatile, i.e. the parameters will be reverted to their default
    /// values after a device reset.
    ///
    /// Cannot be executed during measurement.
    write 0x60e1 time_ms=20 SetNOxAlgorithmTuningParameters(NOxAlgorithmTuningParameters);
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen63c", crate::devices::sensirion::sen63c::SEN63CCommand),
        ("sensirion-sen66", crate::devices::sensirion::sen66::SEN66Command),
    ];

    /// Execute the forced recalibration (FRC) of the CO2 signal. Refer to the datasheet for additional
    /// information on how this is to be used.
    ///
    /// Note: After power-on wait at least 1000 ms and after stopping a measurement 600 ms before
    /// sending this command. The recalibration procedure will take about 500 ms to complete, during
    /// which time no other functions can be executed.
    ///
    /// Cannot be executed during measurement.
    write_read 0x6707 time_ms=500 PerformForcedCo2Recalibration(TargetCo2Concentration) -> Co2Correction;

    /// Gets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor
    /// supports automatic self-calibration (ASC) for long-term stability of the CO2 output. This
    /// feature can be enabled or disabled. By default, it is enabled.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// Cannot be executed during measurement.
    read 0x6711 time_ms=20 GetCo2SensorAutomaticSelfCalibration() -> Co2SensorAutomaticSelfCalibration;

    /// Sets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor
    /// supports automatic self-calibration (ASC) for long-term stability of the CO2 output. This
    /// feature can be enabled or disabled. By default, it is enabled.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// Cannot be executed during measurement.
    write 0x6711 time_ms=20 SetCo2SensorAutomaticSelfCalibration(Co2SensorAutomaticSelfCalibration);

    /// Gets the ambient pressure value. The ambient pressure can be used for pressure
    /// compensation in the CO2 sensor. Setting an ambient pressure overrides any pressure compensation
    /// based on a previously set sensor altitude. Use of this command is recommended for applications
    /// experiencing significant ambient pressure changes to ensure CO2 sensor accuracy. Valid input
    /// values are between 700 to 1200 hPa. The default value is 1013 hPa.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// May be executed during measurement.
    read 0x6720 time_ms=20 GetAmbientPressure() -> AmbientPressure;

    /// Sets the ambient pressure value. The ambient pressure can be used for pressure
    /// compensation in the CO2 sensor. Setting an ambient pressure overrides any pressure compensation
    /// based on a previously set sensor altitude. Use of this command is recommended for applications
    /// experiencing significant ambient pressure changes to ensure CO2 sensor accuracy. Valid input
    /// values are between 700 to 1200 hPa. The default value is 1013 hPa.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// May be executed during measurement.
    write 0x6720 time_ms=20 SetAmbientPressure(AmbientPressure);

    /// Gets the current sensor altitude. The sensor altitude can be used for pressure
    /// compensation in the CO2 sensor. The default sensor altitude value is set to 0 meters above sea
    /// level. Valid input values are between 0 and 3000m.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// Cannot be executed during measurement.
    read 0x6736 time_ms=20 GetSensorAltitude() -> SensorAltitude;

    /// Sets the current sensor altitude. The sensor altitude can be used for pressure
    /// compensation in the CO2 sensor. The default sensor altitude value is set to 0 meters above sea
    /// level. Valid input values are between 0 and 3000m.
    ///
    /// Note: This configuration is volatile, i.e. the parameter will be reverted to its default values
    /// after a device reset.
    ///
    /// Cannot be executed during measurement.
    write 0x6736 time_ms=20 SetSensorAltitude(SensorAltitude);
}
