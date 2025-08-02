use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{Length, Pressure, Ratio, ThermodynamicTemperature},
    length::meter,
    pressure::hectopascal,
    ratio::{part_per_million, percent},
    thermodynamic_temperature::degree_celsius,
};

interface_objects! {
    /// The variant of the sensor
    enum SensorVariant: u8{3} {
        0b000 SCD40,
        0b001 SCD41,
        0b101 SCD43,
        _ Invalid,
    }

    /// Whether data is ready to be read out
    enum DataReadyStatus: u8{2} {
        0 NotReady,
        _ Ready,
    }

    /// Whether a malfunction was detected
    enum MalfunctionStatus: u16 {
        0 Ok,
        _ Malfunction,
    }

    struct Measurement(size=6) {
        /// CO2 concentration, LSB = 1 ppm
        raw_co2_concentration: u16 = 0 => {
            quantity: Ratio,
            unit: part_per_million,
            lsb: 1f64 / 1f64,
        },
        /// Ambient temperature, T(°C) = -45 + 175 / (2^16 - 1) * value
        raw_temperature: u16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            from_raw: |x| -45.0 + (175.0 * x as f64) / ((1 << 16) - 1) as f64,
            into_raw: |x| (((x + 45.0) * ((1 << 16) - 1) as f64) / 175.0) as u16,
        },
        /// Ambient relative humidity, RH(%) = 100 / (2^16 - 1) * value
        raw_relative_humidity: u16 = 0 => {
            quantity: Ratio,
            unit: percent,
            from_raw: |x| (100.0 * x as f64) / ((1 << 16) - 1) as f64,
            into_raw: |x| ((x * ((1 << 16) - 1) as f64) / 100.0) as u16,
        },
    }

    struct TemperatureOffset(size=2) {
        /// Offset temperature, value = T_offset[°C] * (2^16 - 1) / 175
        temperature: u16,
    }

    struct SensorAltitude(size=2) {
        /// Sensor altitude, between 0 and 3000m. LSB = 1 meter.
        raw_altitude: u16 = 0 => {
            quantity: Length,
            unit: meter,
            lsb: 1f64 / 1f64,
        },
    }

    struct AmbientPressure(size=2) {
        /// Ambient pressure in hPa to be used for pressure compensation. LSB = 1 hPa.
        raw_pressure: u16 = 1013 => {
            quantity: Pressure,
            unit: hectopascal,
            lsb: 1f64 / 1f64,
        },
    }

    struct TargetCo2Concentration(size=2) {
        /// Target co2 concentration of the test setup. LSB = 1 ppm.
        raw_target_co2_concentration: u16 = 400 => {
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
        raw_correction: u16 = 0 => {
            quantity: Ratio,
            unit: part_per_million,
            from_raw: |x| (x - 0x8000) as f64,
            into_raw: |x| x as u16 + 0x8000,
        },
    }

    struct AutomaticSelfCalibrationConfig(size=2) {
        _: u16{15},
        /// Set to true to enable or false to disable the automatic CO2 measurement
        /// self-calibration feature.
        enable: bool,
    }

    struct AutomaticSelfCalibrationTarget(size=2) {
        /// ASC target co2 concentration. LSB = 1 ppm.
        raw_target_co2_concentration: u16 = 400 => {
            quantity: Ratio,
            unit: part_per_million,
            lsb: 1f64 / 1f64,
        },
    }

    struct DataReady(size=2) {
        _: u16{14},
        /// When no measurement is running, [`DataReadyStatus::NotReady`] will be returned.
        data_ready: DataReadyStatus = DataReadyStatus::NotReady,
    }

    struct SerialNumber(size=6) {
        /// 6-byte serial number
        serial_number: [u8; 6],
    }

    struct SelfTestResult(size=2) {
        /// Whether a malfunction was detected.
        malfunction_status: MalfunctionStatus = MalfunctionStatus::Malfunction,
    }

    struct SensorVariantResult(size=2) {
        /// The sensor variant.
        variant: SensorVariant = SensorVariant::Invalid,
        _: u16{13},
    }

    struct AutomaticSelfCalibrationInitialPeriod(size=2) {
        /// ASC initial period in hours.
        period_hours: u16,
    }

    struct AutomaticSelfCalibrationStandardPeriod(size=2) {
        /// ASC standard period in hours.
        period_hours: u16,
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-scd40", crate::devices::sensirion::scd40::SCD40Command),
        ("sensirion-scd41", crate::devices::sensirion::scd41::SCD41Command),
        ("sensirion-scd43", crate::devices::sensirion::scd43::SCD43Command),
    ];

    /// Starts the periodic measurement mode. The signal update interval is 5 seconds.
    send 0x21b1 time_ms=0 StartPeriodicMeasurement();

    /// Reads the sensor output. The measurement data can only be read out once per signal update
    /// interval as the buffer is emptied upon read-out. If no data is available in the buffer, the
    /// sensor returns a NACK. To avoid a NACK response, [`GetDataReady`] can be issued to check
    /// data status. The I2C master can abort the read transfer with a NACK followed by a STOP
    /// condition after any data byte if the user is not interested in subsequent data.
    ///
    /// May be used while measuring.
    read 0xec05 time_ms=1 ReadMeasurement() -> Measurement;

    /// Command returns a sensor running in periodic measurement mode or low power periodic measurement
    /// mode back to the idle state, e.g. to then allow changing the sensor configuration or to save
    /// power.
    ///
    /// May be used while measuring.
    send 0x3f86 time_ms=500 StopPeriodicMeasurement();

    /// Setting the temperature offset of the SCD4x inside the customer device allows the user to
    /// optimize the RH and T output signal. The temperature offset can depend on several factors such
    /// as the SCD4x measurement mode, self-heating of close components, the ambient temperature and
    /// air flow. Thus, the SCD4x temperature offset should be determined after integration into the
    /// final device and under its typical operating conditions (including the operation mode to be
    /// used in the application) in thermal equilibrium. By default, the temperature offset is set to 4
    /// °C. To save the setting to the EEPROM, the [`PersistSettings`] command may be issued.
    write 0x241d time_ms=1 SetTemperatureOffset(TemperatureOffset);

    /// See [`SetTemperatureOffset`] for details.
    read 0x2318 time_ms=1 GetTemperatureOffset() -> TemperatureOffset;

    /// Reading and writing the sensor altitude must be done while the SCD4x is in idle mode.
    /// Typically, the sensor altitude is set once after device installation. To save the setting to
    /// the EEPROM, the [`PersistSettings`] command must be issued. The default sensor altitude value
    /// is set to 0 meters above sea level. Valid input values are between 0 - 3000 m.
    write 0x2427 time_ms=1 SetSensorAltitude(SensorAltitude);

    /// See [`SetSensorAltitude`] for details.
    read 0x2322 time_ms=1 GetSensorAltitude() -> SensorAltitude;

    /// Sets the ambient pressure value. The ambient pressure can be used for pressure
    /// compensation in the CO2 sensor. Setting an ambient pressure overrides any pressure compensation
    /// based on a previously set sensor altitude. Use of this command is recommended for applications
    /// experiencing significant ambient pressure changes to ensure CO2 sensor accuracy. Valid input
    /// values are between 700 to 1200 hPa. The default value is 1013 hPa.
    ///
    /// May be used while measuring.
    write 0xe000 time_ms=1 SetAmbientPressure(AmbientPressure);

    /// See [`SetAmbientPressure`] for details.
    ///
    /// May be used while measuring.
    read 0xe000 time_ms=1 GetAmbientPressure() -> AmbientPressure;

    /// Perform forced recalibration (FRC) of the CO2 signal. Refer to the datasheet for additional
    /// information on how this is to be used.
    write_read 0x362f time_ms=400 PerformForcedRecalibration(TargetCo2Concentration) -> Co2Correction;

    /// Sets the status of the CO2 sensor automatic self-calibration (ASC). The CO2 sensor supports
    /// automatic self-calibration (ASC) for long-term stability of the CO2 output. This feature can be
    /// enabled or disabled. By default, it is enabled.
    write 0x2416 time_ms=1 SetAutomaticSelfCalibrationEnabled(AutomaticSelfCalibrationConfig);

    /// See [`SetAutomaticSelfCalibrationEnabled`] for details.
    read 0x2313 time_ms=1 GetAutomaticSelfCalibrationEnabled() -> AutomaticSelfCalibrationConfig;

    /// Sets the value of the ASC baseline target, i.e. the CO2 concentration in ppm which the ASC
    /// algorithm will assume as lower-bound background to which the SCD4x is exposed to regularly
    /// within one ASC period of operation. To save the setting to the EEPROM, the [`PersistSettings`]
    /// command must be issued subsequently.
    write 0x243a time_ms=1 SetAutomaticSelfCalibrationTarget(AutomaticSelfCalibrationTarget);

    /// Gets the value of the ASC baseline target. See [`SetAutomaticSelfCalibrationTarget`] for
    /// details.
    read 0x233f time_ms=1 GetAutomaticSelfCalibrationTarget() -> AutomaticSelfCalibrationTarget;

    /// To enable use-cases with a constrained power budget, the SCD4x features a low power periodic
    /// measurement mode with a signal update interval of approximately 30 seconds.
    send 0x21ac time_ms=0 StartLowPowerPeriodicMeasurement();

    /// Polls the sensor for whether data from a periodic or single shot measurement is ready to be
    /// read out.
    ///
    /// May be used while measuring.
    read 0xe4b8 time_ms=1 GetDataReady() -> DataReady;

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
    send 0x3615 time_ms=800 PersistSettings();

    /// Reading out the serial number can be used to identify the chip and to verify the presence of
    /// the sensor.
    read 0x3682 time_ms=1 GetSerialNumber() -> SerialNumber;

    /// This command can be used as an end-of-line test to check the sensor functionality.
    read 0x3615 time_ms=10_000 PerformSelfTest() -> SelfTestResult;

    /// Resets all configuration settings stored in the EEPROM and erases the FRC and ASC algorithm
    /// history.
    send 0x3632 time_ms=1_200 FactoryReset();

    /// Reinitializes the sensor by reloading user settings from EEPROM. The sensor must be in the idle
    /// state before sending the reinit command. If the reinit command does not trigger the desired
    /// re-initialization, a power-cycle should be applied to the SCD4x.
    send 0x3646 time_ms=30 Reinit();

    /// Reads out the SCD4x sensor variant (e.g. SCD40, SCD41 or SCD43).
    read 0x202f time_ms=1 GetSensorVariant() -> SensorVariantResult;
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-scd41", crate::devices::sensirion::scd41::SCD41Command),
        ("sensirion-scd43", crate::devices::sensirion::scd43::SCD43Command),
    ];

    /// On-demand measurement of CO2 concentration, relative humidity and temperature. The sensor
    /// output is read out by using the [`ReadMeasurement`] command.
    send 0x219d time_ms=1 MeasureSingleShot(); // NOTE: Adjusted time to 1ms; datasheet specifies 5_000ms but we want to allow polling manually using [`GetDataReady`].

    /// On-demand measurement of relative humidity and temperature only, significantly reduces power
    /// consumption. The sensor output is read out by using the read_measurement command. CO2 output is
    /// returned as 0 ppm.
    send 0x2196 time_ms=1 MeasureSingleShotRhtOnly(); // NOTE: Adjusted time to 1ms; datasheet specifies 50ms but we want to allow polling manually using [`GetDataReady`].

    /// Put the sensor from idle to sleep to reduce current consumption. Can be used to power down when
    /// operating the sensor in power-cycled single shot mode.
    send 0x36e0 time_ms=1 PowerDown();

    /// Wake up the sensor from sleep mode into idle mode. Note that the SCD4x does not acknowledge the
    /// wake_up command. The sensor's idle state after wake up can be verified by reading out the
    /// serial number.
    send 0x36f6 time_ms=30 WakeUp();

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
    write 0x2445 time_ms=1 SetAutomaticSelfCalibrationInitialPeriod(AutomaticSelfCalibrationInitialPeriod);

    /// See [`SetAutomaticSelfCalibrationInitialPeriod`] for details.
    read 0x2340 time_ms=1 GetAutomaticSelfCalibrationInitialPeriod() -> AutomaticSelfCalibrationInitialPeriod;

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
    write 0x244e time_ms=1 SetAutomaticSelfCalibrationStandardPeriod(AutomaticSelfCalibrationStandardPeriod);

    /// See [`SetAutomaticSelfCalibrationStandardPeriod`] for details.
    read 0x234b time_ms=1 GetAutomaticSelfCalibrationStandardPeriod() -> AutomaticSelfCalibrationStandardPeriod;
}
