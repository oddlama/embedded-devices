use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{Pressure, Ratio, ThermodynamicTemperature},
    pressure::pascal,
    ratio::{part_per_million, percent},
    thermodynamic_temperature::degree_celsius,
};

interface_objects! {
    /// Measurement output from the STCC4 sensor.
    ///
    /// Temperature and humidity values are provided as received from the internal SHT4x sensor.
    struct Measurement(size=8) {
        /// CO₂ concentration in ppm (signed integer encoding, i16 range)
        raw_co2_concentration: u16 = 0 => {
            quantity: Ratio,
            unit: part_per_million,
            from_raw: |x| (x as i16) as f64,
            into_raw: |x| x as i16 as u16,
        },
        /// Ambient temperature from internal SHT4x, T[°C] = 175 × ticks / (2¹⁶ − 1) − 45
        raw_temperature: u16 = 0 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            from_raw: |x| -45.0 + (175.0 * x as f64) / 65535.0,
            into_raw: |x| ((x + 45.0) * 65535.0 / 175.0) as u16,
        },
        /// Ambient relative humidity from internal SHT4x, RH[%] = 125 × ticks / 2¹⁶ − 6
        raw_relative_humidity: u16 = 0 => {
            quantity: Ratio,
            unit: percent,
            from_raw: |x| -6.0 + (125.0 * x as f64) / 65536.0,
            into_raw: |x| ((x + 6.0) * 65536.0 / 125.0) as u16,
        },
        /// Sensor status word. Bit 14 (2nd MSB of high byte) = 1 when in testing mode.
        status: u16 = 0,
    }

    /// RHT compensation values for the STCC4.
    ///
    /// Use when the SHT4x is not connected through the STCC4 I²C controller interface.
    struct RhtCompensation(size=4) {
        /// Temperature for RHT compensation, T[°C] = 175 × ticks / (2¹⁶ − 1) − 45.
        /// Default: 25 °C → 26214
        raw_temperature: u16 = 26214 => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            from_raw: |x| -45.0 + (175.0 * x as f64) / 65535.0,
            into_raw: |x| ((x + 45.0) * 65535.0 / 175.0) as u16,
        },
        /// Relative humidity for RHT compensation, RH[%] = 125 × ticks / 2¹⁶ − 6.
        /// Default: 50 %RH → 29360
        raw_relative_humidity: u16 = 29360 => {
            quantity: Ratio,
            unit: percent,
            from_raw: |x| -6.0 + (125.0 * x as f64) / 65536.0,
            into_raw: |x| ((x + 6.0) * 65536.0 / 125.0) as u16,
        },
    }

    /// Pressure compensation value for the STCC4. Input range: 40'000–110'000 Pa.
    struct PressureCompensation(size=2) {
        /// Ambient pressure, LSB = 2 Pa. Default 101'300 Pa → 50650.
        raw_pressure: u16 = 50650 => {
            quantity: Pressure,
            unit: pascal,
            lsb: 2f64 / 1f64,
        },
    }

    /// Target CO₂ concentration for forced recalibration.
    struct TargetCo2Concentration(size=2) {
        /// Target CO₂ concentration. LSB = 1 ppm. Accepted range: 0–32'000 ppm.
        raw_target_co2_concentration: u16 = 400 => {
            quantity: Ratio,
            unit: part_per_million,
            lsb: 1f64 / 1f64,
        },
    }

    /// Result of the [`PerformForcedRecalibration`] command.
    struct ForcedRecalibrationResult(size=2) {
        /// Correction value from FRC. Applied correction = output − 32768 ppm.
        ///
        /// If the recalibration failed, this value is `0xFFFF`.
        raw_correction: u16 = 0 => {
            quantity: Ratio,
            unit: part_per_million,
            from_raw: |x| (x as f64) - 32768.0,
            into_raw: |x| (x + 32768.0) as u16,
        },
    }

    /// Result of the [`PerformFactoryReset`] command.
    struct FactoryResetResult(size=2) {
        /// Result word. `0x0000` = pass, `0xFFFF` = command failed.
        result: u16 = 0,
    }

    /// Result of the [`PerformSelfTest`] command.
    struct SelfTestResult(size=2) {
        /// Self-test result bitfield. `0x0000` or `0x0010` = pass.
        ///
        /// Non-zero values (other than `0x0010`) indicate a failure:
        /// - Bit 0: supply voltage out of range
        /// - Bits 3:1: debugging information (contact Sensirion)
        /// - Bit 4: SHT4x not connected through STCC4 I²C controller interface
        /// - Bits 6:5: memory error
        result: u16 = 0,
    }

    /// Product ID and unique serial number returned by [`GetProductId`].
    struct ProductId(size=12) {
        /// 4-byte product ID. Expected value for STCC4: `0x0901018A`.
        product_id: [u8; 4],
        /// 8-byte unique serial number.
        serial_number: [u8; 8],
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-stcc4", crate::devices::sensirion::stcc4::STCC4Command),
    ];

    /// Starts continuous CO₂ measurement with a 1 s sampling interval.
    send 0x218B time_ms=0 StartContinuousMeasurement();

    /// Stops the continuous measurement and puts the sensor into idle mode.
    ///
    /// During the execution time the sensor will not acknowledge its I²C address.
    ///
    /// May be used while measuring.
    send 0x3F86 time_ms=1200 StopContinuousMeasurement();

    /// Reads out the measurement data. The measurement buffer is emptied upon read-out.
    ///
    /// The sensor responds with a NACK if no measurement data is available yet.
    ///
    /// May be used while measuring.
    read 0xEC05 time_ms=1 ReadMeasurement() -> Measurement;

    /// Writes RHT values used for CO₂ compensation.
    ///
    /// Use only when the SHT4x is **not** connected through the STCC4 I²C controller interface.
    /// Written values take effect after the next measurement interval. Power cycling resets the
    /// values to the defaults (25 °C, 50 %RH).
    ///
    /// May be used while measuring.
    write 0xE000 time_ms=1 SetRhtCompensation(RhtCompensation);

    /// Writes the pressure value used for CO₂ compensation.
    ///
    /// Written value takes effect after the next measurement interval. Power cycling resets to the
    /// default value of 101'300 Pa. Input range: 40'000–110'000 Pa.
    ///
    /// May be used while measuring.
    write 0xE016 time_ms=1 SetPressureCompensation(PressureCompensation);

    /// Performs an on-demand single-shot measurement of CO₂ concentration.
    ///
    /// See [`ReadMeasurement`] to retrieve the result after this command completes.
    send 0x219D time_ms=500 MeasureSingleShot();

    /// Sets the sensor from idle mode into sleep mode.
    send 0x3650 time_ms=1 EnterSleepMode();

    /// Conditions the sensor to improve CO₂ sensing performance after idle or power-off periods
    /// longer than 3 hours.
    send 0x29BC time_ms=22000 PerformConditioning();

    /// Resets the FRC and ASC algorithm history and re-enables the initial bypass phase.
    ///
    /// Returns `0x0000` on success or `0xFFFF` if the command failed.
    read 0x3632 time_ms=90 PerformFactoryReset() -> FactoryResetResult;

    /// Performs an on-chip self-test. A successful test returns `0x0000` or `0x0010`.
    read 0x278C time_ms=360 PerformSelfTest() -> SelfTestResult;

    /// Enables testing mode, pausing the ASC algorithm. The sensor is in testing mode when bit 14
    /// (2nd MSB) of the status word in [`ReadMeasurement`] output is set.
    ///
    /// May be used while measuring.
    send 0x3FBC time_ms=0 EnableTestingMode();

    /// Disables testing mode, resuming the ASC algorithm.
    ///
    /// May be used while measuring.
    send 0x3F3D time_ms=0 DisableTestingMode();

    /// Performs forced recalibration (FRC) to a known target CO₂ concentration.
    ///
    /// Returns the signed FRC correction (output − 32768 ppm), or `0xFFFF` if the command failed.
    write_read 0x362F time_ms=90 PerformForcedRecalibration(TargetCo2Concentration) -> ForcedRecalibrationResult;

    /// Returns the 32-bit product ID and 64-bit unique serial number.
    read 0x365B time_ms=1 GetProductId() -> ProductId;
}

define_sensirion_commands! {
    id_len 1;
    marker [
        ("sensirion-stcc4", crate::devices::sensirion::stcc4::STCC4Command),
    ];

    /// Wakes the sensor from sleep into idle mode.
    ///
    /// Note: the payload byte `0x00` is **not acknowledged** by the sensor. Use
    /// [`GetProductId`] afterwards to confirm the sensor is awake.
    send 0x00 time_ms=5 ExitSleepMode();
}
