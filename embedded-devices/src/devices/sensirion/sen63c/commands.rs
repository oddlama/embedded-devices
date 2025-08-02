use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{MassConcentration, Ratio, ThermodynamicTemperature},
    mass_concentration::microgram_per_cubic_meter,
    ratio::{part_per_million, percent},
    thermodynamic_temperature::degree_celsius,
};

pub use crate::devices::sensirion::sen6x::commands::{
    ActivateSHTHeater, DeviceReset, GetAmbientPressure, GetCo2SensorAutomaticSelfCalibration, GetDataReady,
    GetProductName, GetSensorAltitude, GetSerialNumber, PerformForcedCo2Recalibration, ReadNumberConcentrationValues,
    SetAmbientPressure, SetCo2SensorAutomaticSelfCalibration, SetSensorAltitude, SetTemperatureAccelerationParameters,
    SetTemperatureOffsetParameters, StartContinuousMeasurement, StartFanCleaning, StopMeasurement,
};

interface_objects! {
    struct MeasuredValues(size=14) {
        /// PM1 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm1: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10_000_000f64,
        },
        /// PM2.5 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm2_5: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10_000_000f64,
        },
        /// PM4 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm4: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10_000_000f64,
        },
        /// PM10 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm10: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10_000_000f64,
        },
        /// Ambient relative humidity, LSB = 0.01%
        raw_relative_humidity: i16 = i16::MAX => {
            quantity: Ratio,
            unit: percent,
            lsb: 1f64 / 100f64,
        },
        /// Ambient temperature, LSB = 0.005°C
        raw_temperature: i16 = i16::MAX => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 200f64,
        },
        /// CO2 concentration, LSB = 1 ppm
        /// Will be u16::MAX for the first 5-6 seconds after startup or reset.
        raw_co2_concentration: u16 = u16::MAX => {
            quantity: Ratio,
            unit: part_per_million,
            lsb: 1f64 / 1f64,
        },
    }

    struct MeasuredRawValues(size=4) {
        /// Raw relative humidity, LSB = 0.01%
        raw_relative_humidity: i16 = i16::MAX => {
            quantity: Ratio,
            unit: percent,
            lsb: 1f64 / 100f64,
        },
        /// Raw temperature, LSB = 0.005°C
        raw_temperature: i16 = i16::MAX => {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 200f64,
        },
    }

    struct DeviceStatus(size=4) {
        _: u16{10},
        /// Fan is switched on, but its speed is more than 10% off the target speed for multiple
        /// consecutive measurement intervals. During the first 10 seconds after starting the
        /// measurement, the fan speed is not checked (settling time). Very low or very high ambient
        /// temperature could trigger this warning during startup. If this flag is set constantly, it
        /// might indicate a problem with the power supply or with the fan, and the measured PM values
        /// might be wrong. This flag is automatically cleared as soon as the measured speed is within
        /// 10% of the target speed or when leaving the measure mode.
        ///
        /// Can occur only in measurement mode.
        fan_speed_warning: bool = false,
        _: u8,
        /// Error related to the CO2 sensor. The CO2 values might be unknown or wrong if this flag is
        /// set, relative humidity and temperature values might be out of specs due to compensation
        /// algorithms depending on CO2 sensor state.
        ///
        /// Can occur only in measurement mode.
        co2_error: bool = false,
        /// Error related to the PM sensor. The particulate matter values might be unknown or wrong if
        /// this flag is set, relative humidity and temperature values might be out of specs due to
        /// compensation algorithms depending on PM sensor state.
        ///
        /// Can occur only in measurement mode.
        pm_error: bool = false,
        _: bool, // hcho_error
        _: bool, // co2_error2
        _: bool, // reserved
        _: bool, // gas_error
        /// Error related to the RH&T sensor. The temperature and humidity values might be unknown or
        /// wrong if this flag is set, and other measured values might be out of specs due compensation
        /// algorithms depending on RH&T sensor values.
        ///
        /// Can occur only in measurement mode.
        rh_t_error: bool = false,
        _: bool, // reserved
        /// Fan is switched on, but 0 RPM is measured for multiple consecutive measurement intervals.
        /// This can occur if the fan is mechanically blocked or broken. Note that the measured values
        /// are most likely wrong if this error is reported.
        ///
        /// Can occur only in measurement mode.
        fan_error: bool = false,
        _: u8{4},
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen63c", crate::devices::sensirion::sen63c::SEN63CCommand),
    ];

    /// Returns the measured values. The command [`GetDataReady`] can be used to check if new data is
    /// available since the last read operation. If no new data is available, the previous values will
    /// be returned. If no data is available at all (e.g. measurement not running for at least one
    /// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
    ///
    /// May be executed during measurement.
    read 0x0471 time_ms=20 ReadMeasuredValues() -> MeasuredValues;

    /// Returns the measured raw values. The command [`GetDataReady`] can be used to check if new data
    /// is available since the last read operation. If no new data is available, the previous values
    /// will be returned. If no data is available at all (e.g. measurement not running for at least one
    /// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
    ///
    /// May be executed during measurement.
    read 0x0492 time_ms=20 ReadMeasuredRawValues() -> MeasuredRawValues;

    /// Reads the current device status.
    ///
    /// Note: The status flags of type `Error` are sticky, i.e. they are not cleared automatically even
    /// if the error condition no longer exists. So, they can only be cleared manually with
    /// [`ReadAndClearDeviceStatus`] or through a reset, either by calling [`DeviceReset`] or through a
    /// power cycle. All other flags are not sticky, i.e. they are cleared automatically if the trigger
    /// condition disappears.
    ///
    /// May be executed during measurement.
    read 0xd206 time_ms=20 ReadDeviceStatus() -> DeviceStatus;

    /// Reads the current device status (like command [`ReadDeviceStatus`]) and clears all
    /// flags afterwards.
    ///
    /// May be executed during measurement.
    read 0xd210 time_ms=20 ReadAndClearDeviceStatus() -> DeviceStatus;
}
