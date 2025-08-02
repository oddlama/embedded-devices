use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{MassConcentration, Ratio, ThermodynamicTemperature},
    mass_concentration::microgram_per_cubic_meter,
    ratio::{part_per_billion, percent},
    thermodynamic_temperature::degree_celsius,
};

pub use crate::devices::sensirion::sen6x::commands::{
    ActivateSHTHeater, DeviceReset, GetDataReady, GetNOxAlgorithmTuningParameters, GetProductName, GetSerialNumber,
    GetVOCAlgorithmState, GetVOCAlgorithmTuningParameters, ReadNumberConcentrationValues,
    SetNOxAlgorithmTuningParameters, SetTemperatureAccelerationParameters, SetTemperatureOffsetParameters,
    SetVOCAlgorithmState, SetVOCAlgorithmTuningParameters, StartContinuousMeasurement, StartFanCleaning,
    StopMeasurement,
};

interface_objects! {
    struct MeasuredValues(size=18) {
        /// PM1 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm1: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10f64,
        },
        /// PM2.5 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm2_5: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10f64,
        },
        /// PM4 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm4: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10f64,
        },
        /// PM10 mass concentration, LSB = 0.1 µg/m³
        raw_mass_concentration_pm10: u16 = u16::MAX => {
            quantity: MassConcentration,
            unit: microgram_per_cubic_meter,
            lsb: 1f64 / 10f64,
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
        /// VOC index, LSB = 0.1
        voc_index: i16 = i16::MAX,
        /// NOx index, LSB = 0.1
        /// Will be i16::MAX for the first 10-11 seconds after startup or reset.
        nox_index: i16 = i16::MAX,
        /// Formaldehyde (HCHO) concentration, LSB = 0.1 ppb
        /// Will be u16::MAX for the first 60 seconds after first measurement start after startup or
        /// reset.
        raw_hcho_concentration: u16 = u16::MAX => {
            quantity: Ratio,
            unit: part_per_billion,
            lsb: 1f64 / 10f64,
        },
    }

    struct MeasuredRawValues(size=8) {
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
        /// VOC ticks, LSB = 1
        voc_ticks: u16 = u16::MAX,
        /// NOx ticks, LSB = 1
        /// Will be i16::MAX for the first 10-11 seconds after startup or reset.
        nox_ticks: u16 = u16::MAX,
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
        _: u8, // reserved1
        _: bool, // reserved_co2_error
        /// Error related to the PM sensor. The particulate matter values might be unknown or wrong if
        /// this flag is set, relative humidity and temperature values might be out of specs due to
        /// compensation algorithms depending on PM sensor state.
        ///
        /// Can occur only in measurement mode.
        pm_error: bool = false,
        /// Error related to the formaldehyde sensor. The formaldehyde values might be unknown or wrong
        /// if this flag is set, relative humidity and temperature values might be out of specs due to
        /// compensation algorithms depending on formaldehyde sensor state.
        ///
        /// Can occur only in measurement mode.
        hcho_error: bool = false,
        _: bool, // reserved_co2_error2
        _: bool, // reserved2
        /// Error related to the gas sensor. The VOC index and NOx index might be unknown or wrong if
        /// this flag is set, relative humidity and temperature values might be out of specs due to
        /// compensation algorithms depending on gas sensor state.
        ///
        /// Can occur only in measurement mode.
        gas_error: bool = false,
        /// Error related to the RH&T sensor. The temperature and humidity values might be unknown or
        /// wrong if this flag is set, and other measured values might be out of specs due compensation
        /// algorithms depending on RH&T sensor values.
        ///
        /// Can occur only in measurement mode.
        rh_t_error: bool = false,
        _: bool, // reserved3
        /// Fan is switched on, but 0 RPM is measured for multiple consecutive measurement intervals.
        /// This can occur if the fan is mechanically blocked or broken. Note that the measured values
        /// are most likely wrong if this error is reported.
        ///
        /// Can occur only in measurement mode.
        fan_error: bool = false,
        _: u8{4}, // reserved4
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen68", crate::devices::sensirion::sen68::SEN68Command),
    ];

    /// Returns the measured values. The command [`GetDataReady`] can be used to check if new data is
    /// available since the last read operation. If no new data is available, the previous values will
    /// be returned. If no data is available at all (e.g. measurement not running for at least one
    /// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
    ///
    /// May be executed during measurement.
    read 0x0467 time_ms=20 ReadMeasuredValues() -> MeasuredValues;

    /// Returns the measured raw values. The command [`GetDataReady`] can be used to check if new data
    /// is available since the last read operation. If no new data is available, the previous values
    /// will be returned. If no data is available at all (e.g. measurement not running for at least one
    /// second), all values will be at their upper limit (0xFFFF for u16, 0x7FFF for i16).
    ///
    /// May be executed during measurement.
    read 0x0455 time_ms=20 ReadMeasuredRawValues() -> MeasuredRawValues;

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
