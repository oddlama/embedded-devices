use crate::devices::sensirion::commands::define_sensirion_commands;
use embedded_interfaces::codegen::interface_objects;
use uom::si::{
    f64::{MassConcentration, VolumetricNumberDensity},
    mass_concentration::microgram_per_cubic_meter,
    volumetric_number_density::per_cubic_centimeter,
};

interface_objects! {
    /// Whether data is ready to be read out
    enum DataReadyStatus: u16{11} {
        0 NotReady,
        _ Ready,
    }

    struct DataReady(size=2) {
        _: u8{5},
        /// When no measurement is running, [`DataReadyStatus::NotReady`] will be returned.
        data_ready: DataReadyStatus = DataReadyStatus::NotReady,
    }

    struct MeasuredValuesMassConcentrationOnly(size=8) {
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
    }

    struct MeasuredValues(size=18) {
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
        /// PM0.5 volumetric number concentration, LSB = 0.1 particles/cm³
        raw_number_density_pm0_5: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM1 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_density_pm1: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM2.5 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_density_pm2_5: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM4 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_density_pm4: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
        /// PM10 volumetric number density, LSB = 0.1 particles/cm³
        raw_number_density_pm10: u16 = u16::MAX => {
            quantity: VolumetricNumberDensity,
            unit: per_cubic_centimeter,
            lsb: 1f64 / 10f64,
        },
    }

    struct SerialNumber(size=6) {
        /// 6-byte serial number
        serial_number: [u8; 6],
    }

    struct DeviceStatus(size=2) {
        _: u16{11},
        /// Fan is switched on, but 0 RPM is measured for multiple consecutive measurement intervals.
        /// This can occur if the fan is mechanically blocked or broken. Note that the measured values
        /// are most likely wrong if this error is reported.
        ///
        /// Can occur only in measurement mode.
        fan_error: bool = false,
        _: u8{2},
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
        _: bool,
    }
}

define_sensirion_commands! {
    id_len 2;
    marker [
        ("sensirion-sen60", crate::devices::sensirion::sen60::SEN60Command),
    ];

    /// Starts a continuous measurement. After starting the measurement, it takes some time (~1.1s)
    /// until the first measurement results are available. You could poll with the command
    /// [`GetDataReady`] to check when the results are ready to be read.
    ///
    /// Cannot be executed during measurement.
    send 0x2152 time_ms=1 StartContinuousMeasurement();

    /// Stops the measurement and returns to idle mode.
    ///
    /// May be executed during measurement.
    send 0x3f86 time_ms=1000 StopMeasurement();

    /// This command can be used to check if new measurement results are ready to read. The data ready
    /// flag is automatically reset after reading the measurement values
    ///
    /// May be executed during measurement.
    read 0xe4b8 time_ms=1 GetDataReady() -> DataReady;

    /// Returns the measured values (mass concentration only). The command [`GetDataReady`] can be used to
    /// check if new data is available since the last read operation. The measurement data can only be read
    /// out once per signal update interval, as the buffer is emptied upon read-out. If no data is available
    /// in the buffer, the sensor returns a NACK. To avoid a NACK response, the Get Data Ready SEN60 can be
    /// issued to check data status. The I2C controller can abort the read transfer with a NACK followed by
    /// a STOP condition after any data byte if the user is not interested in the subsequent data
    ///
    /// May be executed during measurement.
    read 0xec05 time_ms=1 ReadMeasuredValuesMassConcentrationOnly() -> MeasuredValuesMassConcentrationOnly;

    /// Returns the measured values (full measurement data). The command [`GetDataReady`] can be used to
    /// check if new data is available since the last read operation. The measurement data can only be read
    /// out once per signal update interval, as the buffer is emptied upon read-out. If no data is available
    /// in the buffer, the sensor returns a NACK. To avoid a NACK response, the Get Data Ready SEN60 can be
    /// issued to check data status. The I2C controller can abort the read transfer with a NACK followed by
    /// a STOP condition after any data byte if the user is not interested in the subsequent data
    ///
    /// May be executed during measurement.
    read 0xec05 time_ms=1 ReadMeasuredValues() -> MeasuredValues;

    /// Gets the serial number from the device.
    ///
    /// May be executed during measurement.
    read 0x3682 time_ms=1 GetSerialNumber() -> SerialNumber;

    /// Reads the current device status.
    ///
    /// Note: The status flags of type `Error` are sticky, i.e. they are not cleared automatically even
    /// if the error condition no longer exists. So, they can only be cleared manually through a reset,
    /// either by calling [`DeviceReset`] or through a power cycle. All other flags are not sticky,
    /// i.e. they are cleared automatically if the trigger condition disappears.
    ///
    /// May be executed during measurement.
    read 0xe00b time_ms=1 ReadDeviceStatus() -> DeviceStatus;

    /// Executes a reset on the device. This has the same effect as a power cycle.
    ///
    /// Cannot be executed during measurement.
    send 0x3f8d time_ms=1 DeviceReset();

    /// This command triggers fan cleaning. The fan is set to the maximum speed for 10 seconds and then
    /// automatically stopped.
    ///
    /// Note: Wait at least 10s after this command before starting a measurement.
    ///
    /// Cannot be executed during measurement.
    send 0x3730 time_ms=1 StartFanCleaning();
}
