use embedded_devices_derive::device_register;
use embedded_registers::register;

use crate::devices::sensirion::{SensirionI2cCodec, sensirion_command};

/// Starts a continuous measurement. After starting the measurement, it takes some time (~1.1s)
/// until the first measurement results are available. You could poll with the command [`DataReady`]
/// to check when the results are ready to be read.
#[device_register(super::SEN60)]
#[register(address = 0x2152, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartContinuousMeasurement {}
sensirion_command!(StartContinuousMeasurement, 1, false);

/// Stops the measurement and returns to idle mode.
#[device_register(super::SEN60)]
#[register(address = 0x3F86, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StopMeasurement {}
sensirion_command!(StopMeasurement, 1000, true);

/// This command can be used to check if new measurement results are ready to read. The data ready
/// flag is automatically reset after reading the measurement values
#[device_register(super::SEN60)]
#[register(address = 0xE4B8, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DataReady {
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
    /// Unequal to 0 if data is ready, 0 if not. When no measurement is running, 0 will be
    /// returned.
    #[register(default = 0)]
    #[bondrewd(bit_length = 11, reserve)]
    pub data_ready: u16,
}
sensirion_command!(DataReady, 1, true);

/// Returns the measured values. The command [`DataReady`] can be used to check if new data
/// is available since the last read operation. The measurement data can only be read out once per
/// signal update interval, as the buffer is emptied upon read-out. If no data is available in the
/// buffer, the sensor returns a NACK. To avoid a NACK response, the Get Data Ready SEN60 can be
/// issued to check data status. The I2C controller can abort the read transfer with a NACK
/// followed by a STOP condition after any data byte if the user is not interested in the
/// subsequent data
#[device_register(super::SEN60)]
#[register(address = 0xEC05, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 8)]
pub struct MeasuredValuesMassConcentrationOnly {
    /// PM1 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm1: u16,
    /// PM2.5 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm2_5: u16,
    /// PM4 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm4: u16,
    /// PM10 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm10: u16,
}
sensirion_command!(MeasuredValuesMassConcentrationOnly, 1, true);

/// Returns the measured values. The command [`DataReady`] can be used to check if new data
/// is available since the last read operation. The measurement data can only be read out once per
/// signal update interval, as the buffer is emptied upon read-out. If no data is available in the
/// buffer, the sensor returns a NACK. To avoid a NACK response, the Get Data Ready SEN60 can be
/// issued to check data status. The I2C controller can abort the read transfer with a NACK
/// followed by a STOP condition after any data byte if the user is not interested in the
/// subsequent data
#[device_register(super::SEN60)]
#[register(address = 0xEC05, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 18)]
pub struct MeasuredValues {
    /// PM1 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm1: u16,
    /// PM2.5 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm2_5: u16,
    /// PM4 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm4: u16,
    /// PM10 mass concentration, LSB = 0.1 µg/m³
    #[register(default = u16::MAX)]
    pub mass_concentration_pm10: u16,
    /// PM0.5 volumetric number concentration, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_density_pm0_5: u16,
    /// PM1 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_density_pm1: u16,
    /// PM2.5 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_density_pm2_5: u16,
    /// PM4 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_density_pm4: u16,
    /// PM10 volumetric number density, LSB = 0.1 particles/cm³
    #[register(default = u16::MAX)]
    pub number_density_pm10: u16,
}
sensirion_command!(MeasuredValues, 1, true);

/// Gets the serial number from the device.
#[device_register(super::SEN60)]
#[register(address = 0x3682, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 6)]
pub struct SerialNumber {
    /// 6-byte serial number
    pub serial_number: [u8; 6],
}
sensirion_command!(SerialNumber, 1, true);

/// Reads the current device status.
///
/// Note: The status flags of type `Error` are sticky, i.e. they are not cleared automatically even
/// if the error condition no longer exists. So, they can only be cleared manually through a reset,
/// either by calling [`DeviceReset`] or through a power cycle. All other flags are not sticky,
/// i.e. they are cleared automatically if the trigger condition disappears.
#[device_register(super::SEN60)]
#[register(address = 0xE00B, mode = "r", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceStatus {
    #[bondrewd(bit_length = 11, reserve)]
    #[allow(dead_code)]
    pub reserved0: u16,
    /// Fan is switched on, but 0 RPM is measured for multiple consecutive measurement intervals.
    /// This can occur if the fan is mechanically blocked or broken. Note that the measured values
    /// are most likely wrong if this error is reported.
    ///
    /// Can occur only in measurement mode.
    #[register(default = false)]
    pub fan_error: bool,
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
    /// Fan is switched on, but its speed is more than 10% off the target speed for multiple
    /// consecutive measurement intervals. During the first 10 seconds after starting the
    /// measurement, the fan speed is not checked (settling time). Very low or very high ambient
    /// temperature could trigger this warning during startup. If this flag is set constantly, it
    /// might indicate a problem with the power supply or with the fan, and the measured PM values
    /// might be wrong. This flag is automatically cleared as soon as the measured speed is within
    /// 10% of the target speed or when leaving the measure mode.
    ///
    /// Can occur only in measurement mode.
    #[register(default = false)]
    pub fan_speed_warning: bool,
    #[bondrewd(reserve)]
    #[allow(dead_code)]
    pub reserved2: bool,
}
sensirion_command!(DeviceStatus, 1, true);

/// Executes a reset on the device. This has the same effect as a power cycle.
#[device_register(super::SEN60)]
#[register(address = 0x3F8D, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct DeviceReset {}
sensirion_command!(DeviceReset, 1, true);

/// This command triggers fan cleaning. The fan is set to the maximum speed for 10 seconds and then
/// automatically stopped.
///
/// Note: Wait at least 10s after this command before starting a measurement.
#[device_register(super::SEN60)]
#[register(address = 0x3730, mode = "w", i2c_codec = "SensirionI2cCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 0)]
pub struct StartFanCleaning {}
sensirion_command!(StartFanCleaning, 1, false);
