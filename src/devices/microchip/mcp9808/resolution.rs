use bondrewd::BitfieldEnum;
use embedded_devices_derive::simple_device_register;

/// Temperature resolution. Affects both sensor accuracy and conversion time.
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum TemperatureResolution {
    /// +0.5°C (t_CONV = 30 ms typical)
    Deg_0_5C = 0b00,
    /// +0.25°C (t_CONV = 65 ms typical)
    Deg_0_25C = 0b01,
    /// +0.125°C (t_CONV = 130 ms typical)
    Deg_0_125C = 0b10,
    /// +0.0625°C (power-up default, t_CONV = 250 ms typical)
    #[default]
    Deg_0_0625C = 0b11,
}

/// The device resolution register.
///
/// This register allows the user to change the sensor resolution.
/// The Power-on Reset default resolution is +0.0625°C.
#[simple_device_register(device = super::MCP9808, address = 0b1000, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Resolution {
    #[bondrewd(bit_length = 6, reserve)]
    #[allow(dead_code)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub temperature_resolution: TemperatureResolution,
}
