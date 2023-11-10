use embedded_devices_derive::simple_device_register;

pub const MANUFACTURER_ID_VALID: u16 = 0x0054;

/// The device-id and revision register.
#[simple_device_register(device = super::MCP9808, address = 0b0110, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ManufacturerId {
    /// The Manufacturer ID for the MCP9808 is `0x0054`.
    manufacturer_id: u16,
}
