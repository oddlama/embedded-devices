use super::{MCP9808RegisterProviderAsync, MCP9808RegisterProviderSync};

use embedded_registers::register;

pub const DEVICE_ID_VALID: u8 = 0x04;

/// The device-id and revision register.
#[register(provider = MCP9808, address = 0b0111, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceIdRevision {
    /// The Device ID for the MCP9808 is `0x04`.
    device_id: u8,
    /// The revision begins with 0x00 for the first release, with the number
    /// being incremented as revised versions are released.
    device_revision: u8,
}
