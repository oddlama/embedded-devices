use defmt::Format;

const PRIMARY_ADDRESS: u8 = 0b1110110;
const SECONDARY_ADDRESS: u8 = 0b1110111;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// Primary device address 0b1110110 (SDO connected to GND).
    Primary,
    /// Secondary device address 0b1110111 (SDO connected to V_DDIO).
    Secondary,
    /// Custom address not directly supported by the device, but may be useful
    /// when using address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Primary => PRIMARY_ADDRESS,
            Address::Secondary => SECONDARY_ADDRESS,
            Address::Custom(x) => x,
        }
    }
}
