use defmt::Format;

const ADDRESS: u8 = 0x62;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// default address
    Default,
    /// Custom address not directly supported by the device, but may be useful
    /// when using I2c address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Default => ADDRESS,
            Address::Custom(x) => x,
        }
    }
}
