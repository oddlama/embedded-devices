use defmt::Format;

const ADDRESS_SEN60: u8 = 0x6c;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// Default address
    Default,
    /// Custom address not directly supported by the device, but may be useful
    /// when using I2C address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Default => ADDRESS_SEN60,
            Address::Custom(x) => x,
        }
    }
}
