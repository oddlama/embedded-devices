const ADDRESS_SEN6X: u8 = 0x6b;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
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
            Address::Default => ADDRESS_SEN6X,
            Address::Custom(x) => x,
        }
    }
}
