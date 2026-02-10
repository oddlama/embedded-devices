const REGULAR_ADDRESS: u8 = 0x10;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Address {
    /// Regular device address 0x10.
    Regular,
    /// Custom address not directly supported by the device, but may be useful
    /// when using address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Regular => REGULAR_ADDRESS,
            Address::Custom(x) => x,
        }
    }
}
