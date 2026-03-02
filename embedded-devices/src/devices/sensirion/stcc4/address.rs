const ADDRESS_DEFAULT: u8 = 0x64;
const ADDRESS_ALT: u8 = 0x65;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Address {
    /// Default address (ADDR=GND)
    Default,
    /// Alternative address (ADDR=VDD)
    Alt,
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Default => ADDRESS_DEFAULT,
            Address::Alt => ADDRESS_ALT,
        }
    }
}
