const DEFAULT_ADDRESS: u8 = 0b11000;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Address {
    /// Default device address 0b11000, all address selection pins connected to GND.
    Default,
    /// Alternative device address corresponding to the given pin configuration A2, A1, A0.
    /// The address pins directly correspond to the least significant bits of the address,
    /// so the resulting address is `0b11{a2}{a1}{a0}`.
    Alternative { a2: bool, a1: bool, a0: bool },
    /// Custom address not directly supported by the device, but may be useful
    /// when using I2C address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Default => DEFAULT_ADDRESS,
            Address::Alternative { a2, a1, a0 } => DEFAULT_ADDRESS | (a2 as u8) << 2 | (a1 as u8) << 1 | (a0 as u8),
            Address::Custom(x) => x,
        }
    }
}
