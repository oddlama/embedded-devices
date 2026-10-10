//! I²C address selection using the ADS7830's A0 and A1 pins.

/// Connection of an address selection pin.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pin {
    /// Address pin connected to GND.
    Gnd,
    /// Address pin connected to VDD.
    Vcc,
}

/// ADS7830 I²C address.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Address {
    /// Address selection pins A0 and A1, in that order.
    A0A1(Pin, Pin),
    /// Custom seven-bit address for use with I²C address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::A0A1(a0, a1) => 0x48 | u8::from(a0 == Pin::Vcc) | (u8::from(a1 == Pin::Vcc) << 1),
            Address::Custom(address) => address,
        }
    }
}
