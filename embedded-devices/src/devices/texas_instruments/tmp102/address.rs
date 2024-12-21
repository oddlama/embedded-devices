use defmt::Format;

const ADDRESS_GND: u8 = 0b1001000; // 0x48
const ADDRESS_VCC: u8 = 0b1001001; // 0x49
const ADDRESS_SDA: u8 = 0b1001010; // 0x4A
const ADDRESS_SCL: u8 = 0b1001011; // 0x4B

#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// Address selection pin connected to GND
    Gnd,
    /// Address selection pin connected to VCC
    Vcc,
    /// Address selection pin connected to SDA
    Sda,
    /// Address selection pin connected to SCL
    Scl,
    /// Custom address not directly supported by the device, but may be useful
    /// when using I2c address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::Gnd => ADDRESS_GND,
            Address::Vcc => ADDRESS_VCC,
            Address::Sda => ADDRESS_SDA,
            Address::Scl => ADDRESS_SCL,
            Address::Custom(x) => x,
        }
    }
}
