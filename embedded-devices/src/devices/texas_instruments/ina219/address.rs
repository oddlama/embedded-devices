const ADDRESS_A0_GND_A1_GND: u8 = 0b1000000;
const ADDRESS_A0_VCC_A1_GND: u8 = 0b1000001;
const ADDRESS_A0_SDA_A1_GND: u8 = 0b1000010;
const ADDRESS_A0_SCL_A1_GND: u8 = 0b1000011;
const ADDRESS_A0_GND_A1_VCC: u8 = 0b1000100;
const ADDRESS_A0_VCC_A1_VCC: u8 = 0b1000101;
const ADDRESS_A0_SDA_A1_VCC: u8 = 0b1000110;
const ADDRESS_A0_SCL_A1_VCC: u8 = 0b1000111;
const ADDRESS_A0_GND_A1_SDA: u8 = 0b1001000;
const ADDRESS_A0_VCC_A1_SDA: u8 = 0b1001001;
const ADDRESS_A0_SDA_A1_SDA: u8 = 0b1001010;
const ADDRESS_A0_SCL_A1_SDA: u8 = 0b1001011;
const ADDRESS_A0_GND_A1_SCL: u8 = 0b1001100;
const ADDRESS_A0_VCC_A1_SCL: u8 = 0b1001101;
const ADDRESS_A0_SDA_A1_SCL: u8 = 0b1001110;
const ADDRESS_A0_SCL_A1_SCL: u8 = 0b1001111;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pin {
    /// Address pin connected to GND
    Gnd,
    /// Address pin connected to VCC
    Vcc,
    /// Address pin connected to SDA
    Sda,
    /// Address pin connected to SCL
    Scl,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Address {
    /// Address selection pins A0 and A1 tied to specific pins
    A0A1(Pin, Pin),
    /// Custom address not directly supported by the device, but may be useful
    /// when using I2C address translators.
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(address: Address) -> Self {
        match address {
            Address::A0A1(Pin::Gnd, Pin::Gnd) => ADDRESS_A0_GND_A1_GND,
            Address::A0A1(Pin::Vcc, Pin::Gnd) => ADDRESS_A0_VCC_A1_GND,
            Address::A0A1(Pin::Sda, Pin::Gnd) => ADDRESS_A0_SDA_A1_GND,
            Address::A0A1(Pin::Scl, Pin::Gnd) => ADDRESS_A0_SCL_A1_GND,
            Address::A0A1(Pin::Gnd, Pin::Vcc) => ADDRESS_A0_GND_A1_VCC,
            Address::A0A1(Pin::Vcc, Pin::Vcc) => ADDRESS_A0_VCC_A1_VCC,
            Address::A0A1(Pin::Sda, Pin::Vcc) => ADDRESS_A0_SDA_A1_VCC,
            Address::A0A1(Pin::Scl, Pin::Vcc) => ADDRESS_A0_SCL_A1_VCC,
            Address::A0A1(Pin::Gnd, Pin::Sda) => ADDRESS_A0_GND_A1_SDA,
            Address::A0A1(Pin::Vcc, Pin::Sda) => ADDRESS_A0_VCC_A1_SDA,
            Address::A0A1(Pin::Sda, Pin::Sda) => ADDRESS_A0_SDA_A1_SDA,
            Address::A0A1(Pin::Scl, Pin::Sda) => ADDRESS_A0_SCL_A1_SDA,
            Address::A0A1(Pin::Gnd, Pin::Scl) => ADDRESS_A0_GND_A1_SCL,
            Address::A0A1(Pin::Vcc, Pin::Scl) => ADDRESS_A0_VCC_A1_SCL,
            Address::A0A1(Pin::Sda, Pin::Scl) => ADDRESS_A0_SDA_A1_SCL,
            Address::A0A1(Pin::Scl, Pin::Scl) => ADDRESS_A0_SCL_A1_SCL,
            Address::Custom(x) => x,
        }
    }
}
