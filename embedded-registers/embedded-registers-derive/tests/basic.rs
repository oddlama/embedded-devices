use embedded_registers::register;
use embedded_registers::Register;

//#[derive(Bitfields, Clone, PartialEq, Eq, Debug, Format)]
//#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
#[derive(Clone, PartialEq, Eq, Debug)]
#[register(address = 0x04, read)]
struct ConfigBitfield {
    //#[bondrewd(bit_length = 5)]
    something: u8,
}

#[test]
fn example_register() {}
