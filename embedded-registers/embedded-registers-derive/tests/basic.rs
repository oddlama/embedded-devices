#![feature(generic_arg_infer)]

use bondrewd::Bitfields;
use embedded_registers::register;

#[derive(Bitfields, Debug, Clone, PartialEq, Eq, defmt::Format)]
#[bondrewd(default_endianess = "msb", read_from = "lsb0", enforce_bytes = "1")]
pub struct StatusMagnetometer {
    mtm1: bool,
    mtm2: bool,
    mtm3: bool,
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    reserved: u8,
}

#[register(address = 0x04, read)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    #[bondrewd(struct_size = 1)]
    pub status: StatusMagnetometer,
    //#[bondrewd(bit_length = 5)]
    something1: u8,
}

#[test]
fn example_register() {
    let mut reg: Configuration = Default::default();
    reg.write_status(StatusMagnetometer {
        mtm1: true,
        mtm2: true,
        mtm3: true,
        reserved: 0,
    });

    let bitfield = reg.read_all();
    let mut reg2 = reg.clone();
    reg2.write_all(bitfield);
    assert_eq!(reg, reg2);
}
