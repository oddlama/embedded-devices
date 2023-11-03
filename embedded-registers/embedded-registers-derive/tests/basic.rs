#![feature(generic_arg_infer)]

use embedded_registers::register;

#[register(address = 0x04, read)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct Configuration {
    //#[bondrewd(bit_length = 5)]
    something1: u8,
    //#[bondrewd(bit_length = 5)]
    something2: u8,
}

#[test]
fn example_register() {}
