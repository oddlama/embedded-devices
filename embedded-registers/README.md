# Embedded Registers

[![Crate](https://img.shields.io/crates/v/embedded-registers.svg)](https://crates.io/crates/embedded-registers)
[![API](https://docs.rs/embedded-registers/badge.svg)](https://docs.rs/embedded-registers)

**WARNING: This crate is currently in experimental state, so anything may change at any time.**

This crate provides a procedural macro for effortless definitions of registers
in embedded device drivers. This is automatically generates functions to read/write
the register over I2C and SPI, although it isn't limited to those buses. The
resulting struct may trivially be extended to work with any other similar communication bus.

- Allows defintion of read-only, write-only and read-write registers
- Generates I2C and SPI read/write functions
- Registers are defined as bitfields via [bondrewd](https://github.com/Devlyn-Nelson/Bondrewd).
- Only the accessed bitfield members are decoded, conserving memory and saving on CPU time.
- Supports both async and blocking operation modes

This crate was made primarily for [embedded-devices](https://github.com/oddlama/embedded-devices),
which is a collection of drivers for a variety of different embedded sensors and devices.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
# You will need these to define your own bitfield-capable enums
bondrewd = { version = "0.1.14", default-features = false, features = ["derive"] }
bytemuck = { version = "1.16.3", features = ["derive", "min_const_generics"] }
embedded-registers = "0.9.7"
```

Registers are defined simply by annotating a bondrewd struct with `#[register(address = 0x42, mode = "rw")]`.
Take for example this register definition for the device id of a MCP9808:

```rust
#![feature(generic_arg_infer)]
use embedded_registers::register;

#[register(address = 0b111, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceId {
    device_id: u8,
    revision: u8,
}
```

For more specific usage information and more complex examples, please refer to the [embededded-registers docs](https://docs.rs/embedded-registers) and the [bondrewd docs](https://docs.rs/bondrewd-derive).

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.
Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in this crate by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.
