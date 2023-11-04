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

This crate was designed for the [embedded-devices](https://github.com/oddlama/embedded-devices) crate,
which aims to provide modern async-capable and coherent definitions for many embedded devices.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
# You will need this to define your own bitfield-capable enums
bondrewd = { version = "0.1.14", default_features = false, features = ["derive"] }
embedded-registers = "0.9.0"
```

For a simple register definition example, take a look at this `DeviceId` register
from the MCP9808 temperature sensor:

```rust
#![feature(generic_arg_infer)]

use embedded_registers::register;

#[register(address = 0b111, read)]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct DeviceId {
    device_id: u8,
    revision: u8,
}
```

You may then read the register simply by calling `DeviceId::read_i2c` or `DeviceId::read_i2c_blocking`
(or similarly write to it if you specified `write` in the definition above):

```rust
let reg = DeviceId::read_i2c(&mut i2c, 0x24 /* i2c device address */).await?;
info!("{}", reg);
// Prints: DeviceId ([4, 0]) => DeviceIdBitfield { device_id: 4, revision: 0 }
```

For more information and more complex examples, please refer to the [embededded-registers docs](https://docs.rs/embedded-registers) and the [bondrewd docs](https://docs.rs/bondrewd-derive).
