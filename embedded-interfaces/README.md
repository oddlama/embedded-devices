# embedded-interfaces

A comprehensive framework for building type-safe and ergonomic embedded device
drivers. Includes traits for communication protocols, codecs, and abstractions
for register and command based devices. Fully `no_std` compatible, and suitable
for both sync and async.

## Features

- 🗜️ **Bit-packed data structures** via the `interface_objects!` macro for ergonomic zero-cost packed struct definitions
- 🧮 **Customizable bit patterns** for non-contiguous field layouts
- 🧬 **Flexible enum definitions** supporting ranges, wildcards, and value capture
- 🚌 **Protocol abstraction** of register-based and command-based access models on any supported bus
- 🧵 **Supports both sync and async**, simultaneously if needed
- 🧪 **Physical quantities** and units can directly be specified in struct/register definitions

## Quick Start

This crate provides two main abstractions, the `RegisterInterface` and
`CommandInterface`. By default, we provide implementations for any I2C or SPI
bus from `embedded-hal` or `embedded-hal-async`. These interfaces can be used
to respectively read and write registers or execute arbitrary commands.

- Commands are a more abstract interface which allows downstream drivers to
  represent arbitrary combinations of sending data, reading data and waiting.
- Registers are data that can be read from or written to the device via a
  register pointer or address. The standard codecs that describe how this address
  and data needs to be communicated to the device are covered by this crate.

Finally, a macro called `interface_objects!` is provided by this crate to simplify the
definition of registers and data for commands which often have very specific
bit-packed layouts. It will be explained in the following section.

## Bit-packed structs

The `interface_objects!` macro can be used to define bit-packed structs and
registers. It automatically generates:

- **Packed structs** with exact memory layout control
- **Unpacked variants** for easier access and manipulation
- **Accessor** methods that don't require full unpacking/packing
- **Unit conversion functions** when physical units are specified

### Basic Struct Definition

Define a simple bit-packed struct with automatic field packing:

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    /// A basic struct
    struct BasicStruct(size = 2) {
        /// Some documentation for the field
        field1: u8,
        /// Another field
        field2: u8,
    }
}
```

This defines two types `BasicStruct([u8; 2])` and `BasicStructUnpacked`, where
the former directly wraps the underlying data array while the latter contains
the actual unpacked fields.

The structs can be converted into one another by using `.pack()` and
`.unpack()`. The packed representation also gets specific field accessor
functions like `.read_field1()` or `.write_field1(123)`.

Doc-comments will be transferred to the actual types while automatically adding
generated information like the resulting bit range or default value (if given).

### Defaults, field size control and reserved fields

You can directly associate defaults to each field and control the desired size
in bits. By eliding the field name with an underscore, the field will become a
reserved field that is ignored in packing or unpacking operations. No accessors
will be generated for it.

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    struct BitFields(size = 1) {
        flag1: bool = true,      // 1-bit bool
        flag2: bool = false,     // 1-bit bool
        counter: u8{4} = 0b1010, // 4-bit field
        _: u8{2},                // 2-bit reserved field
    }
}
```

### Custom ranges and bit Patterns

Fields can be comprised of arbitrary bits without order, even of non-contiguous
bit layouts.

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    struct Interleaved(size = 2) {
        // Interleaved bits: even positions for one field, odd for another
        field1: u8[0,2,4,6,8,10,12,14] = 0x55,
        field2: u8[1,3,5,7,9,11,13,15] = 0xAA,
    }
}
```

Fields without explicit bit ranges will automatically use the next N bits after
the highest previously used bit. The macro will ensure that all bits are used
exactly once at compile time.

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    struct Ranges(size = 2) {
        f3: u8[4..12] // 8 specific bits
        f1: u8[0..4]  // The first 4 bits
        f2: u8{4}     // The next 4 bits after the highest used until now (so 12,13,14,15)
    }
}
```

### Flexible enums

You can define enums with a specific bit-width and powerful value patterns:

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    // Underlying type must be some unsigned type, and exact bit width must be given
    enum Status: u8{4} {
        0 Off,             // Single value
        1..=3 Low(u8),     // Value range, captures the specific value
        4..8 Medium,       // Exclusive range, doesn't capture the value. Packing will use lowest associated value.
        8|10|12..16 High,  // Specific values
        _ Invalid(u8),     // Wildcard catch-all
    }

    struct Device(size = 1) {
        mode: Status = Status::Off,  // Enums can be used directly in structs, the size is automatically known.
        _: u8{4},                    // Reserved bits
    }
}
```

### Nested Structures

Bit-packed structures can be nested into other structures. The resulting
bit-ranges will be fully resolved at compile time.

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    struct Point(size = 2) {
        x: u8 = 10,
        y: u8 = 20,
    }

    struct Rectangle(size = 4) {
        top_left: PointUnpacked = PointUnpacked { x: 1, y: 2 },
        bottom_right: PointUnpacked = PointUnpacked { x: 3, y: 4 },
    }
}
```

### Arrays

Arrays and nested arrays of primitive types work similarly, but arrays of
structs or enums are currently not supported:

```rust
use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    struct DataPacket(size = 5) {
        data: [u8; 4] = [0x01, 0x02, 0x03, 0x04],
        flags: [bool; 8] = [true, false, true, false, true, false, true, false],
    }
}
```

### Physical Units Integration

You can directly associate physical quantities from the `uom` crate with any
`raw_*` fields. The smallest representable value must be provided as a rational
`a / b` to allow the macro to convert between the raw and typed representation
automatically. For complex fields the conversion functions can also be given
directly.

New accessors without the `raw_` prefix will also be generated to allow
convenient access, for example through a `.read_temperature()` or
`.write_temperature()` function.

```rust
use embedded_interfaces::codegen::interface_objects;
use uom::si::f64::{Pressure, ThermodynamicTemperature};
use uom::si::pressure::pascal;
use uom::si::thermodynamic_temperature::degree_celsius;

interface_objects! {
    struct Units(size = 4) {
        raw_temperature: u16 = 75 {
            quantity: ThermodynamicTemperature,
            unit: degree_celsius,
            lsb: 1f64 / 128f64,
        },
        raw_pressure: u16 = 12 {
            quantity: Pressure,
            unit: pascal,
            from_raw: |x| (x as f64 + 7.0) * 31.0,
            into_raw: |x| (x / 31.0 - 7.0) as u16,
        },
    }
}
```
