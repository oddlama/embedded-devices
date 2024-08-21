[![Crate](https://img.shields.io/crates/v/embedded-devices.svg)](https://crates.io/crates/embedded-devices)
[![API](https://docs.rs/embedded-devices/badge.svg)](https://docs.rs/embedded-devices)

# Embedded-devices

**WARNING: This crate is currently in experimental state, so anything may change at any time.**

Welcome to the embedded-devices project! Here you'll find a collection of drivers for a variety of different
embedded sensors and devices, all of which are built with this framework which facilitates building drivers
for register based devices. These drivers are designed for `async` use but also support synchronous contexts
via a feature switch. Our goal is to provide feature-complete, up-to-date drivers with an ergonomic interface,
catering both to high-level device functions and low-level register access. Please refer to the list below
for supported devices.

The ecosystem of embedded-rust device drivers seems quite fragmented,
consisting both of high quality implementations but also of several incomplete or outdated drivers,
where only a few have first-class async support. This renders them hard (or even impossible) to use
in embedded frameworks like [embassy](https://github.com/embassy-rs/embassy), which in my opinion would
benefit from having access to more ready-to-use async drivers with frequently updated dependencies.

For the time being, this crate should serve as a proof-of-concept. It shows how sensor and device drivers
can benefit from a common framework, allowing new drivers to be added with ease in order to streamline
future collaborative efforts and solve some of the aforementioned issues.
The main component of our framework is [embedded-registers](https://github.com/oddlama/embedded-registers)
which provides an ergonomic solution to defining and interfacing with device registers over I2C/SPI.

## Supported Devices

Below you will find all supported devices. Please visit their respective documentation links for more information and usage examples.

<!-- TODO: should we order by category and also list the most interesting specs? like accuracy for temp sens, feature matrix style for pressure, humidity, ... -->

| Manufacturer | Device | Interface | Description | Docs |
|---|---|---|---|---|
| Analog Devices | MAX31865 | SPI | Precision temperature converter for RTDs, NTCs and PTCs | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/analog_devices/max31865/index.html) |
| Bosch | BME280 | I2C/SPI | Temperature, pressure and humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bme280/index.html) |
| Bosch | BMP280 | I2C/SPI | Temperature and pressure sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bmp280/index.html) |
| Bosch | BMP390 | I2C/SPI | Temperature and pressure sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bmp390/index.html) |
| Microchip | MCP9808 | I2C | Digital temperature sensor with ±0.5°C (max.) accuracy | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp9808/index.html) |
| Texas Instruments | TMP117 | I2C | Temperature sensor with ±0.1°C to ±0.3°C accuracy depending on the temperature range | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/tmp117/index.html) |

## Architecture

Driver implementations are organized based on the manufacturer name and device name.
Each driver exposes a struct with the name of the device, for example `BME280`. There are no restrictions on the
struct's generics, since devices may require different interface combinations, or own extra pins.

The vast majority of devices use a similar "protocol" over I2C or SPI to read and write registers.
So usually they will have a generic interface and expose a `new_i2c` and/or `new_spi` function to construct
the appropriate object from an interface and address.

### Register based devices

Our [embedded-registers](https://github.com/oddlama/embedded-registers) crate provides a generic way to define registers, and an interface implementation
to allow reading and writing those registers via I2C or SPI.

A register usually refers to a specific memory address (or consecutive memory region) on the device by
specifiying its start address. We also associate each register to a specific device by
specifying a marker trait. This prevents the generated API from accepting registers of unrelated devices.
In the following, we'll have a short look at how to define and work with register based devices.

### Defining a register

First, lets start with a very simple register definition. We will later create a device struct which can
read from and write to this register. So to define our very simple register, we

1. specify which device belongs to using the `#[device_register]` macro (we will later define this `MyDevice`),
2. define the start address and mode (read-only, write-only, read-write) using the `#[register]` macro,
3. define the contents of the register using [bondrewd](https://github.com/Devlyn-Nelson/Bondrewd), a general purpose bitfield macro.

Most devices support a burst-read/write operations, where you can begin reading from address `A` and will automatically receive
values from the consecutive memory region until you stop reading. This means you can define registers with a `size > 1 byte`
and will get the content you expect.

Let's imagine our `MyDevice` had a 2-byte read-write register at device address `0x42(,0x43)`, which contains two `u8` values.
The corresponding register can be defined like this:

```rust
/// Insert explanation of this register from the datasheet.
#[device_register(MyDevice)]
#[register(address = 0x42, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ValueRegister {
    /// Insert explanation of this value from the datasheet.
    pub width: u8,
    /// Insert explanation of this value from the datasheet.
    pub height: u8,
}
```

This will create two structs called `ValueRegister` and `ValueRegisterBitfield`.
The first will only contain a byte array `[u8; N]` to store the packed register contents,
and the latter will contain the unpacked actual members as defined above.
You will always interface with a device using the packed data, which can be transferred over the bus as-is.

> [!NOTE]
> I find it a bit misleading that the members written in `ValueRegister` end up in `ValueRegisterBitfield`.
> So this might change in the future, but I currently cannot think of another design that is as simple
> to use as the one we have now. The issue is that we need a struct for the packed data and one for
> the unpacked data. Since we usually deal with the packed data, and want to allow direct read/write
> operations on the packed data for performance, the naming gets confusing quite quickly.

### Accessing a register

After defining a register, we may access it through `MyDevice`:

```rust
// Imagine we already have constructed a device:
let mut dev = MyDevice::new_i2c(i2c_bus /* the i2c bus from your controller */, 0x12 /* address */);
// We can now retrieve the register
let mut reg = dev.read_register::<ValueRegister>().await?;

// Unpack a specific field from the register and print it
println!("{}", reg.read_width());
// If you need all fields (or are not bound to tight resource constraints),
// you can also unpack all fields and access them more conveniently
let data = reg.read_all();
// All bitfields implement Debug and defmt::Format, so you can conveniently
// print the contents
println!("{:?}", data);

// We can also change a single value
reg.write_height(190);
// Or pack a bitfield and replace everything
reg.write_all(data); // same as reg = ValueRegister::new(data);

// Which we can now write back to the device, given that the register is writable.
dev.write_register(reg).await?;
```

### A more complex register

A more complex register might contain more than just simple values. Often there are
enums, or bitflags involved, which we can luckily also represent with bondrewd. Using
bondrewd's attribute macros, we can specify exactly which bit corresponds to which field:

```rust
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Copy, Clone, Default, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum TemperatureResolution {
    Deg_0_5C = 0b00,
    Deg_0_25C = 0b01,
    Deg_0_125C = 0b10,
    #[default]
    Deg_0_0625C = 0b11,
}

#[device_register(MyDevice)]
#[register(address = 0x44, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ComplexRegister {
    #[bondrewd(bit_length = 6, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pub temperature_resolution: TemperatureResolution,
}
```

> [!NOTE]
> Instead of naming all registers `*Register`, in reality we would likely place all registers
> in a common `registers` module for convenience and drop the suffix.

### Defining a device

Now we also need to define our device so we can actually use the register.
Imagine our simple device would communicate over I2C only.

First of all, we can define a struct for it by using the `#[device]` macro.
This struct stores the runtime state necessary for our device, such as the communication interface.
The macro just defines a marker trait which we will need later to define registers, you can ignore it for now.

```rust
/// Insert description from datasheet.
#[device]
pub struct MyDevice<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}
```

A register interface is a trait from `embedded-registers`, which is any object exposing a specific
`read_register` and `write_register` function. Internally this will be the function we can later call
to read or write a specific register.

But before we can do that, we need to add a constructor to our device which
allows us to create a new device from a real I2C bus and an address. The `embedded-registers` crate
also provides a struct called `I2cDevice` that implements `RegisterInterface` for I2C, so
we just use that as our interface.

For very simple devices that need no additional fields
there is a convenience macro called `simple_device::i2c!` that defines the necessary device for you:

```rust
simple_device::i2c!(MyDevice, MyAddress, SevenBitAddress, OneByteRegAddrCodec);
```

The address enum `MyAddress` should contain all valid addresses for the device,
plus a variant to allow specifying arbitrary addresses, in case the user uses an
address translation unit. The address can be any type that is convertible to the
convertible to the underlying `embedded_hal::i2c::AddressMode` of the bus
(7-bit or 10-bit addressing). For a real example, refer to `mcp9808::Address`.

The third parameter is either `SevenBitAddress` or `TenBitAddress`, depending on the
addressing mode of your device. In most cases, devices use seven bit addresses.

The last parameter specifies the codec that is used to read or write registers over I2C.
Again, this is usually a simple codec that just prepends the register address before reading
or writing data, but depending on the device it can be more complex. This would be the
location where you can adjust the required protocol.

Finally, this macro actually just defines a constructor for our device and is equivalent to this `impl` block below.
We'll later see why we need `#[maybe_async_cfg]`.

```rust
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> MyDevice<I2cDevice<I, hal::i2c::SevenBitAddress, OneByteRegAddrCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    #[inline]
    pub fn new_i2c(interface: I, address: MyAddress) -> Self {
        Self {
            interface: I2cDevice::new(interface, address.into(), OneByteRegAddrCodec::default()),
        }
    }
}
```

Now that we have a device, we need to be able to read and write registers, and may also implement high-level functions
that use these registers. The shortest implementation is just an empty block with the `#[device_impl]` attribute macro.

```rust
#[device_impl]
impl<I: RegisterInterface> MyDevice<I> {
}
```

Still remember the marker trait from the beginning, which was created by `#[device]`?
This is where it now comes into play; The `I2cDevice` which we store as our interface is very generic and
would allow us to read any valid register definition that we wish. But since there are multiple drivers
with valid registers in this crate, we want to prevent registers from being used with unrelated devices.

The `#[device_impl]` macro therefore defines two functions `read_register` and `write_register` that internally just
call the similarly named functions on `self.interface`, but additionally require the passed register types to implement
our marker trait. The registers are annotated with the marker trait by `#[device_register]`,
so passing unrelated registers will now result in a compilation error.

The user is now already able to interface with the device's registers in a mostly safe way,
except of course for things that are not represented by our typed registers (such as implicit device states).
The last step is to expose convenience functions for our device. The classics are things like `init`, `reset`,
`measure`, `configure` or others, but in principle you are free to write anything.

Finally, when writing functions for our device, we will always write async code.
The `maybe_async_cfg` macro will automatically derive the sync code from our definition, and replace
references to `hal` with from `embedded_hal` or `embedded_hal_async`. This is necessary
so we can support both async and sync crate consumers. Take for example this `init` from the `MCP9808`:

```rust
#[device_impl]
impl<I: RegisterInterface> MyDevice<I> {
    /// Initialize the sensor by verifying its device id and manufacturer id.
    /// Not mandatory, but recommended.
    pub async fn init(&mut self) -> Result<(), InitError<I::Error>> {
        use self::registers::DeviceIdRevision;
        use self::registers::ManufacturerId;

        let device_id = self.read_register::<DeviceIdRevision>().await.map_err(InitError::Bus)?;
        if device_id.read_device_id() != self::registers::DEVICE_ID_VALID {
            return Err(InitError::InvalidDeviceId);
        }

        let manufacturer_id = self.read_register::<ManufacturerId>().await.map_err(InitError::Bus)?;
        if manufacturer_id.read_manufacturer_id() != self::registers::MANUFACTURER_ID_VALID {
            return Err(InitError::InvalidManufacturerId);
        }

        Ok(())
    }
}
```

## Contributing

**If you have any suggestions or ideas to improve the current architecture, please feel encouraged to open an issue or reach out via [Matrix](https://matrix.to/#/@oddlama:matrix.org)**

Contributions are whole-heartedly welcome! Please feel free to suggest new features, implement device drivers,
or generally suggest improvements.

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.
Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in this crate by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.
