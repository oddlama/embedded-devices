[![Crate](https://img.shields.io/crates/v/embedded-devices.svg)](https://crates.io/crates/embedded-devices)
[![API](https://docs.rs/embedded-devices/badge.svg)](https://docs.rs/embedded-devices)

# embedded-devices

**WARNING: All drivers in this crate are fully functional, but the crate design
is in an experimental state. Until v1.0.0 is released there may be breaking
changes at any time.**

This project contains a collection of drivers for embedded devices, as well as a
framework to make building drivers more streamlined and simple. As a user
you get:

- ✅ **Type-safe and ergonomic access** to all device registers and functions
- 📚 **Thorough documentation** for each device based on the original datasheets
- 🧵 **Supports both sync and async** usage for each driver - simultaneously if needed
- 🧩 **Consistent interaction** with all devices following the same principles
- 🧪 **Physical quantities** and their units like °C/°F or Ω
  are associated to each value to prevent mix-ups and to allow automatic unit conversions

Please refer to the list below for supported devices. For driver developers,
this framework aims to make it a lot easier to write consistent and
fully-featured drivers:

- ✨ **Effortless, type-safe representation** of registers - capture exactly what has been specified in the datasheet
- ⚡ **Zero-cost abstractions** for efficient register access
- 🧰 **Unified framework** for building and extending drivers
- 🔄 **Reusable codecs** to handle extended communication protocols (e.g. CRC checks over I2C/SPI)

> This crate started as a proof-of-concept to show how sensor and device
> drivers can benefit from a common framework, allowing new drivers to be added
> with ease while being able to skip writing bus communication boilerplate over
> and over again.

This project contains of several crates:

- [embedded-devices](./embedded-devices) - contains the actual driver implementations for real hardware
- [embedded-registers](./embedded-registers) - provides the abstraction to define and use registers over I2C/SPI, used heavily by embedded-devices
- [embedded-registers-derive](./embedded-registers-derive), a proc-macro to reduce boilerplate for register definitions
- [embedded-devices-derive](./embedded-registers-derive), a proc-macro to reduce boilerplate for device definitions

## Supported Devices

Below you will find a list of all currently supported devices. Please visit their respective documentation links for more information and usage examples.

<!-- TODO: should we order by category and also list the most interesting specs? like accuracy for temp sens, feature matrix style for pressure, humidity, ... -->

| Manufacturer | Device | Interface | Description | Docs |
|---|---|---|---|---|
| Analog Devices | MAX31865 | SPI | Precision temperature converter for RTDs, NTCs and PTCs | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/analog_devices/max31865/index.html) |
| Bosch | BME280 | I2C/SPI | Temperature, pressure and relative humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bme280/index.html) |
| Bosch | BMP280 | I2C/SPI | Temperature and pressure sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bmp280/index.html) |
| Bosch | BMP390 | I2C/SPI | Temperature and pressure sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bmp390/index.html) |
| Microchip | MCP3204 | SPI | 12-bit ADC, 4 single- or 2 differential channels | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp3204/index.html) |
| Microchip | MCP3208 | SPI | 12-bit ADC, 8 single- or 4 differential channels | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp3208/index.html) |
| Microchip | MCP9808 | I2C | Digital temperature sensor with ±0.5°C (max.) accuracy | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp9808/index.html) |
| Texas Instruments | INA219 | I2C | 12-bit current shunt and power monitor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/ina219/index.html) |
| Texas Instruments | INA226 | I2C | 36V, 16-bit current shunt and power monitor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/ina226/index.html) |
| Texas Instruments | INA228 | I2C | 85V, 20-bit current shunt and power monitor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/ina228/index.html) |
| Texas Instruments | TMP102 | I2C | Temperature sensor with ±0.5°C to ±3°C accuracy depending on the temperature range | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/tmp102/index.html) |
| Texas Instruments | TMP117 | I2C | Temperature sensor with ±0.1°C to ±0.3°C accuracy depending on the temperature range | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/texas_instruments/tmp117/index.html) |
| Sensirion | SCD40 | I2C | Photoacoustic NDIR CO₂ sensor (400-2000ppm) ±50ppm ±5.0%m.v. | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/scd4x/index.html) |
| Sensirion | SCD41 | I2C | Improved photoacoustic NDIR CO₂ sensor (400-5000ppm) ±50ppm ±2.5%m.v. | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/scd4x/index.html) |
| Sensirion | SCD43 | I2C | High accureacy photoacoustic NDIR CO₂ sensor (400-5000ppm) ±30ppm ±3.0%m.v. | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/scd4x/index.html) |
| Sensirion | SEN60 | I2C | Particulate matter (PM1, PM2.5, PM4, PM10) sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/sen60/index.html) |
| Sensirion | SEN63C | I2C | Particulate matter (PM1, PM2.5, PM4, PM10), CO₂, temperature and relative humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/sen63c/index.html) |
| Sensirion | SEN65 | I2C | Particulate matter (PM1, PM2.5, PM4, PM10), VOC, NOₓ, temperature and relative humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/sen65/index.html) |
| Sensirion | SEN66 | I2C | Particulate matter (PM1, PM2.5, PM4, PM10), CO₂, VOC, NOₓ, temperature and relative humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/sen66/index.html) |
| Sensirion | SEN68 | I2C | Particulate matter (PM1, PM2.5, PM4, PM10), CO₂, VOC, NOₓ, HCHO, temperature and relative humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/sensirion/sen68/index.html) |

## Example usage

This example shows how to read temperature, pressure and humidity from the
BME280 environmental sensor to give you an idea of how the drivers work. This
example shows how to use it in an async context. The `BME280Sync` variant works
analogous, just without calling `.await`:

```rust
// Create a device on the given interface and address
let mut bme280 = BME280Async::new_i2c(delay, i2c, Address::Primary);

// Initializes (resets) the device and makes sure we can communicate
bme280.init().await?;

// Configure certain device parameters
bme280.configure(Configuration {
    temperature_oversampling: Oversampling::X_16,
    pressure_oversampling: Oversampling::X_16,
    humidity_oversampling: Oversampling::X_16,
    iir_filter: IIRFilter::Disabled,
}).await?;

// Measure now
let measurement = bme280.measure().await?;

// Retrieve the returned temperature as °C, pressure in Pa and humidity in %RH
let temp = measurement.temperature.get::<degree_celsius>();
let pres = measurement.pressure.unwrap().get::<pascal>();
let hum = measurement.humidity.unwrap().get::<percent>();

println!("Current temperature: {}°C", temp);
println!("Current pressure: {}Pa", pres);
println!("Current humidity: {}%RH", hum);
```

Note that every device is gated behind a crate feature, for example
`bosch-bme280` for the device above. You can also disable all `sync` or `async`
variants globally by disabling the respective feature.

## Architecture

Driver implementations are organized based on the manufacturer name and device
name. Each driver exposes a struct with the name of the device, for example
`BME280`, which automatically get translated into a `BME280Sync` and
`BME280Async` variant.

Usually the device owns an interface for communication. There are no further
restrictions on the struct, so if it requires multiple interfaces or extra
pins, then this is easily possible.

Most devices will expose a `new_i2c` and/or `new_spi` function to construct the
appropriate object given an interface (and address if required).

### Codecs

The vast majority of devices use similar "protocols" on top of I2C or SPI to
expose their registers - which we call codecs. For both I2C and SPI we provide
a `SimpleCodec` implementation, that should allow communication with most of
the simpler devices in existence. If a device (or single register) requires a
more involved codec (for example to verify CRC checksums), we probably have
that covered already. You can always define custom codecs if necessary.

### Register based devices

The [embedded-registers](./embedded-registers) crate provides a generic way to define registers, and an interface implementation
to allow reading and writing those registers via I2C or SPI.

A register usually refers to a specific memory address (or consecutive memory region) on the device by
specifiying its start address. We also associate each register to a specific device by
specifying a marker trait. This prevents the generated API from accepting registers of unrelated devices.
In the following, we'll have a short look at how to define and work with register based devices.

### Defining a register

First, lets start with a very simple register definition. We will later create
a device struct which we can use to read and write this register.

To define our very simple register, we

1. Specify the device it belongs to using the `#[device_register(MyDevice)]` macro. The mentioned device will be created later.
2. Define its address and access mode (read-only, write-only, read-write) using the `#[register(...)]` macro,
3. Describe its internal structure with [bondrewd](https://github.com/Devlyn-Nelson/Bondrewd), a flexible bitfield macro crate.

Most devices support a burst-read/write operations, where you can begin reading from address `A` and will automatically receive
values from the consecutive memory region until you stop reading. This means you can define registers with a `size > 1 byte`
and will get the content you expect.

Let's imagine our `MyDevice` had a 2-byte read-write register at device address `0x42(,0x43)`, which contains two `u8` values.
We can define the corresponding register like so:

```rust
/// Insert explanation of this register from the datasheet.
#[device_register(MyDevice)]
#[register(address = 0x42, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 2)]
pub struct ValueRegister {
    /// Insert explanation of this value from the datasheet.
    pub width: u8,
    /// Insert explanation of this value from the datasheet.
    pub height: u8,
}
```

This macro combination generates two structs: `ValueRegister` (the packed
representation) and `ValueRegisterBitfield` (the unpacked fields). The former
will only contain a byte array `[u8; N]` to store the packed register contents,
and the latter will have the unpacked fields as we defined them. Usually we
will interface with a device using the packed data representation which can be
transferred over the bus as-is. Each field will automatically get accessor
functions with which you may read or write them without incurring the overhead
for full (de-)serialization.

The given codec provides then necessary information about the protocol that is
needed to access the register on a certain bus. It can determine how the
register address is used on the wire and could do extra checks like CRC
checksums.

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

A real-world application may involve describing registers with more complex layouts involving
different data types or even enumerations. Luckily, all of this is fairly simple with bondrewd.

We also make sure to annotate all fields with `#[register(default = ...)]` to allow
easy reconstruction of the power-up defaults.

```rust
#[allow(non_camel_case_types)]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum TemperatureResolution {
    Deg_0_5C = 0b00,
    Deg_0_25C = 0b01,
    Deg_0_125C = 0b10,
    Deg_0_0625C = 0b11,
}

#[device_register(MyDevice)]
#[register(address = 0x44, mode = "rw", i2c_codec = "OneByteRegAddrCodec")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ComplexRegister {
    #[bondrewd(bit_length = 6, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// All fields should be documented with data from the datasheet.
    /// This docstring will also be copied to all generated read/write functions
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = TemperatureResolution::Deg_0_0625C)]
    pub temperature_resolution: TemperatureResolution,
}
```

> [!NOTE]
> Instead of naming all registers `*Register`, in a real driver you'd likely place all registers
> in a common `registers` module for convenience and then drop the suffix.

### Defining a device

Now we also need to define our device so we can actually use the register.
Imagine our simple device would communicate over I2C only.

First of all, we create a struct for it and annotate it with `#[device]`. This
struct stores the runtime state necessary to use our device, such as the
communication interface. The macro just defines a marker trait which we will
need later to define associated registers.

Also, we always write async code and let the `#[maybe_async_cfg::maybe]` macro
rewrite our definition twice to provide a `MyDeviceSync` and `MyDeviceAsync`
variant. All given idents are replaced with their respective sync or async
variant, too:

```rust
/// Insert description from datasheet.
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct MyDevice<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}
```

The register interface `I` can be anything that implements the corresponding
trait from `embedded-registers` - which requires that the interface exposes
`read_register` and `write_register` functions. These function will later be
used to actually read or write a specific register. But before we can do that,
we still need to add a constructor to our device.

### Constructing a device

Now we need a way to create a new instance of our device given a real I2C bus
and an address. Usually we expose a `new_i2c` and/or `new_spi` function to
construct the device with the given interface.

```rust
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> MyDevice<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    #[inline]
    pub fn new_i2c(interface: I, address: MyAddress) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
        }
    }
}
```

Here we use the `I2cDevice` interface provided by `embedded-registers`, which
already implements the necessary trait from above. This means we pass down the
given interface implementation plus some compile time information like the kind
of I2C address that the device uses (7/10-bit). In turn, `I2cDevice` provides a
simple interface we can use to read/write registers.

The address enum `MyAddress` should contain all valid addresses for the device,
plus a variant to allow specifying arbitrary addresses, in case the user uses
an address translation unit. The address can be any type that is convertible to
the underlying `embedded_hal::i2c::AddressMode` of the bus (7-bit or 10-bit
addressing).

### High-level device functions

Now that we have a device and a way to read or write registers, we want to
expose the `read_register` and `write_register` functions directly on our
device to make it convenient for a user to use these without first requiring
them to get the interface.

Since this is something every device will do, we again have a convenience macro
`#[device_impl]` that lifts these functions into the device.

The last step - apart from defining registers - is to expose some convenience
functions for our device. The classics are things like `init`, `reset`,
`measure`, `configure` or others, but in principle you are free to write
anything that makes the device easy to use.

```rust
#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> MyDevice<I> {
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

### Definining registers

For an in-depth explanation of how registers are defined, please
please refer to the [embedded-registers-derive](https://docs.rs/embedded-registers-derive/latest/embedded_registers_derive/) docs.

### Register and interface traits

For an in-depth explanation of how the defined registers are used,
please refer to the [embedded-registers](https://docs.rs/embedded-registers/latest/embedded_registers/) docs.

## Device driver best-practices

When writing new device drivers, please consider the following best-practices:

- Expose all registers defined in the datasheet to allow accessing all device functions, only define functions for functionality
- If the device is a sensor:
  - Create a `Measurement` struct (singular!) that holds all values measured
      by the sensor. Implement the relevant `*Measurement` traits.
  - Implement the `Sensor` trait and all relevant subtraits (e.g. `TemperatureSensor`)

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
