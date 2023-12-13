[![Crate](https://img.shields.io/crates/v/embedded-devices.svg)](https://crates.io/crates/embedded-devices)
[![API](https://docs.rs/embedded-devices/badge.svg)](https://docs.rs/embedded-devices)

# Embedded-devices

**WARNING: This crate is currently in experimental state, so anything may change at any time.**

Welcome to the embedded-devices project, where you'll find a collection of drivers for a variety of different embedded sensors and devices.
These drivers are designed for `async` use but also support synchronous contexts via a feature switch.
Our goal is to provide feature-complete, up-to-date drivers with an ergonomic interface, catering both to high-level device functions and low-level register access.
Please refer to the list below for supported devices.

The ecosystem of embedded-rust device drivers seems quite fragmented,
consisting both of high quality implementations but also of several incomplete or outdated drivers,
where only a few have first-class async support. This renders them hard (or even impossible) to use
in embedded frameworks like [embassy](https://github.com/embassy-rs/embassy),
which in my opinion would benefit from having access to more ready-to-use async drivers with frequently updated dependencies.

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
| Bosch | BME280 | I2C/SPI | Temperature, pressure and humidity sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bme280/index.html) |
| Bosch | BMP280 | I2C/SPI | Temperature and pressure sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bmp280/index.html) |
| Microchip | MCP9808 | I2C | Digital temperature sensor with ±0.5°C (max.) accuracy | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp9808/index.html) |

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
