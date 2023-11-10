# Embedded devices

[![Crate](https://img.shields.io/crates/v/embedded-devices.svg)](https://crates.io/crates/embedded-devices)
[![API](https://docs.rs/embedded-devices/badge.svg)](https://docs.rs/embedded-devices)

**WARNING: This crate is currently in experimental state, so anything may change at any time.**

Device driver implementations for many embedded sensors and devices.
Supported devices:

<!-- TODO: should we order by category and also list the most interesting specs? like accuracy for temp sens -->

| Manufacturer | Device | Interface | Description | Docs |
|---|---|---|---|---|
| Bosch | BME280/BMP280 | I2C/SPI | Temperature, pressure and humidity (BME) sensor | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/bosch/bme280/index.html) |
| Microchip | MCP9808 | I2C | Digital temperature sensor with ±0.5°C (max.) accuracy | [Docs](https://docs.rs/embedded-devices/latest/embedded_devices/devices/microchip/mcp9808/index.html) |

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.
Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in this crate by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.
