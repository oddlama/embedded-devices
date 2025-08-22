# v0.10.1

- Fixed wrong BME280 trimming parameter definition
- Add BitdumpFormatter for `std` execution environments. This provides a
  hexdump-like output format for all bit-packed structs
- Enabling the `trace-communication` will log all registers that are being sent or received
  for debugging purposes

# v0.10.0

This release includes several breaking changes!

The whole crate was reworked to allow non-register based interfaces (such as
commands) and allow a more ergonomic definition of bit-packed structs. New
traits have been added to unify how different devices are initialized, reset
and used - especially sensors which now implement measurements using a
common abstraction.

## Features / Improvements

`embedded-registers` was retired in favor of `embedded-interfaces`, which is
more general and brings its own ergonomic bit-packing macro.

#### Devices

- Added common traits for devices and sensors:
  - `ResettableDevice` for any device that supports software reset
  - `OneshotSensor` for any sensor that supports oneshot measurements (or can reasonably emulate it)
  - `ContinuousSensor` for any sensor that supports continuous measurements
- Functions for measurement have been harmonized across all devices and now use traits from above
- All devices now own a delay provider which is required when calling `new`.
  Function calls on the device no longer require a delay. 
- Support for new sensors:
  - Sensirion SEN60, SEN63C, SEN65, SEN66, SEN68
- Some sensor implementations were reworked:
  - Sensirion SCD40, SCD41, SCD43

#### Codecs

- Codecs can now return specific errors

#### General

- Removed all calls that can `panic!()`. From now on, all drivers in this crate are 100% panic-free
  and thus ready to be used in a high-reliability contexts.
- All `uom` types now always use `f64` to reduce compile times and make usage easier.
  While `rational32`/`rational64` do retain better precision, usage is awkward and
  rational reduction calculations are pretty slow. `f64` seemed like the best compromise
  until fixed point user types or auto conversion is supported.
- Other `uom` bases were disabled in order to speed up compilation significantly
- Error types are now made with `thiserror`, and appropriate `From<E>` impls were added to avoid excessive `.map_err()` calls
