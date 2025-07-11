# v0.9.14

This release includes several breaking changes.

## Features / Improvements

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

- Codecs can now return errors, so CRC errors no longer panic
- Codecs were reworked to no longer operate on a specific interface type. Instead,
  they are now only responsible for encoding and decoding registers to bytes.

#### General

- `uom` types always use `f64` to reduce compile times and make usage easier.
  While `rational32`/`rational64` do retain better precision, usage is awkward and
  rational reduction calculations are pretty slow. `f64` seemed like the best compromise
  until fixed point user types or auto conversion is supported.
- Other `uom` base were disabled in order to speed up compilation by a large factor
- Error types are now made with thiserror, and appropriate From<> impls were added to avoid excessive `.map_err()` calls
