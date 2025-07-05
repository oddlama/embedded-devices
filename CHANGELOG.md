# v0.9.14

#### Features

- add common traits for devices and sensors
  - `ResettableDevice` for any device that supports software reset
  - `OneshotSensor` for any sensor that supports oneshot measurements (or can reasonably emulate it)
  - `ContinuousSensor` for any sensor that supports continuous measurements
- codecs can now return errors, so crc errors no longer panic
- a delay provider is now only needed at device creation, not for each function call
- support new sensors: Sensirion SEN60, SEN63C, SEN65, SEN66, SEN68
- reworked: Sensirion SCD40, SCD41, SCD43

#### Improvements

- switched to uom f64 types everywhere. While rational32/rational64 do retain
  better precision, usage is awkward and reducing calculations are pretty slow.
  f64 seemed like the best compromise until fixed point user types are supported.
- disable other uom base types to speed up compilation a lot
- unify sensor measurement interface. oneshot is now always exposed as `Sensor::measure` and new functions for continuous mode measurements were added
- error types are now made with thiserror, and appropriate From<> impls were added to avoid excessive `.map_err()` calls
