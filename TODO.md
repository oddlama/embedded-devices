- workspace dependencies for stuff that is needed all the time, bondrewd, embedded-hal, maybe-async-cfg, ...
- make #[register(default = )] add a #[doc] annotation
- move sensor and device traits into a new crate embedded-devices-hal
- untangle bme and bmp
- Configuration -> Config

codegen:

- register_devices as register_defaults.devices
- structs / enums in arrays
- per field read/write annotation (to skip generating read_ write_ with_ functions) (?)
- typst output for each register block
