- rename embedded-registers to embedded-interfaces
- rename embedded registers stuff to add registers:: mod in between

- all defmt derives only if defmt feature is enabled
- workspace dependencies for stuff that is needed all the time, bondrewd, embedded-hal, maybe-async-cfg, ...
- make #[register(default = )] add a #[doc] annotation
- move sensor and device traits into a new crate embedded-devices-hal
- any register that has physical values (temperature, ambient pressure) also provide read_ write_ and with_ implementations that do the conversion to/from uom units
