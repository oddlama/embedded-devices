- all defmt derives only if defmt feature is enabled
- workspace dependencies for stuff that is needed all the time, bondrewd, embedded-hal, maybe-async-cfg, ...
- make #[register(default = )] add a #[doc] annotation
- move sensor and device traits into a new crate embedded-devices-hal

codegen:

- check doc comments in embedded-interfaces and embedded-interfaces-codegen

- structs / enums in arrays
- per field read/write annotation (to skip generating read_ write_ with_ functions) (?)
- typst output for each register block
