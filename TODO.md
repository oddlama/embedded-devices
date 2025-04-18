- uom is nice and all, but compile times are heavy and it is hard to capture true
  register resolution with rational32 or rational64, which is also expensive.
  You need at least storage w.r.t. different magnitudes to avoid rationals.
  Ideally I'd like to use a fixed point library and some form of simple physical quantity enforcement.
  Last time I checked the most popular fixed point library was not available for no_std.
- (minor) dot (.) at the end of docstrings? I'm being very inconsistent here
- use thiserr
- all defmt derives only if defmt feature is enabled
- workspace dependencies for stuff that is needed all the time, bondrewd, embedded-hal, maybe-async-cfg, ...
- make #[register(default = )] add a #[doc] annotation
- remove i2c! macro. too inflexible to provide real benefit.
