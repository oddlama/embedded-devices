- remove all #[default] in enums and instead annotate the field.
  an enum may be used in different contexts with different defaults.
- explicitly document all defaults with #[register(default = ...)]
- uom is nice and all, but compile times are heavy and it is hard to capture true
  register resolution with rational32 or rational64, which is also expensive.
  You need at least storage w.r.t. different magnitudes to avoid rationals.
  Ideally I'd like to use a fixed point library and some form of simple physical quantity enforcement.
  Last time I checked the most popular fixed point library was not available for no_std.
- Gate each driver behind a crate feature
- Don't gate sync/async behind features, but allow simultaneous use. (async::Device, sync::Device).
- (minor) dot (.) at the end of docstrings? I'm being very inconsistent here
- use thiserr
- use workspace dependencies for at least common stuff like embedded-hal etc.
