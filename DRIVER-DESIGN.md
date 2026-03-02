# Driver Design Guide

This document captures the design principles, required patterns, and known anti-patterns for
drivers in this crate. All new drivers and changes to existing drivers must follow these rules.

---

## Non-Negotiable Invariants

These are hard guarantees the entire crate upholds. Violating them is a bug.

### 1. No panics

The crate sets `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]`.

- **Never** use `.unwrap()`, `.expect()`, `panic!()`, or array indexing that can go out of bounds.
- Use `Result` and `Option` propagation throughout.
- Division or bit-operations that could overflow must be checked or use saturating/wrapping variants.

### 2. No loops, no unconditional delays inside measurement functions

Measurement functions must be **deterministic**: a single attempt, no retry loops, no
unconditional delays. Polling (checking and sleeping repeatedly) is the caller's responsibility,
not the driver's.

**Wrong — loops forever:**
```rust
loop {
    if self.is_measurement_ready().await? {
        return self.current_measurement().await;
    }
    self.delay.delay_ms(100).await;
}
```

**Wrong — swallows errors, truly infinite:**
```rust
loop {
    match self.execute::<ReadMeasurement>(()).await {
        Ok(m) => return Ok(m),
        Err(_) => self.delay.delay_ms(150).await, // also a delay inside measurement fn
    }
}
```

**Correct — check once, return immediately:**
```rust
// For sensors with a data-ready flag:
if !self.is_measurement_ready().await? {
    return Err(MeasurementError::DataNotReady);
}
self.current_measurement().await?.ok_or(MeasurementError::DataNotReady)

// For sensors without a data-ready flag (e.g. STCC4, where NACK = not ready):
let m = self.execute::<ReadMeasurement>(()).await?; // NACK surfaces as a bus error
Ok(build_measurement(m))
```

The caller waits `measurement_interval_us()` before calling `next_measurement`. If the driver
returns `DataNotReady` or a bus error, the caller decides whether to retry or give up.

---

## Delay Placement

Delays must be **tied to a specific operation** that causes the need for the delay.
Unconditional delays at the start of a function are almost always wrong.

### Power-on delay

Every sensor datasheet specifies a minimum time from VDD stable to the first I2C command.
**This is the caller's responsibility**, not the driver's. Document it, do not enforce it.

**Wrong:**
```rust
pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
    // Wastes 30 ms even when called on a device that's been powered for seconds
    self.delay.delay_ms(30).await;
    self.reset().await?;
    Ok(())
}
```

**Correct:**
```rust
/// Initializes the sensor.
///
/// # Power-on timing
///
/// The datasheet requires at least 30 ms between VDD reaching the operating voltage and
/// the first I2C command. The caller must ensure this delay has elapsed before calling
/// `init`. Calling `init` immediately after `new_i2c` without an external delay may
/// result in a failed initialization.
pub async fn init(&mut self) -> Result<(), InitError<I::BusError>> {
    self.reset().await?;
    Ok(())
}
```

### Delays after a specific command

Delays that follow a concrete command are correct and stay in the driver:

```rust
// OK: datasheet specifies the sensor needs 5 ms to wake from sleep
pub async fn exit_sleep_mode(&mut self) {
    let _ = self.execute::<ExitSleepMode>(()).await;
    // caller is responsible for the 5 ms if they need it before the next command;
    // or the command executor handles the delay via `time_ms` in define_sensirion_commands!
}
```

For Sensirion command-based drivers, use the `time_ms` field in `define_sensirion_commands!`
to encode execution time so the executor handles it automatically.

### Measurement delays

Delays inside measurement functions must be justified:

- **Register-based** (BME280, TMP117, …): calculate the measurement time from current oversampling
  / averaging settings and delay exactly that long before reading.
- **Polling with data-ready flag**: poll at a fixed interval (≤ 1/10 of the measurement interval)
  up to a bounded count.

---

## Error Type Conventions

### No cross-domain `From` conversions

Never implement `From<ErrorTypeA> for ErrorTypeB` when `A` and `B` belong to different
operation domains (e.g. initialization vs. measurement). Such conversions destroy type
information, imply a relationship that does not exist, and force callers to either catch-all
or deal with a bloated error type.

**Wrong — `InitError` and `MeasurementError` are unrelated domains:**
```rust
impl<E> From<InitError<E>> for MeasurementError<E> { ... } // Never do this
```

The fact that `init()` and `next_measurement()` are called in the same application function
is not a reason to unify their error types in the library API. Use `anyhow::Result` (or a
locally defined application error enum) at the call site instead.

**In doc examples**, use `anyhow::Result<()>` when the example spans multiple error domains:
```rust
# fn test<I, D>(...) -> anyhow::Result<()>
# where I::Error: std::error::Error + Send + Sync + 'static { ... }
```

Gate such examples on `#[cfg(all(feature = "sync", feature = "std"))]` since anyhow
requires std.

---

### Per-operation error types

Use dedicated error enums for each distinct failure domain. Do not reuse one catch-all type:

| Error type | When to define | Typical variants |
|---|---|---|
| `InitError<BusError>` | Device initialization | `Transport(#[from] TransportError<BusError>)`, `InvalidChipId(u8)`, `InvalidProductId(u32)` |
| `MeasurementError<BusError>` | Waiting for / reading measurements | `Transport(#[from] TransportError<BusError>)`, `DataReadyTimeout`, `NotCalibrated` |
| `ContinuousMeasurementError<BusError>` | When continuous differs from oneshot | `Transport(#[from] …)`, `Overflow(Measurement)`, `DataReadyTimeout` |

Use `#[from]` on the `Transport` variant to allow `?` on inner `TransportError` results.

### Sentinel values for unavailable data

Hardware sometimes signals "not yet available" via a sentinel value (e.g. `0xFFFF` or `i16::MAX`).
Represent such fields as `Option<T>` in the `Measurement` struct:

```rust
pm2_5_concentration: (raw != u16::MAX).then(|| decode(raw)),
voc_index: (!matches!(raw, i16::MAX | 0)).then_some(decode(raw)),
```

For indices that start at 0, check both 0 and MAX. For unsigned concentrations, MAX alone is
sufficient.

### Do not swallow errors

Every `Err(_) => { ... }` arm that discards the error value is a red flag. If an error can
legitimately occur and be retried (e.g. NACK on STCC4 ReadMeasurement), still count it toward
the retry budget and propagate `DataReadyTimeout` if the budget is exhausted.

---

## Sensor Trait Implementation Rules

### `ContinuousSensor`

| Method | Rule |
|---|---|
| `start_measuring` | Send the start command. Return transport errors. |
| `stop_measuring` | Send the stop command. Return transport errors. |
| `measurement_interval_us` | Return a fixed constant or compute from device registers. Never wait. |
| `is_measurement_ready` | Query the data-ready flag if available; return `Ok(true)` if not. |
| `current_measurement` | Read the measurement registers/command once. Return `Ok(None)` if data is not ready or invalid. Do not wait. |
| `next_measurement` | Poll `is_measurement_ready` (or read directly) in a **bounded** loop. Return `MeasurementError::DataReadyTimeout` if the bound is exceeded. |

### `OneshotSensor`

| Method | Rule |
|---|---|
| `measure` | Trigger measurement, wait the minimum required time (use `time_ms` in command or a calculated delay), read result. No retry loop unless bounded. |

If `measure` internally calls `next_measurement` (e.g. SCD41/43 single-shot triggers then polls),
the `OneshotSensor::Error` type must be the same as `ContinuousSensor::Error` so the `?` operator
works without a conversion.

### `ResettableDevice`

- Ignore errors from "stop if running" commands (use `let _ = ...`).
- Propagate errors from the actual reset/reinit command.
- Do **not** add power-on delays here either; the caller controls timing.

---

## Command-Based (Sensirion) Driver Specifics

### `define_sensirion_commands!`

- Set `time_ms` to the execution time from the datasheet. The executor will delay after the
  command and before reading the response, so driver code does not need manual delays.
- Commands that send no data and receive no response use `send`.
- Commands that receive a response use `read` or `write_read`.
- Single-byte command IDs (e.g. STCC4 `ExitSleepMode = 0x00`) need a separate
  `define_sensirion_commands!` block with `id_len 1`.

### Wake-up / NACK-expected commands

Some Sensirion sensors do not acknowledge the wake-up command. Always ignore errors:

```rust
let _ = self.execute::<WakeUp>(()).await;
// or
let _ = self.execute::<ExitSleepMode>(()).await;
```

### Devices without `GetDataReady`

STCC4 NACKs `ReadMeasurement` when no data is available instead of providing a data-ready flag.
There is no retry loop. Attempt the read once and propagate the error to the caller:

```rust
// The caller must wait measurement_interval_us() before calling this.
let m = self.execute::<ReadMeasurement>(()).await?; // NACK → TransportError::Bus
Ok(build_measurement(m))
```

`is_measurement_ready()` returns `Ok(true)` since data availability cannot be checked without
attempting a read. The caller controls timing.

---

## Physical Units

Always use `uom` types for physical quantities. Never use bare `f64` or `f32` for anything with a
unit. Use `uom::si::f64::{ThermodynamicTemperature, Pressure, Ratio, …}`.

Conversions from raw register values belong in the `interface_objects!` struct accessors
(`from_raw`, `lsb`, `offset`), not scattered across driver code.

---

## General Coding Rules

- **Expose the complete register/command set** from the datasheet, not just the ones you need
  right now. Future users will need the rest.
- **Feature flag every device**: add `#[cfg(feature = "vendor-partname")]` guards and list the
  feature in both `all-devices` and the per-vendor section of `Cargo.toml`.
- **defmt support**: add `#[cfg_attr(feature = "defmt", derive(defmt::Format))]` to every public
  struct and enum that is part of the driver's public API.
- **Dual sync/async**: write code once as `async`, wrap with
  `#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]`.
  Do not duplicate implementations.
- **No redundant abstraction**: do not create helper functions or traits for patterns used only
  once. Three similar lines are preferable to a premature abstraction.

---

## Checklist for New Drivers

- [ ] No `.unwrap()`, `.expect()`, `panic!()`, or infallible index operations
- [ ] No loops in measurement functions; `next_measurement` is a single non-blocking attempt
- [ ] No unconditional power-on delay inside `init`; documented as caller's responsibility
- [ ] `MeasurementError` (or equivalent) defined with `Transport(#[from] …)` and `DataNotReady`
- [ ] `ContinuousSensor::next_measurement` is a single non-blocking attempt (no loops, no delays)
- [ ] `OneshotSensor::Error` matches `ContinuousSensor::Error` when `measure` calls
      `next_measurement`
- [ ] All delays tied to specific commands or derived from device configuration
- [ ] `defmt::Format` derived behind `#[cfg_attr(feature = "defmt", …)]`
- [ ] Feature flag added to `Cargo.toml` and to `all-devices`
- [ ] Module registered in `src/devices/<vendor>/mod.rs`
