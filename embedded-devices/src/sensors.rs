use paste::paste;
use uom::si::f64::{
    ElectricCharge, ElectricCurrent, ElectricPotential, Energy, Power, Pressure, Ratio, ThermodynamicTemperature,
};

/// This trait is implemented for any sensor specific measurement struct.
pub trait Measurement {}

/// A sensor.
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(feature = "sync"),
    async(feature = "async")
)]
#[allow(async_fn_in_trait)]
pub trait Sensor {
    /// A type holding the measurements obtained by this sensor
    type Measurement: self::Measurement;
    /// The error type which may occur when measuring
    type Error;

    /// Performs a one-shot measurement
    async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Self::Measurement, Self::Error>;

    // TODO: calling measure while in continuous mode will automatically switch the mode to oneshot.
    // TODO: set_mode(M)
    //   if supported: set sensor into continuous/oneshot mode
    //   if not: noop
    // TODO: current_mode()
    //   if supported: get current mode
    //   if not: return single supported mode
    // TODO: read_last_measurement()
    //   if supported: read last measurement
    //   if not:       return None
    // TODO: conversion_time()
    //   if supported: Some(t_conv)
    //   if not:       None
    // TODO: default impl: wait_for_measurement()
    //   if supported: wait one period of conversion time and return read_last_measurement
    //   if not:       measure()
}

/// Macro to define a sensor trait and its corresponding measurement trait
///
/// # Arguments
/// - `$sensor_name` - The name of the sensor (e.g., Temperature, Pressure)
/// - `$measurement_type` - The Rust type for the measurement (e.g., ThermodynamicTemperature, Pressure)
/// - `$measurement_desc` - A lowercase description of what is measured (e.g., "temperature", "pressure")
macro_rules! define_sensor_measurement {
    (
        $sensor_name:ident,
        $measurement_type:ty,
        $measurement_desc:expr
    ) => {
        paste! {
            #[doc = concat!("A sensor which measures ", $measurement_desc)]
            pub trait [<$sensor_name Sensor>] {}

            #[doc = concat!("A measurement result which may include a ", $measurement_desc, " measurement")]
            pub trait [<$sensor_name Measurement>]: Measurement {
                fn [<$sensor_name:snake>](&self) -> Option<$measurement_type>;
            }
        }
    };
}

define_sensor_measurement!(Charge, ElectricCharge, "charge");
define_sensor_measurement!(Current, ElectricCurrent, "current");
define_sensor_measurement!(Energy, Energy, "energy");
define_sensor_measurement!(Power, Power, "power");
define_sensor_measurement!(Pressure, Pressure, "pressure");
define_sensor_measurement!(RelativeHumidity, Ratio, "relative humidity");
define_sensor_measurement!(Temperature, ThermodynamicTemperature, "temperature");
define_sensor_measurement!(Voltage, ElectricPotential, "voltage");
define_sensor_measurement!(Co2Concentration, Ratio, "co2 concentration");
