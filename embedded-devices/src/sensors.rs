use uom::si::f64::{Pressure, Ratio, ThermodynamicTemperature};

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
}

/// A sensor which measures temperature
#[maybe_async_cfg::maybe(idents(Sensor), sync(feature = "sync"), async(feature = "async"))]
pub trait TemperatureSensor: Sensor {}
/// A measurement result which may include a temperature measurement
pub trait TemperatureMeasurement: Measurement {
    fn temperature(&self) -> Option<ThermodynamicTemperature>;
}

/// A sensor which measures pressure
#[maybe_async_cfg::maybe(idents(Sensor), sync(feature = "sync"), async(feature = "async"))]
pub trait PressureSensor: Sensor {}
/// A measurement result which may include a pressure measurement
pub trait PressureMeasurement: Measurement {
    fn pressure(&self) -> Option<Pressure>;
}

/// A sensor which measures relative humidity
#[maybe_async_cfg::maybe(idents(Sensor), sync(feature = "sync"), async(feature = "async"))]
pub trait RelativeHumiditySensor: Sensor {}
/// A measurement result which may include a relative humidity measurement
pub trait RelativeHumidityMeasurement: Measurement {
    fn relative_humidity(&self) -> Option<Ratio>;
}
