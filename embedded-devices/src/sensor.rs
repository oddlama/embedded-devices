use paste::paste;
use uom::si::f64::{
    ElectricCharge, ElectricCurrent, ElectricPotential, Energy, Illuminance, MassConcentration, Power, Pressure, Ratio,
    ThermodynamicTemperature, Velocity,
};

/// This trait is implemented for any sensor specific measurement struct.
pub trait Measurement: core::fmt::Debug {}

/// A sensor supporting one-shot measurements.
///
/// If the hardware device itself doesn't support one-shot measurements but they can be
/// meaningfully emulated by starting and stopping continuous measurements without drawbacks, it
/// may also implement this trait. If the sensor requires a certain amount of time in continuous
/// measurement mode to yield meaningful results, this trait should not be implemented.
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
#[allow(async_fn_in_trait)]
pub trait OneshotSensor {
    /// The error type which may occur on associated functions
    type Error;
    /// A type holding the measurements obtained by this sensor
    type Measurement: self::Measurement;

    /// Performs a one-shot measurement.
    ///
    /// If the device supports several measurement modes, this function attempts to switch to
    /// oneshot mode on best effort. The mode in which the device is left after measurement is not
    /// specified further, but if possible, a sleep mode or equivalent mode should be preferred.
    async fn measure(&mut self) -> Result<Self::Measurement, Self::Error>;
}

/// A sensor supporting continuous measurement mode.
///
/// If the hardware device itself doesn't support continuous measurements but they can be
/// meaningfully emulated by using a one-shot mode without drawbacks, it may also implement this
/// trait.
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
#[allow(async_fn_in_trait)]
pub trait ContinuousSensor {
    /// The error type which may occur on associated functions
    type Error;
    /// A type holding the measurements obtained by this sensor
    type Measurement: self::Measurement;

    /// Starts continuous measurement.
    ///
    /// If the device supports several measurement modes, this function attempts to switch to
    /// continuous mode on best effort.
    async fn start_measuring(&mut self) -> Result<(), Self::Error>;

    /// Stops continuous measurement.
    ///
    /// The mode in which the device is left afterwards is not specified further, but if possible,
    /// a sleep mode or equivalent mode should be preferred.
    async fn stop_measuring(&mut self) -> Result<(), Self::Error>;

    /// Expected amount of time between measurements in microseconds. This may need to communicate
    /// with the device to determine the interval based on device settings.
    async fn measurement_interval_us(&mut self) -> Result<u32, Self::Error>;

    /// Returns the most recent measurement. If the sensor cannot provide the measurement
    /// current/last measurement at any time (e.g. the register is cleared after it is read), this
    /// should return `None` when there is no new data available.
    async fn current_measurement(&mut self) -> Result<Option<Self::Measurement>, Self::Error>;

    /// Check if new measurements are available. If the device does not support checking for
    /// availability of new measurements (e.g. via some form of data ready indicator), this method
    /// should always return `true`.
    async fn is_measurement_ready(&mut self) -> Result<bool, Self::Error>;

    /// Wait indefinitely until new measurements are available and return them. If the device does
    /// not support checking for new measurements (e.g. via some form of data ready indicator),
    /// this method should wait for [`Self::measurement_interval_us`] and read the measurement
    /// opportunistically.
    async fn next_measurement(&mut self) -> Result<Self::Measurement, Self::Error>;
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
define_sensor_measurement!(Co2Concentration, Ratio, "CO2 concentration");
define_sensor_measurement!(Current, ElectricCurrent, "current");
define_sensor_measurement!(Energy, Energy, "energy");
define_sensor_measurement!(HchoConcentration, Ratio, "HCHO concentration");
define_sensor_measurement!(Illuminance, Illuminance, "illuminance");
define_sensor_measurement!(NoxIndex, i16, "NOx index");
define_sensor_measurement!(Pm10Concentration, MassConcentration, "PM10 concentration");
define_sensor_measurement!(Pm1Concentration, MassConcentration, "PM1 concentration");
define_sensor_measurement!(Pm2_5Concentration, MassConcentration, "PM2.5 concentration");
define_sensor_measurement!(Pm4Concentration, MassConcentration, "PM4 concentration");
define_sensor_measurement!(Power, Power, "power");
define_sensor_measurement!(Pressure, Pressure, "pressure");
define_sensor_measurement!(RelativeHumidity, Ratio, "relative humidity");
define_sensor_measurement!(SoilMoisture, Ratio, "soil moisture");
define_sensor_measurement!(Temperature, ThermodynamicTemperature, "temperature");
define_sensor_measurement!(VocIndex, i16, "VOC index");
define_sensor_measurement!(Voltage, ElectricPotential, "voltage");
define_sensor_measurement!(WindSpeed, Velocity, "wind speed");
