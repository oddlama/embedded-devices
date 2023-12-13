//! # BMP390
//!
//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
//! module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
//! mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
//! driven devices such as mobile phones, GPS modules or watches.

use embedded_devices_derive::{device, device_impl};
use embedded_registers::{i2c::I2cDevice, RegisterInterface};
use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::ratio::percent;
use uom::si::rational32::{Pressure, Ratio, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::address::Address;
use self::registers::{
    BurstMeasurementsPTH, Config, ControlHumidity, ControlMeasurement, IIRFilter, Id, Oversampling, SensorMode,
    TrimmingParameters1, TrimmingParameters2,
};

/// All possible errors that may occur when using this device
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid ChipId was encountered in `init`
    InvalidChipId(registers::ChipId),
    /// The calibration data was not yet read from the device, but a measurement was requested. Call `init` or `calibrate` first.
    NotCalibrated,
    /// NVM data copy is still in progress.
    NvmCopyInProgress,
}

/// The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor
/// module is housed in an extremely compact 10-pin metal-lid LGA package with a footprint of only 2.0 × 2.0 mm² and max 0.8
/// mm package height. Its small dimensions and its low power consumption of 3.2 μA @1Hz allow the implementation in battery
/// driven devices such as mobile phones, GPS modules or watches.
#[device]
pub struct BMP390<I: RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,
}

/// Common configuration values for the BME280 sensor.
/// The power-on-reset default is to set all oversampling settings to 1X
/// and use no IIR filter.
#[derive(Debug, Clone, Default)]
pub struct Configuration {
    /// The oversampling rate for temperature mesurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pub pressure_oversampling: Oversampling,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<I> BMP390<I2cDevice<I>>
where
    I: hal::i2c::I2c + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
    #[inline]
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
            interface: I2cDevice {
                interface,
                address: address.into(),
            },
        }
    }
}
