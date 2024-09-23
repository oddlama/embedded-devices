#![no_std]
#![no_main]

use core::future::Future;
use core::pin::Pin;
use core::task::Poll;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, peripherals::Peripherals, prelude::*, system::SystemControl,
};
use esp_hal::gpio::Io;
use esp_hal::i2c::I2C;
use uom::si::thermodynamic_temperature::degree_celsius;
use embedded_devices::devices::bosch::{bme280, bme280::BME280};
use uom::num_traits::ToPrimitive;
use embedded_hal::delay::DelayNs;
use uom::si::pressure::{kilopascal, pascal};
use uom::si::ratio::percent;
use uom::si::rational32::ThermodynamicTemperature;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();


    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sda = io.pins.gpio12;
    let scl = io.pins.gpio13;

    let i2c = I2C::new(peripherals.I2C0, sda, scl, 100u32.kHz(), &clocks);

    /// intelij stops working here, something in the macros might be odd or an issuns with remote development.
    let mut bme280 = BME280::new_i2c(i2c, bme280::address::Address::Primary);

    loop {
        match bme280.calibrate() {
            Ok(_) => break,
            Err(e) => {
                log::error!("bme280 did not respond correctly, waiting 500ms");
                delay.delay_ms(500_u32);
            }
        }
    }

    loop {
        let measurements_result = bme280.measure(&mut delay);

        let measurements = match measurements_result {
            Ok(v) => v,
            Err(e) => {
                log::error!("error {e:?}");
                continue;
            }
        };

        let temperature = measurements.temperature.get::<degree_celsius>().to_f32().unwrap();
        /// Current pressure or None if the sensor reported and invalid value
        match measurements.pressure{
            Some(pressure) => {
                let pressure = pressure.get::<pascal>().to_f32().unwrap();
                log::info!("Pressure = {} pascals", pressure);
            },
            None => log::info!("sensore did not reported a pressure or an invalide value")
        }
        /// Current relative humidity
        let humidity = measurements.humidity.get::<percent>().to_f32().unwrap();

        log::info!("Relative Humidity = {}%", humidity);
        log::info!("Temperature = {} deg C", temperature);



        delay.delay_ms(100_u32);
    }
}
