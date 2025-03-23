use clap::Parser;
use embedded_devices::devices::texas_instruments::tmp102::{TMP102, address::Address, registers::Configuration};
use linux_embedded_hal::{Delay, I2cdev};
use uom::num_traits::ToPrimitive;
use uom::si::thermodynamic_temperature::degree_celsius;

/// Read an TMP102 temperature sensor
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Bus number (`/dev/i2c<x>`)
    #[arg()]
    bus: u8,
}

fn main() {
    let args = Args::parse();
    let i2c = I2cdev::new(format!("/dev/i2c-{}", args.bus)).unwrap();

    // Create and initialize the device. Default conversion mode is continuous.
    let mut tmp102 = TMP102::new_i2c(i2c, Address::Gnd);

    // display the configuration register
    let cfg = tmp102.read_register::<Configuration>();
    println!("cfg: {:?}", cfg);

    // display the last converted temperature
    let temp = tmp102
        .read_temperature()
        .unwrap()
        .get::<degree_celsius>()
        .to_f32()
        .unwrap();
    println!("last temp: {:?}°C", temp);

    // start a conversion and display the temperature
    let temp = tmp102
        .oneshot(&mut Delay)
        .unwrap()
        .get::<degree_celsius>()
        .to_f32()
        .unwrap();
    println!("temp: {:?}°C", temp);
}
