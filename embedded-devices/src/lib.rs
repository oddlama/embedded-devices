//! Device driver implementations for many embedded sensors and devices
//!
#![cfg_attr(not(doctest), doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md")))]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(generic_arg_infer)]

pub mod devices;
pub(crate) mod simple_device;
pub mod utils;

#[cfg(test)]
mod tests {
    #[cfg(feature = "bosch-bme280")]
    #[test]
    fn register_bme280_reset() {
        use crate::devices::bosch::bme280::registers::{Reset, ResetBitfield, ResetMagic};
        let value = ResetBitfield {
            magic: ResetMagic::Reset,
        };

        let reg = Reset::new(value.clone());
        assert_eq!(reg.data, [0xb6]);
        assert_eq!(reg.read_all(), value);
    }

    #[cfg(feature = "bosch-bmp390")]
    #[test]
    fn register_bmp390_error() {
        use crate::devices::bosch::bmp390::registers::{Error, ErrorBitfield};
        let value = ErrorBitfield {
            fatal_err: true,
            cmd_err: false,
            conf_err: true,
            reserved: 0,
        };

        let reg = Error::new(value.clone());
        assert_eq!(reg.data, [0b00000101]);
        assert_eq!(reg.read_all(), value);
    }

    #[cfg(feature = "bosch-bmp390")]
    #[test]
    fn register_bmp390_pressure() {
        use crate::devices::bosch::bmp390::registers::{Pressure, PressureBitfield};
        let value = PressureBitfield { pressure: 0x123456 };

        let reg = Pressure::new(value.clone());
        assert_eq!(reg.data, [0x56, 0x34, 0x12]);
        assert_eq!(reg.read_all(), value);
    }

    #[cfg(feature = "texas_instruments-ina228")]
    #[test]
    fn register_defaults() {
        use crate::devices::texas_instruments::ina228::registers::AdcConfiguration;
        let reg = AdcConfiguration::default();
        assert!(reg.read_enable_temperature());
    }

    // FIXME: once bondrewd #[test]
    // FIXME: once bondrewd fn register_bmp390_fifo_length() {
    // FIXME: once bondrewd     let value = FifoLengthBitfield {
    // FIXME: once bondrewd         reserved: 0b0100010,
    // FIXME: once bondrewd         length: 0b110000101,
    // FIXME: once bondrewd     };

    // FIXME: once bondrewd     let reg = FifoLength::new(value.clone());
    // FIXME: once bondrewd     assert_eq!(reg.data, [0b10000101, 0b01000101]);
    // FIXME: once bondrewd     assert_eq!(reg.read_all(), value);
    // FIXME: once bondrewd }
}
