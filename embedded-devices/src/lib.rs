//! Device driver implementations for many embedded sensors and devices
//!
#![cfg_attr(not(doctest), doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md")))]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(generic_arg_infer)]

pub mod device;
pub mod devices;
pub mod sensor;
pub mod utils;

pub use uom;

#[cfg(test)]
mod tests {
    #[cfg(feature = "bosch-bme280")]
    #[test]
    fn register_bme280_reset() {
        use crate::devices::bosch::bme280::registers::{ResetMagic, ResetUnpacked};
        let value = ResetUnpacked {
            magic: ResetMagic::Reset,
        };

        let reg = value.pack();
        assert_eq!(reg.0, [0xb6]);
        assert_eq!(reg.unpack(), value);
    }

    #[cfg(feature = "bosch-bmp390")]
    #[test]
    fn register_bmp390_error() {
        use crate::devices::bosch::bmp390::registers::ErrorUnpacked;
        let value = ErrorUnpacked {
            fatal_err: true,
            cmd_err: false,
            conf_err: true,
        };

        let reg = value.pack();
        assert_eq!(reg.0, [0b00000101]);
        assert_eq!(reg.unpack(), value);
    }

    #[cfg(feature = "bosch-bmp390")]
    #[test]
    fn register_bmp390_pressure() {
        use crate::devices::bosch::bmp390::registers::PressureUnpacked;
        let value = PressureUnpacked { value: 0x123456 };

        let reg = value.pack();
        assert_eq!(reg.0, [0x56, 0x34, 0x12]);
        assert_eq!(reg.unpack(), value);
    }

    #[cfg(feature = "texas_instruments-ina228")]
    #[test]
    fn register_defaults() {
        use crate::devices::texas_instruments::ina228::registers::AdcConfiguration;
        let reg = AdcConfiguration::default();
        assert!(reg.read_enable_temperature());
    }

    #[cfg(feature = "bosch-bmp390")]
    #[test]
    fn register_bmp390_fifo_length() {
        use crate::devices::bosch::bmp390::registers::FifoLengthUnpacked;
        let value = FifoLengthUnpacked { length: 0b110000101 };

        let reg = value.pack();
        assert_eq!(reg.0, [0b10000101, 0b00000001]);
        assert_eq!(reg.unpack(), value);
    }
}
