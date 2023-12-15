//! Device driver implementations for many embedded sensors and devices
//!
#![doc = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md"))]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(generic_arg_infer)]

pub mod devices;
pub(crate) mod simple_device;

#[cfg(test)]
mod tests {
    use crate::devices::bosch::{
        bme280::registers::{Reset, ResetBitfield, ResetMagic},
        bmp390::registers::{Error, ErrorBitfield, FifoLength, FifoLengthBitfield, Pressure, PressureBitfield},
    };

    #[test]
    fn register_bme280_reset() {
        let value = ResetBitfield {
            magic: ResetMagic::Reset,
        };

        let reg = Reset::new(value.clone());
        assert_eq!(reg.data, [0xb6]);
        assert_eq!(reg.read_all(), value);
    }

    #[test]
    fn register_bmp390_error() {
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

    #[test]
    fn register_bmp390_pressure() {
        let value = PressureBitfield { pressure: 0x123456 };

        let reg = Pressure::new(value.clone());
        assert_eq!(reg.data, [0x56, 0x34, 0x12]);
        assert_eq!(reg.read_all(), value);
    }

    #[test]
    fn register_bmp390_fifo_length() {
        let value = FifoLengthBitfield {
            reserved: 0b0100010,
            length: 0b110000101,
        };

        let reg = FifoLength::new(value.clone());
        assert_eq!(reg.data, [0b10000101, 0b01000101]);
        assert_eq!(reg.read_all(), value);
    }
}
