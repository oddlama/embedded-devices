#[cfg(feature = "sensirion-scd4x")]
pub mod scd4x;

use crc::{Algorithm, CRC_8_NRSC_5};
use embedded_registers::i2c::codecs::crc_codec::{Crc8Algorithm, Crc8Codec};

#[derive(Default)]
pub struct SCD4xCrcCodec {}
impl Crc8Algorithm for SCD4xCrcCodec {
    fn instance() -> &'static Algorithm<u8> {
        const CUSTOM_ALG: crc::Algorithm<u8> = CRC_8_NRSC_5;
        &CUSTOM_ALG
    }
}

pub type SensirionI2cCodec = Crc8Codec<2, 2, SCD4xCrcCodec>;
