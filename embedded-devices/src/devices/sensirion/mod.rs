#[cfg(feature = "sensirion-scd4x")]
pub mod scd4x;

use crc::CRC_8_NRSC_5;
use embedded_registers::{define_crc_algo, i2c::codecs::crc8_codec::Crc8Codec};

define_crc_algo!(SensirionCrcAlgorithm, CRC_8_NRSC_5);
pub type SensirionI2cCodec = Crc8Codec<2, 2, SensirionCrcAlgorithm>;
