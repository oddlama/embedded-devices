#[cfg(any(
    feature = "sensirion-scd40",
    feature = "sensirion-scd41",
    feature = "sensirion-scd43",
))]
pub(crate) mod scd4x;

#[cfg(feature = "sensirion-scd40")]
pub mod scd40;
#[cfg(feature = "sensirion-scd41")]
pub mod scd41;
#[cfg(feature = "sensirion-scd43")]
pub mod scd43;

#[cfg(any(
    feature = "sensirion-sen63c",
    feature = "sensirion-sen65",
    feature = "sensirion-sen66",
    feature = "sensirion-sen68"
))]
pub(crate) mod sen6x;

#[cfg(feature = "sensirion-sen60")]
pub mod sen60;
#[cfg(feature = "sensirion-sen63c")]
pub mod sen63c;
#[cfg(feature = "sensirion-sen65")]
pub mod sen65;
#[cfg(feature = "sensirion-sen66")]
pub mod sen66;
#[cfg(feature = "sensirion-sen68")]
pub mod sen68;

use crc::CRC_8_NRSC_5;
use embedded_registers::{define_crc_algo, i2c::codecs::crc8_codec::Crc8Codec};

define_crc_algo!(SensirionCrcAlgorithm, CRC_8_NRSC_5);

/// Default codec used by sensirion devices
pub type SensirionI2cCodec = Crc8Codec<2, 2, SensirionCrcAlgorithm>;
/// Some operations require a write followed by read without sending the command
/// header twice. To allow this through the register interface, those commands
/// can be split into a write only and a read only register, where the latter
/// uses a 0-byte command header available through this codec.
pub type SensirionI2cCodecConsecutiveFetch = Crc8Codec<0, 2, SensirionCrcAlgorithm>;

/// This trait should be implemented by any registers that represent sensirion commands.
pub trait SensirionCommand {
    /// This reflects the execution time for this command in milliseconds, if specified in the
    /// datasheet. Read operations do not need to await this time since the I2C transaction will
    /// implicitly await it when the expected data is read from the bus.
    const EXECUTION_TIME_MS: u32;
    /// Whether this command can be executed during measurement
    const DURING_MEASUREMENT: bool;
}

#[allow(unused)]
macro_rules! sensirion_command {
    ($register:ident, None, $during_measurement:expr) => {
        sensirion_command!(@define, $register, 0, $during_measurement);
    };
    ($register:ident, $execution_time_ms:expr, $during_measurement:expr) => {
        sensirion_command!(@define, $register, $execution_time_ms, $during_measurement);
    };
    (@define, $register:ident, $execution_time_ms:expr, $during_measurement:expr) => {
        impl crate::devices::sensirion::SensirionCommand for $register {
            const EXECUTION_TIME_MS: u32 = $execution_time_ms;
            const DURING_MEASUREMENT: bool = $during_measurement;
        }
    };
}
#[allow(unused)]
pub(crate) use sensirion_command;
