pub mod commands;

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
