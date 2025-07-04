/// A device which supports any form of software-triggered reset.
#[maybe_async_cfg::maybe(sync(feature = "sync"), async(feature = "async"))]
#[allow(async_fn_in_trait)]
pub trait ResettableDevice {
    /// The error type which may occur when resetting the device
    type Error;

    /// Performs a best-effort soft-reset of the device.
    async fn reset(&mut self) -> Result<(), Self::Error>;
}
