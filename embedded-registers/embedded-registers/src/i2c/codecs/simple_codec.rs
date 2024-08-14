use crate::{
    i2c::{Codec, I2cBoundBus},
    ReadableRegister, WritableRegister,
};
use bytemuck::Zeroable;

/// This codec represents the most commonly found codecs for I2C devices.
/// The main variable is the size of register addresses in bytes.
///
/// This codec has no information over register sizes or the existence of
/// read/write address-auto-increment which some devices support.
/// It will always send one header and then receive/send the associated register data,
/// so it's compatible with auto-increment usage, but cannot be used to read or write
/// registers that require interspersing the address between bytes.
///
/// Devices often use a mixed mode, where some registers allow auto-increment while others
/// don't, or where the address is directly associated with a specific, but varying, register size.
/// Therefore, it is up to the user to make sure that accessing a register via this codec
/// is supported by the hardware.
///
/// The following generic parameters are available:
///
/// | Parameter | Type | Description |
/// |---|---|---|
/// | `HEADER_SIZE` | `usize` | The size of the command header (register address) in bytes |
#[derive(Default)]
pub struct SimpleCodec<const HEADER_SIZE: usize> {}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<const HEADER_SIZE: usize> Codec for SimpleCodec<HEADER_SIZE> {
    #[inline]
    async fn read_register<R, I>(&mut self, bound_bus: &mut I2cBoundBus<I>) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::i2c::I2c + hal::i2c::ErrorType,
    {
        let header = &R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..];
        let mut register = R::zeroed();

        bound_bus
            .interface
            .write_read(bound_bus.address, header, register.data_mut())
            .await?;
        Ok(register)
    }

    #[inline]
    async fn write_register<R, I>(
        &mut self,
        bound_bus: &mut I2cBoundBus<I>,
        register: impl AsRef<R>,
    ) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::i2c::I2c + hal::i2c::ErrorType,
    {
        #[repr(C, packed(1))]
        #[derive(Copy, Clone, bytemuck::Pod, Zeroable)]
        struct Buffer<const HEADER_SIZE: usize, R> {
            header: [u8; HEADER_SIZE],
            register: R,
        }

        let mut buffer = Buffer::<{ HEADER_SIZE }, R> {
            header: R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..]
                .try_into()
                .expect("Unexpected compile-time header address size. This is a bug in the chosen Codec or embedded-registers."),
            register: *register.as_ref(),
        };

        let data = bytemuck::bytes_of_mut(&mut buffer);
        bound_bus.interface.write(bound_bus.address, data).await
    }
}
