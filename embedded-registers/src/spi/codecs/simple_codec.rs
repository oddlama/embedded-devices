use crate::{spi::Codec, ReadableRegister, Register, WritableRegister};
use bytemuck::Zeroable;

/// This codec represents the most commonly found codecs for SPI devices.
/// It consists of an N-bit big-endian register address, a 1-bit R/W indicator
/// and uses zero initializion for reserved bits. The header is always a multiple of 8-bit
/// in width.
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
/// | `HEADER_SIZE`              | `usize` | The size of the command header in bytes |
/// | `ADDR_MSB`                 | `u8`    | The bit index of the MSB of the register-address (inclusive) |
/// | `ADDR_LSB`                 | `u8`    | The bit index of the LSB of the register-address (inclusive) |
/// | `RW_BIT`                   | `u8`    | The bit index of the RW bit when interpreting the struct in big-endian |
/// | `RW_1_IS_READ`             | `bool`  | whether the setting the RW bit signals read-mode (true) or write-mode (false) |
/// | `READ_DELAY`               | `usize` | Number of bytes that we have to wait (send additional zeros) after sending the header until data arrives |
#[derive(Default)]
pub struct SimpleCodec<
    const HEADER_SIZE: usize,
    const ADDR_MSB: u8,
    const ADDR_LSB: u8,
    const RW_BIT: u8,
    const RW_1_IS_READ: bool,
    const READ_DELAY: usize,
> {}

impl<
        const HEADER_SIZE: usize,
        const ADDR_MSB: u8,
        const ADDR_LSB: u8,
        const RW_BIT: u8,
        const RW_1_IS_READ: bool,
        const READ_DELAY: usize,
    > SimpleCodec<HEADER_SIZE, ADDR_MSB, ADDR_LSB, RW_BIT, RW_1_IS_READ, READ_DELAY>
{
    #[inline]
    pub fn fill_addr_header<R>(header: &mut [u8])
    where
        R: Register,
    {
        // create a mask with ADDR_MSB ones.
        let addr_mask = u64::checked_shl(1, ADDR_MSB as u32 + 1).unwrap_or(0).wrapping_sub(1);
        // Shift the address to the correct place
        let addr_shifted = (R::ADDRESS << ADDR_LSB) & addr_mask;
        // incorporate addess
        let addr_bytes = addr_shifted.to_le_bytes();
        let affected_bytes = ((ADDR_MSB - ADDR_LSB) / 8) as usize;
        for i in 0..=affected_bytes {
            header[HEADER_SIZE - 1 - i] |= addr_bytes[i];
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
    sync(not(feature = "async")),
    async(feature = "async"),
    keep_self
)]
impl<
        const HEADER_SIZE: usize,
        const ADDR_MSB: u8,
        const ADDR_LSB: u8,
        const RW_BIT: u8,
        const RW_1_IS_READ: bool,
        const READ_DELAY: usize,
    > Codec for SimpleCodec<HEADER_SIZE, ADDR_MSB, ADDR_LSB, RW_BIT, RW_1_IS_READ, READ_DELAY>
{
    #[inline]
    async fn read_register<R, I>(&mut self, interface: &mut I) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::spi::SpiDevice,
    {
        #[repr(C, packed(1))]
        #[derive(Copy, Clone, bytemuck::Pod, Zeroable)]
        struct Buffer<const HEADER_SIZE: usize, const READ_DELAY: usize, R> {
            header: [u8; HEADER_SIZE],
            delay: [u8; READ_DELAY],
            register: R,
        }

        let mut buffer = Buffer::<{ HEADER_SIZE }, { READ_DELAY }, R>::zeroed();
        let data = bytemuck::bytes_of_mut(&mut buffer);
        // Set RW_BIT if necessary
        data[HEADER_SIZE - 1 - (RW_BIT as usize) / 8] |= (RW_1_IS_READ as u8) << (RW_BIT % 8);
        Self::fill_addr_header::<R>(data);
        interface.transfer_in_place(data).await?;
        Ok(buffer.register)
    }

    #[inline]
    async fn write_register<R, I>(&mut self, interface: &mut I, register: impl AsRef<R>) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::spi::SpiDevice,
    {
        #[repr(C, packed(1))]
        #[derive(Copy, Clone, bytemuck::Pod, Zeroable)]
        struct Buffer<const HEADER_SIZE: usize, R> {
            header: [u8; HEADER_SIZE],
            register: R,
        }

        let mut buffer = Buffer::<{ HEADER_SIZE }, R> {
            header: [0u8; HEADER_SIZE],
            register: *register.as_ref(),
        };

        let data = bytemuck::bytes_of_mut(&mut buffer);
        // Set RW_BIT if necessary
        data[HEADER_SIZE - 1 - (RW_BIT as usize) / 8] |= ((!RW_1_IS_READ) as u8) << (RW_BIT % 8);
        Self::fill_addr_header::<R>(data);
        interface.write(data).await
    }
}
