use core::num::Wrapping;

use embedded_interfaces::TransportError;
use embedded_interfaces::registers::{ReadableRegister, Register, RegisterCodec, WritableRegister};

#[cfg(feature = "belling-bl0942")]
pub mod bl0942;

/// An error representing checksum errors.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, thiserror::Error)]
pub enum ChecksumError {
    #[error("the calculated checksum {calculated:#02x} did not match the expected value {expected:#02x}")]
    Mismatch { calculated: u8, expected: u8 },
}

/// This represents the SPI codec most commonly used in Belling devices. It consists of a static
/// command (read or write), a 8 bit register address, 24 bits of a data and an additive checksum.
///
/// ```text
/// # Read
/// MOSI | CMD_READ[7:0]  | ADDR[7:0] |     0               0      |
/// MISO |        0            0      | DATA[23:0] | CHECKSUM[7:0] |
///
/// # Write
/// MOSI | CMD_WRITE[7:0] | ADDR[7:0] | DATA[23:0] | CHECKSUM[7:0] |
/// ```
pub struct BellingSpiCodec<const CMD_READ: u8, const CMD_WRITE: u8> {}

impl<const CMD_READ: u8, const CMD_WRITE: u8> RegisterCodec for BellingSpiCodec<CMD_READ, CMD_WRITE> {
    type Error = ChecksumError;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<const CMD_READ: u8, const CMD_WRITE: u8> embedded_interfaces::registers::spi::Codec
    for BellingSpiCodec<CMD_READ, CMD_WRITE>
{
    #[inline]
    async fn read_register<R, I>(interface: &mut I) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::spi::r#SpiDevice,
    {
        assert_eq!(R::REGISTER_SIZE, 3);
        let mut buffer = [CMD_READ, R::ADDRESS as u8, 0, 0, 0, 0];
        interface.transfer_in_place(&mut buffer).await?;

        buffer[0] = CMD_READ;
        buffer[1] = R::ADDRESS as u8;
        let data = &buffer[2..5];
        let calculated = !(buffer[0..5].iter().map(|&x| Wrapping(x)).sum::<Wrapping<u8>>().0);
        let expected = buffer[5];
        if expected != calculated {
            return Err(TransportError::r#Codec(ChecksumError::Mismatch {
                calculated,
                expected,
            }));
        }

        // Moving it into a variable ensures alignment.
        let mut register = R::zeroed();
        bytemuck::bytes_of_mut(&mut register).copy_from_slice(data);

        #[cfg(feature = "trace-communication")]
        log::trace!(
            "SPI [32mread[m register_addr={:08x}:\n{}",
            R::ADDRESS,
            register.bitdump()
        );

        Ok(register)
    }

    #[inline]
    async fn write_register<R, I>(
        interface: &mut I,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::spi::r#SpiDevice,
    {
        assert_eq!(R::REGISTER_SIZE, 3);

        #[cfg(feature = "trace-communication")]
        log::trace!(
            "SPI [32mwrite[m register_addr={:08x}:\n{}",
            R::ADDRESS,
            register.as_ref().bitdump()
        );

        let mut buffer = [CMD_WRITE, R::ADDRESS as u8, 0, 0, 0, 0];
        buffer[2..5].copy_from_slice(bytemuck::bytes_of(register.as_ref()));

        // Checksum
        let checksum = !(buffer[0..5].iter().map(|&x| Wrapping(x)).sum::<Wrapping<u8>>().0);
        buffer[5] = checksum;

        Ok(interface.write(&buffer).await?)
    }
}
