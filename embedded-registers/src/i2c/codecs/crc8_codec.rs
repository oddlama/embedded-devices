use core::marker::PhantomData;

use crate::{ReadableRegister, Register, RegisterCodec, TransportError, WritableRegister};
use bytemuck::Zeroable;
use crc::Algorithm;
use heapless::Vec;

/// This codec implements an I2C codec which calculates and expects a 1-byte CRC checksum after
/// every N bytes of data, excluding the register address. This codec does not know anything about
/// the register contents, it operates purely on the raw data bytes.
///
/// The following generic parameters are available:
///
/// | Parameter | Type | Description |
/// |---|---|---|
/// | `HEADER_SIZE` | `usize` | The size of the command header (register address) in bytes |
/// | `CHUNK_SIZE` | `usize` | The chunk size in bytes. Each chunk is followed by a 1-byte CRC checksum |
/// | `C` | [`Crc8Algorithm`] | A type that provides a static reference to the CRC algorithm |
///
/// You can a wrapper that implements [`Crc8Algorithm`] for a given crc::Algorithm by
/// calling the `define_crc_algo!` macro:
///
/// ```rust
/// use embedded_registers::define_crc_algo;
///
/// define_crc_algo!(MyCrc, crc::CRC_8_NRSC_5);
/// //define_crc_algo!(MyCrc, crc::Algorithm { /* ... */ });
/// ```
///
/// To create a custom instance, you can implement your own wrapper:
///
/// ```rust
/// use embedded_registers::i2c::codecs::crc8_codec::Crc8Algorithm;
/// use crc::{Algorithm, CRC_8_NRSC_5};
///
/// #[derive(Default)]
/// struct MyCrc {}
///
/// impl Crc8Algorithm for MyCrc {
///     fn instance() -> &'static Algorithm<u8> {
///         const CUSTOM_ALG: crc::Algorithm<u8> = CRC_8_NRSC_5;
///         &CUSTOM_ALG
///     }
/// }
/// ```
#[derive(Default)]
pub struct Crc8Codec<const HEADER_SIZE: usize, const CHUNK_SIZE: usize, C: Crc8Algorithm> {
    _algo: PhantomData<C>,
}

pub trait Crc8Algorithm: Default {
    /// Return reference to global static CRC Algorithm
    fn instance() -> &'static Algorithm<u8>;
}

#[macro_export]
macro_rules! define_crc_algo {
    ($name:ident, $algo:expr) => {
        #[derive(Default)]
        pub struct $name {}
        impl embedded_registers::i2c::codecs::crc8_codec::Crc8Algorithm for $name {
            fn instance() -> &'static crc::Algorithm<u8> {
                const CUSTOM_ALG: crc::Algorithm<u8> = $algo;
                &CUSTOM_ALG
            }
        }
    };
}

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum CrcError {
    #[error("the calculated crc checksum {calculated:#02x} did not match the expected value {expected:#02x}")]
    CrcMismatch { calculated: u8, expected: u8 },
}

impl<const HEADER_SIZE: usize, const CHUNK_SIZE: usize, C: Crc8Algorithm> RegisterCodec
    for Crc8Codec<HEADER_SIZE, CHUNK_SIZE, C>
{
    type Error = CrcError;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<const HEADER_SIZE: usize, const CHUNK_SIZE: usize, C: Crc8Algorithm> crate::i2c::Codec
    for Crc8Codec<HEADER_SIZE, CHUNK_SIZE, C>
{
    #[inline]
    async fn read_register<R, I, A>(
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
    ) -> Result<R, TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let crc = crc::Crc::<u8>::new(C::instance());
        let header = &R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..];

        let mut array = Vec::<u8, 64>::new();
        array
            .resize_default(R::REGISTER_SIZE + R::REGISTER_SIZE / CHUNK_SIZE)
            .expect(
                "Unexpected compile-time header address size. This is a bug in the chosen Codec or embedded-registers.",
            );

        bound_bus
            .interface
            .write_read(bound_bus.address, header, &mut array)
            .await?;

        let mut register = R::zeroed();
        let data = bytemuck::bytes_of_mut(&mut register);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[i..i + CHUNK_SIZE].copy_from_slice(value);

            let calculated = crc.checksum(value);
            let expected = x[CHUNK_SIZE];
            if expected != calculated {
                return Err(TransportError::r#Codec(CrcError::CrcMismatch { calculated, expected }));
            }
        }

        Ok(register)
    }

    #[inline]
    async fn write_register<R, I, A>(
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        register: impl AsRef<R>,
    ) -> Result<(), TransportError<Self::Error, I::Error>>
    where
        R: Register<CodecError = Self::Error> + WritableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let crc = crc::Crc::<u8>::new(C::instance());
        let header: [u8; HEADER_SIZE] = R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..]
            .try_into()
            .expect(
                "Unexpected compile-time header address size. This is a bug in the chosen Codec or embedded-registers.",
            );
        let register = *register.as_ref();
        let data = bytemuck::bytes_of(&register);

        let mut array = Vec::<u8, 64>::new();
        array
            .extend_from_slice(&header)
            .expect("Register too large for CrcCodec implementation. Raise an issue in embedded_registers.");

        for x in data.chunks(CHUNK_SIZE) {
            array
                .extend_from_slice(x)
                .expect("Register too large for CrcCodec implementation. Raise an issue in embedded_registers.");
            array
                .push(crc.checksum(x))
                .expect("Register too large for CrcCodec implementation. Raise an issue in embedded_registers.");
        }

        Ok(bound_bus.interface.write(bound_bus.address, &array).await?)
    }
}
