use core::marker::PhantomData;

use crate::{ReadableRegister, WritableRegister};
use arrayvec::ArrayVec;
use bytemuck::Zeroable;
use crc::{Algorithm, CRC_8_NRSC_5};

/// This codec implements an I2C codec utilizing crc checksums for data
/// The main variables are:
/// - the size of register addresses in bytes.
/// - the crc algorithm in use
/// - set chunk size that get checksummed
///
/// This implements the codec only for crc outputs of single byte size.
/// If your device has larger crc sums, you cannot use this implementation as is.
///
/// This codec has no information over register sizes or contents.
/// It will always send one header and then receive/send the associated register data,
/// interspersed with crc sums each CHUNK_SIZE bytes.
/// This makes the codec unsuited for cases where the device has
/// registers that require interspersing the crc between fields of differing sizes.
///
/// The following generic parameters are available:
///
/// | Parameter | Type | Description |
/// |---|---|---|
/// | `HEADER_SIZE` | `usize` | The size of the command header (register address) in bytes |
/// | `CHUNK_SIZE` | `usize` | The size of a chunk that has a singular crc sum attached in bytes |
/// | `C` | `Crc8Algorithm` | A static reference to the crc algorithm to be used |
/// Example implemenation for a basic CRC Algorithm:
/// #[derive(Default)]
/// struct MyCrc {}
///
/// impl Crc8Algorithm for MyCrc {
///     fn new() -> &'static Algorithm<u8> {
///         const CUSTOM_ALG: crc::Algorithm<u8> = CRC_8_NRSC_5;
///         &CUSTOM_ALG
///     }
/// }
#[derive(Default)]
pub struct Crc8Codec<const HEADER_SIZE: usize, const CHUNK_SIZE: usize, C: Crc8Algorithm> {
    _algo: PhantomData<C>,
}

pub trait Crc8Algorithm: Default {
    /// Return reference to global static CRC Algorithm
    fn new() -> &'static Algorithm<u8>;
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<const HEADER_SIZE: usize, const CHUNK_SIZE: usize, C: Crc8Algorithm + 'static> crate::i2c::Codec
    for Crc8Codec<HEADER_SIZE, CHUNK_SIZE, C>
{
    #[inline]
    async fn read_register<R, I, A>(bound_bus: &mut crate::i2c::I2cBoundBus<I, A>) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let crc = crc::Crc::<u8>::new(C::new());
        let header = &R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..];

        let mut array = ArrayVec::<_, 64>::new();
        unsafe {
            array.set_len(R::REGISTER_SIZE + R::REGISTER_SIZE / CHUNK_SIZE);
        }

        bound_bus
            .interface
            .write_read(bound_bus.address, header, &mut array)
            .await?;

        let mut register = R::zeroed();
        let data = bytemuck::bytes_of_mut(&mut register);
        for (i, x) in array.chunks(CHUNK_SIZE + 1).enumerate() {
            let value = &x[0..CHUNK_SIZE];
            data[i..i + CHUNK_SIZE].copy_from_slice(value);
            let crc_val = crc.checksum(value);
            let crc_real = x[CHUNK_SIZE];
            if crc_real != crc_val {
                panic!("crc failed")
            }
        }

        Ok(register)
    }

    #[inline]
    async fn write_register<R, I, A>(
        bound_bus: &mut crate::i2c::I2cBoundBus<I, A>,
        register: impl AsRef<R>,
    ) -> Result<(), I::Error>
    where
        R: WritableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        #[repr(C, packed(1))]
        #[derive(Copy, Clone, bytemuck::Pod, Zeroable)]
        struct Buffer<const HEADER_SIZE: usize, R> {
            header: [u8; HEADER_SIZE],
            register: R,
        }
        let crc = crc::Crc::<u8>::new(C::new());

        let mut buffer = Buffer::<{ HEADER_SIZE }, R> {
            header: R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..]
                .try_into()
                .expect("Unexpected compile-time header address size. This is a bug in the chosen Codec or embedded-registers."),
            register: *register.as_ref(),
        };
        let data = bytemuck::bytes_of_mut(&mut buffer);

        let mut array = ArrayVec::<_, 64>::new();
        for x in data.chunks(CHUNK_SIZE) {
            array.try_extend_from_slice(x).unwrap();
            let crc_val = crc.checksum(x);
            array.push(crc_val);
        }

        bound_bus.interface.write(bound_bus.address, &array).await
    }
}
