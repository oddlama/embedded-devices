use core::marker::PhantomData;

use crate::{ReadableRegister, WritableRegister};
use bytemuck::Zeroable;
use crc::{Algorithm, CRC_8_NRSC_5};

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
pub struct Crc8Codec<const HEADER_SIZE: usize, C: Crc8Algorithm> {
    _algo: PhantomData<C>,
}

pub trait Crc8Algorithm: Default {
    fn new() -> &'static Algorithm<u8>;
    fn generate(bytes: &[u8]) -> u8 {
        let crc = crc::Crc::<u8>::new(Self::new());
        crc.checksum(bytes)
    }
}

#[derive(Default)]
struct MyCrc {}

impl Crc8Algorithm for MyCrc {
    fn new() -> &'static Algorithm<u8> {
        const CUSTOM_ALG: crc::Algorithm<u8> = CRC_8_NRSC_5;
        &CUSTOM_ALG
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), Codec, I2cBoundBus),
    sync(feature = "sync"),
    async(feature = "async"),
    keep_self
)]
impl<const HEADER_SIZE: usize, C: Crc8Algorithm + 'static> crate::i2c::Codec for Crc8Codec<HEADER_SIZE, C> {
    #[inline]
    async fn read_register<R, I, A>(bound_bus: &mut crate::i2c::I2cBoundBus<I, A>) -> Result<R, I::Error>
    where
        R: ReadableRegister,
        I: hal::i2c::I2c<A> + hal::i2c::ErrorType,
        A: hal::i2c::AddressMode + Copy,
    {
        let header = &R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..];

        #[repr(C, packed(1))]
        #[derive(Copy, Clone, bytemuck::Pod, Zeroable)]
        struct Buffer<R> {
            register: R,
            crc: u8,
        }
        let mut buffer = Buffer::<R>::zeroed();
        let data = bytemuck::bytes_of_mut(&mut buffer);

        bound_bus.interface.write_read(bound_bus.address, header, data).await?;
        let register = buffer.register;
        let crc = C::generate(register.data());
        if crc != buffer.crc {
            panic!("OhNo")
        } else {
            Ok(register)
        }
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
            crc: u8,
        }

        let crc = C::generate(register.as_ref().data());
        let mut buffer = Buffer::<{ HEADER_SIZE }, R> {
            header: R::ADDRESS.to_be_bytes()[core::mem::size_of_val(&R::ADDRESS) - HEADER_SIZE..]
                .try_into()
                .expect("Unexpected compile-time header address size. This is a bug in the chosen Codec or embedded-registers."),
            register: *register.as_ref(),
            crc,
        };

        let data = bytemuck::bytes_of_mut(&mut buffer);
        bound_bus.interface.write(bound_bus.address, data).await
    }
}
