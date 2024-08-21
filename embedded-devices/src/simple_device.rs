/// This defines a constructor to create a new simple device
/// from an i2c bus and address. The provided address type
/// must be convertible to u8.
macro_rules! i2c {
    ($device:ident, $AddressType:ty, $AddressMode:ident, $DefaultCodec:ty, "init=required") => {
        crate::simple_device::i2c!($device, $AddressType, $AddressMode, $DefaultCodec, "Before using this device, you must call the [`Self::init`] method which initializes the device and ensures that it is working correctly.");
    };
    ($device:ident, $AddressType:ty, $AddressMode:ident, $DefaultCodec:ty, "init=wanted") => {
        crate::simple_device::i2c!($device, $AddressType, $AddressMode, $DefaultCodec, "Before using this device, you should call the [`Self::init`] method which ensures that the device is working correctly.");
    };
    ($device:ident, $AddressType:ty, $AddressMode:ident, $DefaultCodec:ty) => {
        crate::simple_device::i2c!($device, $AddressType, $AddressMode, $DefaultCodec, "");
    };
    ($device:ident, $AddressType:ty, $AddressMode:ident, $DefaultCodec:ty, $ctor_doc:expr) => {
        #[maybe_async_cfg::maybe(
            idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
            sync(not(feature = "async")),
            async(feature = "async"),
            keep_self
        )]
        impl<I> $device<embedded_registers::i2c::I2cDevice<I, hal::i2c::$AddressMode, $DefaultCodec>>
        where
            I: hal::i2c::I2c<hal::i2c::$AddressMode> + hal::i2c::ErrorType,
        {
            #[doc = "Initializes a new device with the given address on the specified bus."]
            #[doc = "This consumes the I2C bus `I`."]
            #[doc = ""]
            #[doc = $ctor_doc]
            #[inline]
            pub fn new_i2c(interface: I, address: $AddressType) -> Self {
                Self {
                    interface: embedded_registers::i2c::I2cDevice::new(
                        interface,
                        address.into(),
                        <$DefaultCodec>::default()
                    )
                }
            }
        }
    };
}

pub(crate) use i2c;
