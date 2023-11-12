/// This defines a constructor to create a new simple device
/// from an i2c bus and address. The provided address type
/// must be convertible to u8.
macro_rules! i2c {
    ($device:ident, $address_type:ty, init_required) => {
        crate::simple_device::i2c!($device, $address_type, "Before using this device, you must call the [`Self::init`] method which initializes the sensor and ensures that it is working correctly.");
    };
    ($device:ident, $address_type:ty, init_wanted) => {
        crate::simple_device::i2c!($device, $address_type, "Before using this device, you should call the [`Self::init`] method which ensures that the sensor is working correctly.");
    };
    ($device:ident, $address_type:ty) => {
        crate::simple_device::i2c!($device, $address_type, "");
    };
    ($device:ident, $address_type:ty, $ctor_doc:expr) => {
        #[maybe_async_cfg::maybe(
            idents(hal(sync = "embedded_hal", async = "embedded_hal_async")),
            sync(not(feature = "async")),
            async(feature = "async"),
            keep_self
        )]
        impl<I> $device<embedded_registers::i2c::I2cDevice<I>>
        where
            I: hal::i2c::I2c + hal::i2c::ErrorType,
        {
            #[doc = "Initializes a new device with the given address on the specified bus."]
            #[doc = "This consumes the I2C bus `I`."]
            #[doc = ""]
            #[doc = $ctor_doc]
            #[inline]
            pub fn new_i2c(interface: I, address: $address_type) -> Self {
                Self {
                    interface: embedded_registers::i2c::I2cDevice {
                        interface,
                        address: address.into(),
                    },
                }
            }
        }
    };
}

pub(crate) use i2c;
