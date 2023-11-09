/// This defines a simple device with the given name.
/// Simple here means that the device can be instanciated without
/// requiring additional generics or constructor arguments except
/// for the interface.
macro_rules! device {
    ($device:ident) => {
        #[doc = concat!("A device of type `", stringify!($device), "` on the specified bus `I`.")]
        pub struct $device<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            #[doc = "The interface to communicate with the device"]
            interface: I,
        }

        impl<I> core::ops::Deref for $device<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            type Target = I;

            fn deref(&self) -> &Self::Target {
                &self.interface
            }
        }

        impl<I> core::ops::DerefMut for $device<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.interface
            }
        }
    };
}

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

/// This adds accessor functions for the given register.
macro_rules! add_register {
    ($($device_path:ident)::*, $register:ident, rw) => {
        crate::simple_device::add_register!($($device_path)::*, $register, r);
        crate::simple_device::add_register!($($device_path)::*, $register, w);
    };
    ($($device_path:ident)::*, $register:ident, r) => {
        impl<I> $($device_path)::*<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            paste::item! {
                pub async fn [<read_ $register:snake>](&mut self) -> Result<$register, I::Error> {
                    self.interface.read_register::<$register>().await
                }
            }
        }
    };
    ($($device_path:ident)::*, $register:ident, w) => {
        impl<I> $($device_path)::*<I>
        where
            I: embedded_registers::RegisterInterface,
        {
            paste::item! {
                pub async fn [<write_ $register:snake>](&mut self, register: &$register) -> Result<(), I::Error> {
                    self.interface.write_register::<$register>(register).await
                }
            }
        }
    };
}

pub(crate) use add_register;
pub(crate) use device;
pub(crate) use i2c;
