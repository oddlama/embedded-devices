pub mod callendar_van_dusen;

#[allow(unused)]
macro_rules! reexport_registers {
    ($common_path:path, { $($register:ident),* $(,)? }) => {
        paste::paste! {
            $(
                pub use $common_path::$register;
                pub use $common_path::[<$register Bitfield>];
            )*
        }
    };
}
#[allow(unused)]
pub(crate) use reexport_registers;

#[allow(unused)]
macro_rules! from_bus_error {
    ($error:ident) => {
        impl<BusError> From<embedded_registers::RegisterError<(), BusError>> for $error<BusError> {
            fn from(value: embedded_registers::RegisterError<(), BusError>) -> Self {
                match value {
                    embedded_registers::RegisterError::Codec(_) => {
                        panic!("BUG: This shouldn't have happened. Please report this to the issue tracker of embedded_devices")
                    }
                    embedded_registers::RegisterError::Bus(x) => Self::Bus(x),
                }
            }
        }
    };
}
#[allow(unused)]
pub(crate) use from_bus_error;
