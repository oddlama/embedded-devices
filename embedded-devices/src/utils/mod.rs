pub mod callendar_van_dusen;

#[allow(unused)]
macro_rules! reexport_structs {
    ($common_path:path, { $($struct:ident),* $(,)? }) => {
        paste::paste! {
            $(
                pub use $common_path::$struct;
                pub use $common_path::[<$struct Unpacked>];
            )*
        }
    };
}
#[allow(unused)]
pub(crate) use reexport_structs;

#[allow(unused)]
macro_rules! from_bus_error {
    ($error:ident) => {
        impl<BusError> From<embedded_interfaces::TransportError<(), BusError>> for $error<BusError> {
            fn from(value: embedded_interfaces::TransportError<(), BusError>) -> Self {
                match value {
                    embedded_interfaces::TransportError::Codec(_) => {
                        panic!("BUG: This shouldn't have happened. Please report this to the issue tracker of embedded_devices")
                    }
                    embedded_interfaces::TransportError::Bus(x) => Self::Bus(x),
                }
            }
        }
    };
}
#[allow(unused)]
pub(crate) use from_bus_error;
