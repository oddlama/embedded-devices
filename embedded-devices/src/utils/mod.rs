pub mod callendar_van_dusen;

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
pub(crate) use from_bus_error;
