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
