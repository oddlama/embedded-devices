use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    enum BadCap: u16{4} {
        1..=5 Range(i16), // captures i16 but underlying is u16
        _ Wild,
    }
}
fn main() {}
