use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    enum DoubleWild: u8{2} {
        0 A,
        _ B,
        _ C, // duplicate wildcard
    }
}
fn main() {}
