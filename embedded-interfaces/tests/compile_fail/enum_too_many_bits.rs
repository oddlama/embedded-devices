use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    enum TooWide: u8{9} {
        _ A
    } // u8 only has 8 bits
}
fn main() {}
