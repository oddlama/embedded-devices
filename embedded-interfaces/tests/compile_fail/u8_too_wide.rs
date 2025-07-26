use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    struct Bad(size = 2) {
        f1: u8{10}, // u8 cannot be 10 bits wide
    }
}
fn main() {}
