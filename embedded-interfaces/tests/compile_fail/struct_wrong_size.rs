use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    struct Bad(size = 1) {
        f1: u16{9},
    }
}
fn main() {}
