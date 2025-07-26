use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    enum Bad: i8{3} {
        _ A
    } // only unsigned types are allowed
}
fn main() {}
