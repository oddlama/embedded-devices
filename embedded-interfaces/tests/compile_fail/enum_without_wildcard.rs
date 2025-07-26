use embedded_interfaces::codegen::interface_objects;

interface_objects! {
    enum Bad: u8{2} {
        0 A,
        1 B,
        // 2 and 3 are uncovered
    }
}

fn main() {}
