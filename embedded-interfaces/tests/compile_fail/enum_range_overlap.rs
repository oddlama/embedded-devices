use embedded_interfaces::codegen::interface_objects;
interface_objects! {
    enum Overlap: u8{4} {
        0..=5 Range1,
        4..=8 Range2, // 4-5 overlap
        _ Wild,
    }
}
fn main() {}
