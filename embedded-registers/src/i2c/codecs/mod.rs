mod crc_codec;
mod no_codec;
mod simple_codec;

pub use crc_codec::Crc8Algorithm;
pub use crc_codec::Crc8Codec;

pub use no_codec::NoCodec;
pub use simple_codec::SimpleCodec;

pub type OneByteRegAddrCodec = SimpleCodec<1>;
pub type TwoByteRegAddrCodec = SimpleCodec<2>;
