pub mod crc_codec;
pub mod no_codec;
pub mod simple_codec;

pub type OneByteRegAddrCodec = simple_codec::SimpleCodec<1>;
pub type TwoByteRegAddrCodec = simple_codec::SimpleCodec<2>;
