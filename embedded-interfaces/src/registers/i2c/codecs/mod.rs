pub mod standard_codec;
pub mod unsupported_codec;

pub type OneByteRegAddrCodec = standard_codec::StandardCodec<1>;
pub type TwoByteRegAddrCodec = standard_codec::StandardCodec<2>;
