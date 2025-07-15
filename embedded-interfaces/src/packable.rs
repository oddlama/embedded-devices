/// A trait for any object that can be packed into and unpacked from an unsigned.
pub trait UnsignedPackable: Sized {
    /// The unsigned base type that is used to store this type. Must be [`u8`], [`u16`], [`u32`],
    /// [`u64`] or [`u128`]. Must be able to hold at least [`Self::BITS`] bits.
    type Base: Copy;
    /// The amount of bits which this type occupies in the base unsigned type. The occupied bits
    /// must always be the least significant N bits.
    const BITS: usize;

    /// Convert the base unsigned value into an instance of [`Self`]. Only the least significant
    /// [`Self::BITS`] bits have meaning, all other bits are guaranteed to be zero. This function
    /// must succeed for any given value.
    fn from_unsigned(value: Self::Base) -> Self;
    /// Convert this object into the base unsigned type. Only the least significant
    /// [`Self::BITS`] bits need to be well defined.
    fn to_unsigned(&self) -> Self::Base;
}
