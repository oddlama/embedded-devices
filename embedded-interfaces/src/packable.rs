use core::mem::MaybeUninit;

use bitvec::{order::Msb0, slice::BitSlice, view::BitView};

/// A trait for any object that can be packed into and unpacked from a bit slice.
pub trait Packable {
    /// The minimum number of bits which this type occupies (inclusive)
    const MIN_BITS: usize;
    /// The maximum number of bits which this type occupies (inclusive)
    const MAX_BITS: usize;

    /// Packs this object into the given bit ranges of the dst bit slice. All ranges must be
    /// utilized fully. The total amount of bits `total_bits` reflects how many bits are covered by
    /// all ranges in total and is guaranteed be within MIN_BITS..=MAX_BITS.
    fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize);
    /// Unpacks this type from the given bit ranges of the src bit slice. All ranges must be
    /// utilized fully. The total amount of bits `total_bits` reflects how many bits are covered by
    /// all ranges in total and is guaranteed be within MIN_BITS..=MAX_BITS.
    fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize) -> Self;
}

pub fn bitcopy(src: &BitSlice<u8, Msb0>, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)]) {
    let mut dst_start = 0usize;
    for &(a, b) in ranges {
        let size = b - a;
        dst[dst_start..(dst_start + size)].copy_from_bitslice(&src[a..b]);
        dst_start += size;
    }
}

macro_rules! pack_unsigned {
    ($T:ty) => {
        impl Packable for $T {
            const MIN_BITS: usize = 0;
            const MAX_BITS: usize = <$T>::BITS as usize;

            #[inline]
            fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize) {
                let src_data = self.to_be_bytes();
                let src = src_data.view_bits::<Msb0>();
                let start = Self::MAX_BITS - total_bits;
                bitcopy(&src[start..], dst, ranges);
            }

            #[inline]
            fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize) -> Self {
                let mut dst_data = [0u8; size_of::<$T>()];
                let dst = dst_data.view_bits_mut::<Msb0>();
                let start = Self::MAX_BITS - total_bits;
                bitcopy(src, &mut dst[start..], ranges);
                <$T>::from_be_bytes(dst_data)
            }
        }
    };
}

macro_rules! pack_signed {
    ($T:ty) => {
        impl Packable for $T {
            const MIN_BITS: usize = 2; // 2 because sign bit needs to exist and only sign makes no sense
            const MAX_BITS: usize = <$T>::BITS as usize;

            #[inline]
            fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize) {
                let start = Self::MAX_BITS - total_bits;
                let sign_byte_index = start / 8;
                let sign_bit_in_byte_index = 7 - (start % 8);

                let mut src_data = self.to_be_bytes();
                src_data[sign_byte_index] |= (src_data[0] >> 7) << sign_bit_in_byte_index;
                let src = src_data.view_bits::<Msb0>();

                bitcopy(&src[start..], dst, ranges);
            }

            #[inline]
            fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], total_bits: usize) -> Self {
                let mut dst_data;
                let sign_bit = ranges[0].0;
                // Initialize with 1s if sign bit is set. This is basically sign extension.
                if src[sign_bit] {
                    dst_data = [0xffu8; size_of::<$T>()];
                } else {
                    dst_data = [0u8; size_of::<$T>()];
                }

                let dst = dst_data.view_bits_mut::<Msb0>();
                let start = Self::MAX_BITS - total_bits;
                bitcopy(src, &mut dst[start..], ranges);
                <$T>::from_be_bytes(dst_data)
            }
        }
    };
}

macro_rules! pack_float {
    ($T:ty) => {
        impl Packable for $T {
            const MIN_BITS: usize = size_of::<$T>() * 8;
            const MAX_BITS: usize = size_of::<$T>() * 8;

            #[inline]
            fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) {
                let src_data = self.to_be_bytes();
                let src = src_data.view_bits::<Msb0>();
                bitcopy(src, dst, ranges);
            }

            #[inline]
            fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) -> Self {
                let mut dst_data = [0u8; size_of::<$T>()];
                let dst = dst_data.view_bits_mut::<Msb0>();
                bitcopy(src, dst, ranges);
                <$T>::from_be_bytes(dst_data)
            }
        }
    };
}

impl Packable for bool {
    const MIN_BITS: usize = 1;
    const MAX_BITS: usize = 1;

    #[inline]
    fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) {
        dst.replace(ranges[0].0, *self);
    }

    #[inline]
    fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) -> Self {
        src[ranges[0].0]
    }
}

pack_unsigned!(u8);
pack_unsigned!(u16);
pack_unsigned!(u32);
pack_unsigned!(u64);
pack_unsigned!(u128);
pack_signed!(i8);
pack_signed!(i16);
pack_signed!(i32);
pack_signed!(i64);
pack_signed!(i128);
// pack_float!(f16);
pack_float!(f32);
pack_float!(f64);

impl<T: Packable, const N: usize> Packable for [T; N] {
    const MIN_BITS: usize = size_of::<T>() * N * 8;
    const MAX_BITS: usize = size_of::<T>() * N * 8;

    #[inline]
    fn pack(&self, dst: &mut BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) {
    }

    #[inline]
    fn unpack(src: &BitSlice<u8, Msb0>, ranges: &[(usize, usize)], _total_bits: usize) -> Self {
    }
}
