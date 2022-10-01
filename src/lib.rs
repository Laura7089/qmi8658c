#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::pedantic)]

mod flags;
mod registers;

pub use flags::config;

use core::marker::PhantomData;

use registers::sa0;

#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace};
use embedded_hal::i2c::blocking::I2c;

pub struct QMI8658C<I, S>
where
    I: I2c,
    S: sa0::SA0,
{
    i2c: I,
    _data: PhantomData<S>,
}

impl<I, S, E> QMI8658C<I, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
{
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            _data: PhantomData,
        }
    }
}

// TODO: more tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn always_passes() {
        assert_eq!(1, 1);
    }
}
