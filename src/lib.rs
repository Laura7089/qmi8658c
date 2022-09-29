//! Doc
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![deny(clippy::pedantic)]

#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace};
use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::i2c::blocking::I2c;

// TODO: more tests
#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    fn always_passes() {
        assert_eq!(1, 1);
    }
}
