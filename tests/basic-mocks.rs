use embedded_hal_mock::delay::MockNoop;
use embedded_hal_mock::i2c::{Mock, Transaction as Tr};
use once_cell::sync::Lazy;
use rand::prelude::*;
use std::ops::Deref;

// We use CSBLow for all the tests since there's no point changing it in mocks
// type HP203B<M> = hp203b::HP203B<Mock, M, csb::CSBLow>;

#[test]
fn always_passes() {
    assert_eq!(1, 1);
}
