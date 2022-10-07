#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::pedantic)]

// TODO: interrupts
// TODO: FIFO implemented
// TODO: Wake on Motion
// TODO: type-model AttitudeEngine mode
// TODO: support magnetometer? note: not connected on bob so cannot test at all
// TODO: `TryInto` conversions for the different enable states?

mod flags;
mod registers;

pub use flags::config;
use flags::Flags;
pub use registers::sa0;

use core::marker::PhantomData;

use registers::{Register16, Register8, Registers};

#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace, write, Formatter};
use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::i2c::blocking::I2c;
#[cfg(feature = "micromath")]
use micromath::vector::{I16x3, Vector3d};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(non_camel_case_types)]
pub(crate) enum CTRL9Command {
    /// No operation
    NOP = 0x00,
    /// Copies bias_gx,y,z from CAL registers to FIFO and set GYROBIAS_PEND bit
    GYRO_BIAS = 0x01,
    /// SDI MOD (Motion on Demand), request to read SDI data
    REQ_SDI = 0x03,
    /// Reset FIFO from Host
    RST_FIFO = 0x04,
    /// Get FIFO data from Device
    REQ_FIFO = 0x05,
    /// Set up and enable Wake on Motion (WoM)
    WRITE_WOM_SETTING = 0x08,
    /// Change accelerometer offset
    ACCEL_HOST_DELTA_OFFSET = 0x09,
    /// Change gyroscope offset
    GYRO_HOST_DELTA_OFFSET = 0x0a,
    /// Read USID_Bytes and FW_Version bytes
    COPY_USID = 0x10,
    /// Configures IO pull-ups
    SET_RPU = 0x11,
    /// Internal AHB clock gating switch
    AHB_CLOCK_GATING = 0x12,
    /// On-Demand Calibration on gyroscope
    ON_DEMAND_CALIBRATION = 0xa2,
}

/// An acceleration measurement, in g
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Acceleration {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
#[cfg(feature = "defmt")]
impl defmt::Format for Acceleration {
    fn format(&self, fmt: Formatter) {
        write!(fmt, "Accel(x: {}g, y: {}g, z: {}g)", self.x, self.y, self.z);
    }
}

/// An angular rate measurement, in degrees/second
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AngularRate {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}
#[cfg(feature = "defmt")]
impl defmt::Format for AngularRate {
    fn format(&self, fmt: Formatter) {
        write!(
            fmt,
            "Angular(x: {}°/s, y: {}°/s, z: {}°/s)",
            self.x, self.y, self.z
        );
    }
}

/// A temperature measurement, in °C (degrees celsius)
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Temperature(pub i16);
#[cfg(feature = "defmt")]
impl defmt::Format for Temperature {
    fn format(&self, fmt: Formatter) {
        write!(fmt, "{}°C", self.0);
    }
}

/// Sensor mode typestate for the [`QMI8658C`]
pub mod modes {
    /// Status of the onboard accelerometer
    pub trait AccelStatus {}
    /// Accelerometer active
    pub struct AccelActive;
    /// Accelerometer deactivated
    pub struct AccelOff;
    impl AccelStatus for AccelActive {}
    impl AccelStatus for AccelOff {}

    /// Status of the onboard gyroscope
    pub trait GyroStatus {}
    /// Gyroscope active
    pub struct GyroActive;
    /// Gyroscope deactivated
    pub struct GyroOff;
    impl GyroStatus for GyroActive {}
    impl GyroStatus for GyroOff {}

    /// A [`crate::QMI8658C`] with no sensors active
    ///
    /// This is the default state, returned by both [`crate::QMI8658C::new`] and [`crate::QMI8658C::reset`].
    pub type QMI8658COff<I, S> = super::QMI8658C<I, AccelOff, GyroOff, S>;
    pub type QMI8658CActive<I, S> = super::QMI8658C<I, AccelActive, GyroActive, S>;
}

/// A QST Corp QMI8658C Inertial Measurement Unit
pub struct QMI8658C<I, A, G, S>
where
    I: I2c,
    S: sa0::SA0,
    A: modes::AccelStatus,
    G: modes::GyroStatus,
{
    i2c: I,
    cmd_running: bool,
    _data: PhantomData<(S, A, G)>,
}

impl<I, S, E> modes::QMI8658COff<I, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
{
    /// Take control of a new QMI8658c IMU
    pub fn new(i2c: I, delay: &mut impl DelayUs) -> Result<Self, E> {
        let mut to_ret = Self {
            i2c,
            cmd_running: false,
            _data: PhantomData,
        };
        to_ret = to_ret.reset(delay)?;

        let mut new_ctrl1 = to_ret.get_ctrl1()?;
        new_ctrl1.insert(flags::CTRL1::ADDR_AI);
        to_ret.set_ctrl1(new_ctrl1)?;

        Ok(to_ret)
    }

    /// Convenience function for activating both sensors
    ///
    /// Activates accelerometer first.
    pub fn activate_both(self) -> Result<modes::QMI8658CActive<I, S>, E> {
        Ok(self.activate_accel()?.activate_gyro()?)
    }
}

impl<I, S, E, A, G> QMI8658C<I, A, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    A: modes::AccelStatus,
    G: modes::GyroStatus,
{
    /// Perform a software reset of the device
    pub fn reset(self, d: &mut impl DelayUs) -> Result<modes::QMI8658COff<I, S>, E> {
        let mut new = modes::QMI8658COff {
            i2c: self.i2c,
            cmd_running: false,
            _data: PhantomData,
        };
        new.write_reg8(Register8::RESET, bytemuck::cast(0xb0))?;
        d.delay_ms(150).unwrap(); // TODO: messy unwrap, should return instead
        Ok(new)
    }

    /// Make a temperature measurement
    pub fn read_temp(&mut self) -> Result<Temperature, E> {
        Ok(Temperature(self.read_reg16s(Register16::TEMP)?))
    }

    fn ctrl9_command(&mut self, cmd: CTRL9Command) -> nb::Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Sending {} to CTRL9 register", cmd);
        if !self.cmd_running {
            self.cmd_running = true;
            self.write_raw(Register8::CTRL9 as u8, [cmd as u8])?;
            Err(nb::Error::WouldBlock)
        } else if self
            .get_statusint()?
            .contains(flags::STATUSINT::CTRL9_CMD_DONE)
        {
            self.cmd_running = false;
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

// TODO: deactivation methods?
impl<I, S, E, G> QMI8658C<I, modes::AccelOff, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    G: modes::GyroStatus,
{
    /// Turn on the accelerometer and update type-state
    pub fn activate_accel(self) -> Result<QMI8658C<I, modes::AccelActive, G, S>, E> {
        // all needs replacing
        let new_flags = flags::CTRL2 {
            aodr: todo!(),
            ..self.get_ctrl2()?
        };
        self.set_ctrl2(new_flags)?;

        todo!()
    }

    /// Turn off the accelerometer and update type-state
    pub fn deactivate_accel(self) -> Result<QMI8658C<I, modes::AccelOff, G, S>, E> {
        // all needs replacing
        let new_flags = flags::CTRL2 {
            aodr: todo!(),
            ..self.get_ctrl2()?
        };
        self.set_ctrl2(new_flags)?;

        todo!()
    }
}

impl<I, S, E, A> QMI8658C<I, A, modes::GyroOff, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    A: modes::AccelStatus,
{
    /// Turn on the gyroscope and update type-state
    pub fn activate_gyro(self) -> Result<QMI8658C<I, A, modes::GyroActive, S>, E> {
        // all needs replacing
        let new_flags = flags::CTRL3 {
            godr: todo!(),
            ..self.get_ctrl3()?
        };
        self.set_ctrl3(new_flags)?;

        todo!()
    }

    /// Turn off the gyroscope and update type-state
    pub fn deactivate_gyro(self) -> Result<QMI8658C<I, A, modes::GyroOff, S>, E> {
        // all needs replacing
        let new_flags = flags::CTRL3 {
            godr: todo!(),
            ..self.get_ctrl3()?
        };
        self.set_ctrl3(new_flags)?;

        todo!()
    }
}

impl<I, S, E, G> QMI8658C<I, modes::AccelActive, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    G: modes::GyroStatus,
{
    /// Make an acceleration measurement
    pub fn read_accel(&mut self) -> Result<Acceleration, E> {
        Ok(Acceleration {
            x: self.read_reg16s(Register16::AX)?,
            y: self.read_reg16s(Register16::AY)?,
            z: self.read_reg16s(Register16::AZ)?,
        })
    }

    /// Make an acceleration measurement with a `micromath` vector
    #[cfg(feature = "micromath")]
    pub fn read_accel_mm(&mut self) -> Result<I16x3, E> {
        Ok(Vector3d {
            x: self.read_reg16s(Register16::AX)?,
            y: self.read_reg16s(Register16::AY)?,
            z: self.read_reg16s(Register16::AZ)?,
        }
        .into())
    }

    // TODO: delay?
    pub fn accel_self_test(&mut self) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Running acceleromter self-test");

        self.set_ctrl7(flags::CTRL7::empty())?;
        let new_ctrl2 = flags::CTRL2 {
            ast: true.into(),
            ..self.get_ctrl2()?
        };
        self.set_ctrl2(new_ctrl2)?;
        // TODO: wait for INT2 high
        let new_ctrl2 = flags::CTRL2 {
            ast: false.into(),
            ..self.get_ctrl2()?
        };
        self.set_ctrl2(new_ctrl2)?;
        // TODO: wait for INT2 low

        for reg in [Register16::dVX, Register16::dVY, Register16::dVZ] {
            let val = self.read_reg16s(reg)?;
            // TODO: correct scaling?
            assert!(
                val / 5 > 200,
                "accelerometer self-test failed, reg {:?}",
                reg
            );
        }

        #[cfg(feature = "defmt")]
        info!("Accelerometer self test passed");
        Ok(())
    }
}

#[cfg(feature = "accelerometer")]
use accelerometer::{
    vector::{F32x3, I16x3 as I16x3a},
    Accelerometer, Error, RawAccelerometer,
};
#[cfg(feature = "accelerometer")]
impl<I, S, E, G> RawAccelerometer<I16x3a> for QMI8658C<I, modes::AccelActive, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    G: modes::GyroStatus,
    E: core::fmt::Debug,
{
    type Error = E;

    fn accel_raw(&mut self) -> Result<I16x3a, Error<E>> {
        let reading = self.read_accel()?;
        Ok(I16x3a {
            x: reading.x,
            y: reading.y,
            z: reading.z,
        })
    }
}

#[cfg(feature = "accelerometer")]
impl<I, S, E, G> Accelerometer for QMI8658C<I, modes::AccelActive, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    G: modes::GyroStatus,
    E: core::fmt::Debug,
{
    type Error = E;

    fn accel_norm(&mut self) -> Result<F32x3, Error<E>> {
        let reading = self.read_accel()?;
        Ok(F32x3 {
            x: reading.x.into(),
            y: reading.y.into(),
            z: reading.z.into(),
        })
    }

    fn sample_rate(&mut self) -> Result<f32, Error<E>> {
        todo!()
    }
}

impl<I, S, E, A> QMI8658C<I, A, modes::GyroActive, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    A: modes::AccelStatus,
{
    /// Make a gyroscope measurement
    pub fn read_gyro(&mut self) -> Result<AngularRate, E> {
        Ok(AngularRate {
            x: self.read_reg16s(Register16::GX)?,
            y: self.read_reg16s(Register16::GY)?,
            z: self.read_reg16s(Register16::GZ)?,
        })
    }

    /// Make a gyroscope measurement with a `micromath` vector
    #[cfg(feature = "micromath")]
    pub fn read_gyro_mm(&mut self) -> Result<I16x3, E> {
        Ok(Vector3d {
            x: self.read_reg16s(Register16::GX)?,
            y: self.read_reg16s(Register16::GY)?,
            z: self.read_reg16s(Register16::GZ)?,
        }
        .into())
    }

    // TODO: delay?
    pub fn gyro_self_test(&mut self) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Running gyroscope self-test");

        self.set_ctrl7(flags::CTRL7::empty())?;
        let new_ctrl3 = flags::CTRL3 {
            gst: true.into(),
            ..self.get_ctrl3()?
        };
        self.set_ctrl3(new_ctrl3)?;
        // TODO: wait for INT2 high
        let new_ctrl3 = flags::CTRL3 {
            gst: false.into(),
            ..self.get_ctrl3()?
        };
        self.set_ctrl3(new_ctrl3)?;
        // TODO: wait for INT2 low

        for reg in [Register16::dVX, Register16::dVY, Register16::dVZ] {
            let val = self.read_reg16s(reg)?;
            assert!(val / 5 > 200, "gyroscope self-test failed, reg {:?}", reg);
        }

        #[cfg(feature = "defmt")]
        info!("Gyroscope self test passed");
        Ok(())
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
