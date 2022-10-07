use super::QMI8658C;
#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace};
use embedded_hal::i2c::blocking::I2c;

// TODO: TIMESTAMP register

/// Configure the I2C address select pin
pub mod sa0 {
    /// I2C address select pin
    pub trait SA0 {
        /// I2C address indicated by the pin state
        const ADDR: u8;
    }
    /// SA0 pin held high
    #[derive(Copy, Clone, Debug)]
    pub struct SA0High;
    /// SA0 pin held low
    #[derive(Copy, Clone, Debug)]
    pub struct SA0Low;
    impl SA0 for SA0Low {
        const ADDR: u8 = 0x6b >> 1;
    }
    impl SA0 for SA0High {
        const ADDR: u8 = 0x6a >> 1;
    }
}

/// 8-bit numeric registers
#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum Register8 {
    WHO_AM_I = 0x00,
    REVISION_ID = 0x01,
    CTRL9 = 0x0a,
    FIFO_WTM_TH = 0x13,
    FIFO_SMPL_CNT = 0x15,
    FIFO_DATA = 0x17,
    RESET = 0x60,
}

/// 16-bit numeric registers
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum Register16 {
    CAL1 = 0x0b,
    CAL2 = 0x0d,
    CAL3 = 0x0f,
    CAL4 = 0x11,
    TEMP = 0x33,
    AX = 0x35,
    AY = 0x37,
    AZ = 0x39,
    GX = 0x3b,
    GY = 0x3d,
    GZ = 0x40,
    dQW = 0x49,
    dQX = 0x4b,
    dQY = 0x4d,
    dQZ = 0x4f,
    dVX = 0x51,
    dVY = 0x53,
    dVZ = 0x55,
}

pub(crate) trait Registers<I: I2c> {
    const ADDR: u8;

    fn i2c(&mut self) -> &mut I;

    fn read_raw<const N: usize>(&mut self, regaddr: u8) -> Result<[u8; N], I::Error> {
        let mut val = [0; N];
        self.i2c().write_read(Self::ADDR, &[regaddr], &mut val)?;
        Ok(val)
    }

    fn write_raw<const N: usize>(&mut self, regaddr: u8, val: [u8; N]) -> Result<(), I::Error> {
        let mut to_write = [0; N];
        to_write[0] = regaddr;
        to_write[1..].copy_from_slice(&val);

        self.i2c().write(Self::ADDR, &to_write)
    }

    fn read_reg8(&mut self, reg: Register8) -> Result<i8, I::Error> {
        let [raw] = self.read_raw(reg as u8)?;
        Ok(bytemuck::cast(raw))
    }

    fn read_reg16u(&mut self, reg: Register16) -> Result<u16, I::Error> {
        let lsb_addr = reg as u8;
        let mut raw: [u8; 2] = self.read_raw(lsb_addr)?;
        raw.reverse();
        Ok(bytemuck::cast(raw))
    }

    fn read_reg16s(&mut self, reg: Register16) -> Result<i16, I::Error> {
        let lsb_addr = reg as u8;
        let mut raw: [u8; 2] = self.read_raw(lsb_addr)?;
        raw.reverse();
        Ok(bytemuck::cast(raw))
    }

    fn write_reg8(&mut self, reg: Register8, val: i8) -> Result<(), I::Error> {
        self.write_raw::<1>(reg as u8, bytemuck::cast(val))
    }

    fn write_reg16u(&mut self, reg: Register16, val: u16) -> Result<(), I::Error> {
        let lsb_addr = reg as u8;
        let raw = bytemuck::bytes_of(&val);

        self.write_raw(lsb_addr, [raw[1], raw[0]])
    }

    fn write_reg16s(&mut self, reg: Register16, val: i16) -> Result<(), I::Error> {
        let lsb_addr = reg as u8;
        let raw = bytemuck::bytes_of(&val);

        self.write_raw(lsb_addr, [raw[1], raw[0]])
    }
}

impl<I, S, A, G, E> Registers<I> for QMI8658C<I, A, G, S>
where
    I: I2c<Error = E>,
    S: sa0::SA0,
    A: crate::modes::AccelStatus,
    G: crate::modes::GyroStatus,
{
    const ADDR: u8 = S::ADDR;
    fn i2c(&mut self) -> &mut I {
        &mut self.i2c
    }
}
