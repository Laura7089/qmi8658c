#![allow(non_camel_case_types)]

use super::registers::Registers;
use bitflags::bitflags;
#[cfg(feature = "defmt")]
use defmt::trace;
use embedded_hal::i2c::blocking::I2c;

pub mod config {
    pub struct ConvertibleBool(bool);
    impl From<bool> for ConvertibleBool {
        fn from(b: bool) -> Self {
            Self(b)
        }
    }
    impl From<u8> for ConvertibleBool {
        fn from(int: u8) -> Self {
            Self(int > 0)
        }
    }
    impl From<ConvertibleBool> for u8 {
        fn from(b: ConvertibleBool) -> Self {
            b.0 as Self
        }
    }

    macro_rules! u8_convert {
        (
            $(
                $(#[$o:meta])*
                $v:vis enum $name:ident {
                    $( $field:ident = $val:literal, )*
                }
            )*
        ) => {
            $(
            $(#[$o])*
            $v enum $name {
                $( $field = $val, )*
            }
            impl From<u8> for $name {
                fn from(input: u8) -> Self {
                    match input {
                        $( $val => Self::$field, )*
                        _ => unreachable!(),
                    }
                }
            }

            impl From<$name> for u8 {
                fn from(input: $name) -> Self {
                    input as Self
                }
            }
            )*
        };
    }

    u8_convert! {
    /// Accelerometer Output Data Rate (ODR)
    // TODO
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AODR {}

    /// Accelerometer Full-scale
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AFS {
        AFS2G = 0b000,
        AFS4G = 0b001,
        AFS8G = 0b010,
        AFS16G = 0b011,
    }

    /// Gyroscope Full-scale
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum GFS {
        GFS16dps = 0b000,
        GFS32dps = 0b001,
        GFS64dps = 0b010,
        GFS128dps = 0b011,
        GFS256dps = 0b100,
        GFS512dps = 0b101,
        GFS1024dps = 0b110,
        GFS2048dps = 0b111,
    }

    /// Gyroscope Output Data Rate (ODR), in Hz
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum GODR {
        GODR7520 = 0b0000,
        GODR3760 = 0b0001,
        GODR1880 = 0b0010,
        GODR940 = 0b0011,
        GODR470 = 0b0100,
        GODR235 = 0b0101,
        GODR117_5 = 0b0110,
        GODR58_75 = 0b0111,
        GODR29_375 = 0b1000,
        // TODO: do we need the others?
    }

    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum GLPF_MODE {
        BW2_66 = 0b00,
        BW3_64 = 0b01,
        BW5_39 = 0b10,
        BW13_37 = 0b11,
    }
    }
}

#[allow(clippy::upper_case_acronyms)]
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum FlagRegister {
    CTRL1 = 0x02,
    CTRL2 = 0x03,
    CTRL7 = 0x08,
    CTRL8 = 0x09,
    STATUSINT = 0x2d,
    STATUS0 = 0x2e,
    STATUS1 = 0x2f,
    AE_REG1 = 0x57,
    AE_REG2 = 0x58,
}

fn mask(start: u8, end: u8) -> u8 {
    (start..end).into_iter().map(|p| 1 << p).sum()
}

macro_rules! hasbits {
    (
        $(
        $(#[$outer: meta])*
        $v:vis struct $name:ident {
            $($field:ident : $ftype:ty => $bs:literal : $be:literal),*
            $(,)?
        }
        )+
    ) => {
        $(
        $(#[$outer])*
        $v struct $name { $($field: $ftype,)* }

        impl $name {
            pub(crate) fn bits(self) -> u8 {
                let parts: &[u8] = &[
                    $( <$ftype as Into<u8>>::into(self.$field) << $bs,)+
                ];
                parts.into_iter().sum()
            }

            pub(crate) fn from_bits_truncate(__i: u8) -> Self {
                Self {
                    $( $field: ((__i & mask($bs, $be)) >> $bs).into(), )*
                }
            }
        }
        )*
    };
}

hasbits! {
    pub(crate) struct CTRL2 {
        ast: config::ConvertibleBool => 7:7,
        afs: config::AFS => 4:6,
        aodr: config::AODR => 0:3,
    }

    pub(crate) struct CTRL3 {
        gst: config::ConvertibleBool => 7:7,
        gfs: config::GFS => 4:6,
        godr: config::GODR => 0:3,
    }
}

bitflags! {
    pub(crate) struct CTRL1: u8 {
        const SIM = 1 << 7;
        const ADDR_AI = 1 << 6;
        const BE = 1 << 5;
        const SENSOR_DISABLE = 1;
    }

    pub(crate) struct CTRL7: u8 {
        const SYNC_SMPL = 1 << 7;
        const SYS_HS = 1 << 6;
        const GSN = 1 << 4;
        const SEN = 1 << 3;
        const GEN = 1 << 1;
        const AEN = 1;
    }

    pub(crate) struct CTRL8: u8 {
        const CTRL9_HANDSHAKE_TYPE = 1 << 7;
        const INT_MOTION = 1 << 6;
        const PED_ENABLE = 1 << 4;
        const SIG_MOTION_ENABLE = 1 << 3;
        const NO_MOTION_ENABLE = 1 << 2;
        const ANY_MOTION_ENABLE = 1 << 1;
        const TAP_ENABLE = 1;
    }

    pub(crate) struct STATUSINT: u8 {
        const CTRL9_CMD_DONE = 1 << 7;
        const LOCKED = 1 << 1;
        const AVAIL = 1;
    }

    pub(crate) struct STATUS0: u8 {
        const SDA = 1 << 3;
        const GDA = 1 << 1;
        const ADA = 1;
    }

    pub(crate) struct STATUS1: u8 {
        const SIG_MOTION = 1 << 7;
        const NO_MOTION = 1 << 6;
        const ANY_MOTION = 1 << 5;
        const PEDOMETER = 1 << 4;
        const TAP = 1 << 1;
    }

    pub(crate) struct AE_REG1: u8 {
        const GYRO_BIAS_ACK = 1 << 6;
        const WZ_CLIP = 1 << 5;
        const WY_CLIP = 1 << 4;
        const WX_CLIP = 1 << 3;
        const AZ_CLIP = 1 << 2;
        const AY_CLIP = 1 << 1;
        const AX_CLIP = 1;
    }

    pub(crate) struct AE_REG2: u8 {
        const DVZ_OF = 1 << 2;
        const DVY_OF = 1 << 1;
        const DVX_OF = 1;
    }
}

macro_rules! getter {
    ($funcname:ident -> $reg:ident) => {
        fn $funcname(&mut self) -> Result<$reg, I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Reading flags from {}", FlagRegister::$reg);
            Ok(<$reg>::from_bits_truncate(
                self.read_flags(FlagRegister::$reg)?,
            ))
        }
    };
}
macro_rules! setter {
    ($funcname:ident -> $reg:ident) => {
        fn $funcname(&mut self, val: $reg) -> Result<(), I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Writing flags to {}", FlagRegister::$reg);
            self.write_flags(FlagRegister::$reg, val.bits())
        }
    };
}

// TODO: merge this with `Registers`?
pub(crate) trait Flags<I: I2c> {
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error>;
    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error>;

    getter!(get_ctrl1 -> CTRL1);
    getter!(get_ctrl2 -> CTRL2);
    getter!(get_ctrl7 -> CTRL7);
    getter!(get_ctrl8 -> CTRL8);
    getter!(get_statusint -> STATUSINT);
    getter!(get_status0 -> STATUS0);
    getter!(get_status1 -> STATUS1);
    getter!(get_ae_reg1 -> AE_REG1);
    getter!(get_ae_reg2 -> AE_REG2);

    setter!(set_ctrl1 -> CTRL1);
    setter!(set_ctrl2 -> CTRL2);
    setter!(set_ctrl7 -> CTRL7);
    setter!(set_ctrl8 -> CTRL8);
}

impl<I: I2c, T> Flags<I> for T
where
    T: Registers<I>,
{
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error> {
        self.read_raw(reg as u8)
    }

    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error> {
        self.write_raw(reg as u8, val)
    }
}
