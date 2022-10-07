#![allow(non_camel_case_types)]

use super::registers::Registers;
use bitflags::bitflags;
#[cfg(feature = "defmt")]
use defmt::trace;
use embedded_hal::i2c::blocking::I2c;

pub mod config {
    /// A simple newtype around a [`bool`]
    ///
    /// Most easily used with [`Into<bool>`]:
    ///
    /// ```rust
    /// # use qmi8658c::config::ConfBool;
    /// let my_conf_bool: ConfBool = true.into();
    /// ```
    #[derive(Copy, Clone, Debug, Eq, PartialEq, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ConfBool(pub bool);
    impl From<bool> for ConfBool {
        fn from(b: bool) -> Self {
            Self(b)
        }
    }
    impl Into<bool> for ConfBool {
        fn into(self) -> bool {
            self.0
        }
    }
    impl From<u8> for ConfBool {
        fn from(int: u8) -> Self {
            Self(int > 0)
        }
    }
    impl From<ConfBool> for u8 {
        fn from(b: ConfBool) -> Self {
            b.0 as Self
        }
    }

    macro_rules! u8_convert {
        (
            $(
                $(#[$outer:meta])*
                $v:vis enum $name:ident {
                    $(
                        $(#[$inner:meta])*
                        $field:ident = $val:literal,
                    )*
                }
            )*
        ) => {
            $(
            $(#[$outer])*
            $v enum $name {
                $(
                    $(#[$inner])*
                    $field = $val,
                )*
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

    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ALPF_MODE {
        BW2_66 = 0b00,
        BW3_64 = 0b01,
        BW5_39 = 0b10,
        BW13_37 = 0b11,
    }

    /// Attitude Engine Output Data Rate (ODR)
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum SODR {
        ODR1 = 0b000,
        ODR2 = 0b001,
        ODR4 = 0b010,
        ODR8 = 0b011,
        ODR16 = 0b100,
        ODR32 = 0b101,
        ODR64 = 0b111,
    }

    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FIFO_SIZE {
        Samples16 = 0b00,
        Samples32 = 0b01,
        Samples64 = 0b10,
        Samples128 = 0b11,
    }

    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FIFO_MODE {
        /// FIFO disable
        Bypass = 0b00,
        FIFO = 0b01,
        Stream = 0b10,
        /// See datasheet
        StreamToFIFO = 0b11,
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
    CTRL3 = 0x04,
    CTRL5 = 0x06,
    CTRL6 = 0x07,
    CTRL7 = 0x08,
    CTRL8 = 0x09,
    FIFO_CTRL = 0x14,
    FIFO_STATUS = 0x16,
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
            $($fv:vis $f:ident : $ft:ty => $bs:literal : $be:literal),*
            $(,)?
        }
        )+
    ) => {
        $(
        $(#[$outer])*
        $v struct $name { $($fv $f: $ft,)* }

        impl $name {
            pub(crate) fn bits(self) -> u8 {
                let parts: &[u8] = &[
                    $( <$ft as Into<u8>>::into(self.$f) << $bs,)+
                ];
                parts.into_iter().sum()
            }

            pub(crate) fn from_bits_truncate(__i: u8) -> Self {
                Self {
                    $( $f: ((__i & mask($bs, $be)) >> $bs).into(), )*
                }
            }
        }
        )*
    };
}

hasbits! {
    pub(crate) struct CTRL2 {
        pub ast: config::ConfBool => 7:7,
        pub afs: config::AFS => 4:6,
        pub aodr: config::AODR => 0:3,
    }

    pub(crate) struct CTRL3 {
        pub gst: config::ConfBool => 7:7,
        pub gfs: config::GFS => 4:6,
        pub godr: config::GODR => 0:3,
    }

    pub(crate) struct CTRL5 {
        pub glpf_mode: config::GLPF_MODE => 5:6,
        pub glpf_en: config::ConfBool => 4:4,
        pub alpf_mode: config::ALPF_MODE => 1:2,
        pub alpf_en: config::ConfBool => 0:0,
    }

    pub(crate) struct CTRL6 {
        pub smod: config::ConfBool => 7:7,
        pub sodr: config::SODR => 0:2,
    }

    pub(crate) struct FIFO_CTRL {
        pub fifo_rd_mode: config::ConfBool => 7:7,
        pub fifo_size: config::FIFO_SIZE => 2:3,
        pub fifo_mode: config::FIFO_MODE => 0:1,
    }

    pub(crate) struct FIFO_STATUS {
        pub fifo_full: config::ConfBool => 7:7,
        pub fifo_wtm: config::ConfBool => 6:6,
        pub fifo_ovflow: config::ConfBool => 5:5,
        pub fifo_not_empty: config::ConfBool => 4:4,
        pub fifo_smpl_cnt_msb: u8 => 0:1,
    }
}

bitflags! {
    pub(crate) struct CTRL1: u8 {
        /// Change from 4-wire spi to 3-wire spi
        const SIM = 1 << 7;
        /// Enable auto-incrementing address
        const ADDR_AI = 1 << 6;
        /// Enable big-endianness
        const BE = 1 << 5;
        /// Disables internal 2MHz oscillator
        const SENSOR_DISABLE = 1;
    }

    pub(crate) struct CTRL7: u8 {
        /// Enable syncSmple mode
        const SYNC_SMPL = 1 << 7;
        /// Enable high speed internal clock
        const SYS_HS = 1 << 6;
        /// Enable gyroscope snooze mode (see datasheet)
        const GSN = 1 << 4;
        /// Enable AttitudeEngine orientation and velocity increment computation
        const SEN = 1 << 3;
        /// Enable gyroscope
        const GEN = 1 << 1;
        /// Enable accelerometer
        const AEN = 1;
    }

    pub(crate) struct CTRL8: u8 {
        /// Switch from `INT1` to `STATUSINT::CTRL9_CMD_DONE` for handshake
        const CTRL9_HANDSHAKE_TYPE = 1 << 7;
        /// Unclear
        ///
        /// Datasheet appears to have a typo for this one :/
        const INT_MOTION = 1 << 6;
        /// Enable pedometer engine
        const PED_ENABLE = 1 << 4;
        /// Enable significant motion engine
        const SIG_MOTION_ENABLE = 1 << 3;
        /// Enable no motion engine
        const NO_MOTION_ENABLE = 1 << 2;
        /// Enable any motion engine
        const ANY_MOTION_ENABLE = 1 << 1;
        /// Enable tap engine
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

macro_rules! getters {
    ($( $funcname:ident -> $reg:ident $(,)? )*) => {
        $( fn $funcname(&mut self) -> Result<$reg, I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Reading flags from $reg");
            Ok(<$reg>::from_bits_truncate(
                self.read_flags(FlagRegister::$reg)?,
            ))
        } )*
    };
}
macro_rules! setters {
    ($( $funcname:ident -> $reg:ident $(,)? )*) => {
        $( fn $funcname(&mut self, val: $reg) -> Result<(), I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Writing {} to device", val);
            self.write_flags(FlagRegister::$reg, val.bits())
        } )*
    };
}

// TODO: merge this with `Registers`?
pub(crate) trait Flags<I: I2c> {
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error>;
    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error>;

    getters! {
        get_ctrl1 -> CTRL1,
        get_ctrl2 -> CTRL2,
        get_ctrl3 -> CTRL3,
        get_ctrl5 -> CTRL5,
        get_ctrl6 -> CTRL6,
        get_ctrl7 -> CTRL7,
        get_ctrl8 -> CTRL8,
        get_fifo_ctrl -> FIFO_CTRL,
        get_fifo_status -> FIFO_STATUS,
        get_statusint -> STATUSINT,
        get_status0 -> STATUS0,
        get_status1 -> STATUS1,
        get_ae_reg1 -> AE_REG1,
        get_ae_reg2 -> AE_REG2,
    }

    setters! {
        set_ctrl1 -> CTRL1,
        set_ctrl2 -> CTRL2,
        set_ctrl3 -> CTRL3,
        set_ctrl5 -> CTRL5,
        set_ctrl6 -> CTRL6,
        set_ctrl7 -> CTRL7,
        set_ctrl8 -> CTRL8,
        set_fifo_ctrl -> FIFO_CTRL,
    }
}

impl<I: I2c, T> Flags<I> for T
where
    T: Registers<I>,
{
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error> {
        Ok(self.read_raw::<1>(reg as u8)?[0])
    }

    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error> {
        self.write_raw(reg as u8, [val])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hasbits_ctrl3() {
        assert_eq!(
            CTRL3 {
                gst: false.into(),
                gfs: config::GFS::GFS256dps,
                godr: config::GODR::GODR58_75,
            }
            .bits(),
            0b0100_0111,
        );
    }

    #[test]
    fn hasbits_ctrl5() {
        assert_eq!(
            CTRL5 {
                glpf_mode: config::GLPF_MODE::BW2_66,
                glpf_en: true.into(),
                alpf_mode: config::ALPF_MODE::BW13_37,
                alpf_en: false.into(),
            }
            .bits(),
            0b0001_0110,
        );
    }
}
