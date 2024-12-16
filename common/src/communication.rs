use zerocopy::{AsBytes, FromBytes, FromZeroes};
pub const COMUNICATION_PERIOD_MS: u64 = 40;
use crate::safe_types::{Angle, Percent};
use crc::{CRC_8_I_432_1, Crc};

#[derive(Debug, Default, Clone, Copy, PartialEq, AsBytes, FromZeroes, FromBytes)]
#[repr(C, packed)]
pub struct RcCarControlViaEspReady {
    pub power: Percent,
    pub turn: Angle,
    pub crc: u8
}

const RC_CAR_CONTROL_SIZE: usize = core::mem::size_of::<RcCarControlViaEspReady>();
impl RcCarControlViaEspReady {
    pub fn new(turn: Option<Angle>, power: Option<Percent>) -> Option<Self> {
        match (turn, power) {
            (Some(turn), Some(power)) => {
                let crc = Crc::<u8>::new(&CRC_8_I_432_1);
                let mut digest = crc.digest();
                digest.update(&turn.to_ne_bytes());
                digest.update(&power.to_ne_bytes());
                Some(RcCarControlViaEspReady { turn, power, crc: digest.finalize() })
            },
            _ => None,
        }
    }

    fn crc_correct(&self) -> bool {
        let crc = Crc::<u8>::new(&CRC_8_I_432_1);
        let mut digest = crc.digest();
        let ptr = core::ptr::addr_of!(self.power);
        let power = unsafe { ptr.read_unaligned() };
        digest.update(&self.turn.to_ne_bytes());
        digest.update(&power.to_ne_bytes());
        self.crc == digest.finalize()
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<&Self> {
        if bytes.len() != RC_CAR_CONTROL_SIZE {
            return None;
        }
        
        let control = RcCarControlViaEspReady::ref_from(bytes)?;
        if !control.crc_correct() {
            return None;
        }
        Some(control)
    }

    pub fn turn(&self) -> i8 {
        *self.turn
    }

    pub fn power(&self) -> f32 {
        let power = self.power;
        *power
    }

    pub fn as_slice(&self) -> &[u8; RC_CAR_CONTROL_SIZE] {
        zerocopy::transmute_ref!(self)
    }
    
    pub fn as_mut_slice(&mut self) -> &mut [u8; RC_CAR_CONTROL_SIZE] {
        zerocopy::transmute_mut!(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_create_object_as_oneliner() {
        let angle = 10;
        let power = 90.0;
        let payload = RcCarControlViaEspReady::new(Angle::new(angle), Percent::new(power)).unwrap();
        assert_eq!(angle, payload.turn());
        assert_eq!(power, payload.power());
    }

    #[test]
    fn cannot_create_invalid() {
        assert_eq!(None, RcCarControlViaEspReady::new(Angle::new(-100), Percent::new(90.0)));
    }

    #[test]
    fn can_serialize() {
        let angle = 10;
        let power = 90.0;
        let payload = RcCarControlViaEspReady::new(Angle::new(angle), Percent::new(power)).unwrap();
        assert_eq!(payload.as_bytes(), [0, 0, 180, 66, 10, 90]);
    }

    #[test]
    fn crc_work() {
        let angle = 10;
        let power = 90.0;
        let payload = RcCarControlViaEspReady::new(Angle::new(angle), Percent::new(power)).unwrap();
        assert!(payload.crc_correct());
    }

}
