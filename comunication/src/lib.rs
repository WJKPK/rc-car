#![no_std]

pub mod types;
pub use types::{Angle, Percent};
use zerocopy::{AsBytes, FromBytes, FromZeroes};

#[derive(Debug, Default, Clone, Copy, PartialEq, AsBytes, FromZeroes, FromBytes)]
#[repr(C, packed)]
pub struct RcCarControlViaEspReady {
    pub turn: Angle,
    pub power: Percent,
}

const RC_CAR_CONTROL_SIZE: usize = core::mem::size_of::<RcCarControlViaEspReady>();
impl RcCarControlViaEspReady {
    pub fn new(turn: Option<Angle>, power: Option<Percent>) -> Option<Self> {
        match (turn, power) {
            (Some(turn), Some(power)) => Some(RcCarControlViaEspReady { turn, power }),
            _ => None,
        }
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
        assert_eq!(payload.as_bytes(), [10, 0, 0, 180, 66]);
    }

}
