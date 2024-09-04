use core::ops::{Deref, DerefMut};
use core::default::Default;
use zerocopy::{AsBytes, FromBytes, FromZeroes};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, AsBytes, FromZeroes, FromBytes)]
#[repr(C)]

pub struct Angle(i8);
const ANGLE_SIZE: usize = core::mem::size_of::<Angle>();

impl Angle {
    pub const MIN: i8 = -90;
    pub const MAX: i8 = 90;

    pub const fn new(value: i8) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Angle(value))
        } else {
            None
        }
    }

    pub const fn get(&self) -> i8 {
        self.0
    }

    pub fn as_slice(&self) -> &[u8; ANGLE_SIZE] {
        zerocopy::transmute_ref!(self)
    }
    
    pub fn as_mut_slice(&mut self) -> &mut [u8; ANGLE_SIZE] {
        zerocopy::transmute_mut!(self)
    }

}

impl Deref for Angle {
    type Target = i8;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Angle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, AsBytes, FromZeroes, FromBytes)]
#[repr(C)]

pub struct Percent(f32);
const PERCENT_SIZE: usize = core::mem::size_of::<Percent>();
impl Percent {
    pub const MIN: f32 = -100.0;
    pub const MAX: f32 = 100.0;

    pub fn new(value: f32) -> Option<Self> {
        if value >= Self::MIN && value <= Self::MAX {
            Some(Percent(value))
        } else {
            None
        }
    }

    pub fn get(&self) -> f32 {
        self.0
    }

    pub fn as_slice(&self) -> &[u8; PERCENT_SIZE] {
        zerocopy::transmute_ref!(self)
    }
    
    pub fn as_mut_slice(&mut self) -> &mut [u8; PERCENT_SIZE] {
        zerocopy::transmute_mut!(self)
    }
}

impl Deref for Percent {
    type Target = f32;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Percent {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

