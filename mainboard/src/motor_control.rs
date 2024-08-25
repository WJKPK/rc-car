use core::ops::{Deref, DerefMut};
use esp_hal::{
//  analog::adc::{Adc, AdcChannel, AdcConfig, Attenuation},
    gpio::OutputPin,
    peripheral::Peripheral,
//  peripherals::ADC1,
    prelude::*,
};
use crate::pwm_split::{PwmChannel, PwmDuty, PwmChannelLowSpeed, SplitedPwm, TimerSpeed, SplitterError};
use comunication::Angle;
use esp_println::println;


#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Direction {
    Forward = 0,
    Backward = 1,
    Stop = 2,
    NumOfDirections = 3,
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Drv8210Input {
    In1 = 0,
    In2 = 1,
    NumOfInputs = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum MotorError {
    InternalSetup(SplitterError),
    InputParameter,
}

impl From<SplitterError> for MotorError {
    fn from(error: SplitterError) -> Self {
        MotorError::InternalSetup(error)
    }
}

pub struct PowerMotorInternals<O> {
    pub pins: [O; Drv8210Input::NumOfInputs as usize],
    pub channels: [PwmChannel; Drv8210Input::NumOfInputs as usize],
}

impl<'a, O> PowerMotorInternals<O>
where
    O: Peripheral<P = O> + OutputPin + 'a,
{
    pub fn new(
        pins: [O; Drv8210Input::NumOfInputs as usize],
        channels: [PwmChannel; Drv8210Input::NumOfInputs as usize],
    ) -> Self {
        PowerMotorInternals { pins, channels }
    }
}

pub struct SerwoMotorInternals<O> {
    control: [O; 1],
    channel: [PwmChannel; 1],
}

impl<O: OutputPin> SerwoMotorInternals<O> {
    pub fn new(control: O, channel: PwmChannel) -> Self {
        SerwoMotorInternals { control: [control], channel: [channel] }
    }
}

pub struct MotorDescriptorBoundle<O> {
    pub front_motor: PowerMotorInternals<O>,
    pub back_motor: PowerMotorInternals<O>,
    pub serwo: SerwoMotorInternals<O>,
}

impl<'a, O> MotorDescriptorBoundle<O>
where 
    O: Peripheral<P = O> + OutputPin + 'a,
{
    pub fn new(
        front: PowerMotorInternals<O>,
        back: PowerMotorInternals<O>,
        serwo: SerwoMotorInternals<O>,
    ) -> Self {
        MotorDescriptorBoundle {
            front_motor: front,
            back_motor: back,
            serwo,
        }
    }

    pub fn setup_driver(&'a mut self, splitted: &'a SplitedPwm<'a>) -> Result<MotorDriver<'a, O>, MotorError> {
        let front_motor = splitted.create_channels::<O, {Drv8210Input::NumOfInputs as usize}> (
           TimerSpeed::FastTimer,
           &mut self.front_motor.pins,
           self.front_motor.channels,
        ).map_err(|e| MotorError::from(e))?;

        let back_motor = splitted.create_channels::<O, {Drv8210Input::NumOfInputs as usize}> (
           TimerSpeed::FastTimer,
           &mut self.back_motor.pins,
           self.back_motor.channels,
        ).map_err(|e| MotorError::from(e))?;

        let serwo = splitted.create_channels::<O, 1> (
            TimerSpeed::SlowTimer,
            &mut self.serwo.control,
            self.serwo.channel,
        ).map_err(|e| MotorError::from(e))?;

        Ok(MotorDriver{front_motor, back_motor, serwo})
    }
}


pub struct MotorDriver<'a, O: OutputPin> {
    front_motor: ([PwmChannelLowSpeed<'a, O>; 2], PwmDuty),
    back_motor: ([PwmChannelLowSpeed<'a, O>; 2], PwmDuty),
    serwo: ([PwmChannelLowSpeed<'a, O>; 1], PwmDuty)
}


impl<'a, O: OutputPin> MotorDriver<'a, O> {
    fn set_duty(channel: &PwmChannelLowSpeed<'a, O>, duty: PwmDuty, duty_pct: f32) -> Result<(), MotorError> {
        if duty_pct > 1f32 {
            return Err(MotorError::InputParameter);
        }

        let duty_exp = duty as u32;
        let duty_range = 2u32.pow(duty_exp);
        let duty_value = duty_range as f32 * duty_pct;

        println!("Duty is: {:?}, duty_range: {:?}, duty_value: {:?}", duty_exp, duty_range, duty_value);

        channel.set_duty_hw(duty_value as u32);
        Ok(())
    }

    //  +-----+-----+------+------+---------------------------------+
    //  | IN1 | IN2 | OUT1 | OUT2 | DESCRIPTION                     |
    //  +-----+-----+------+------+---------------------------------+
    //  |  0  |  0  | Hi-Z | Hi-Z | Coast (H-bridge Hi-Z)/          |
    //  |     |     |      |      | low-power automatic sleep mode  |
    //  +-----+-----+------+------+---------------------------------+
    //  |  0  |  1  |  L   |  H   | Reverse                         |
    //  +-----+-----+------+------+---------------------------------+
    //  |  1  |  0  |  H   |  L   | Forward                         |
    //  +-----+-----+------+------+---------------------------------+
    //  |  1  |  1  |  L   |  L   | Brake                           |
    //  +-----+-----+------+------+---------------------------------+
    pub fn drive_motor(&self, direction: Direction, power_percent: f32) -> Result<(), MotorError> {
        let duties: [[f32; Drv8210Input::NumOfInputs as usize]; Direction::NumOfDirections as usize] = [
            [power_percent, 0f32],
            [0f32, power_percent],
            [100f32, 100f32]
        ];
        for (i, pwm_channel) in self.front_motor.0.iter().enumerate() {
            Self::set_duty(pwm_channel, self.front_motor.1, duties[i][direction as usize])?;
        }
        for (i, pwm_channel) in self.back_motor.0.iter().enumerate() {
            Self::set_duty(pwm_channel, self.back_motor.1, duties[i][direction as usize])?;
        }
        Ok(())
    }

    fn map_value(input: Angle) -> f32 {
        let input_min = -90;
        let input_max = 90;
        let output_min = 0.97;
        let output_max = 0.87;
    
        let input_range = (input_max - input_min) as f32;
        let output_range = output_max - output_min;
    
        let normalized = (*input as i32 - input_min) as f32 / input_range;
        let mapped = output_min + (normalized * output_range);
    
        mapped
    }

    pub fn drive_serwo(&self, angle: Angle) -> Result<(), MotorError> {
        Self::set_duty(&self.serwo.0[0], self.serwo.1, Self::map_value(angle))?;
        Ok(())
    }
}
