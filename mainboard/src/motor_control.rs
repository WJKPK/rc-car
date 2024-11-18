use crate::pwm_split::{
    PwmChannel, PwmChannelLowSpeed, PwmDuty, SplitedPwm, SplitterError, TimerSpeed,
};
use common::safe_types::Angle;
use esp_hal::{
    gpio::AnyPin,
    prelude::*,
};

#[derive(PartialEq, Eq, Copy, Clone, Debug, defmt::Format)]
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

pub struct PowerMotorInternals {
    pub pins: [AnyPin; Drv8210Input::NumOfInputs as usize],
    pub channels: [PwmChannel; Drv8210Input::NumOfInputs as usize],
}

impl<'a> PowerMotorInternals
{
    pub fn new(
        pins: [AnyPin; Drv8210Input::NumOfInputs as usize],
        channels: [PwmChannel; Drv8210Input::NumOfInputs as usize],
    ) -> Self {
        PowerMotorInternals { pins, channels }
    }
}

pub struct SerwoMotorInternals {
    control: [AnyPin; 1],
    channel: [PwmChannel; 1],
}

impl SerwoMotorInternals {
    pub fn new(control: AnyPin, channel: PwmChannel) -> Self {
        SerwoMotorInternals {
            control: [control],
            channel: [channel],
        }
    }
}

pub struct MotorDescriptorBoundle {
    pub front_motor: PowerMotorInternals,
    pub back_motor: PowerMotorInternals,
    pub serwo: SerwoMotorInternals,
}

impl<'a> MotorDescriptorBoundle
{
    pub fn new(
        front: PowerMotorInternals,
        back: PowerMotorInternals,
        serwo: SerwoMotorInternals,
    ) -> Self {
        MotorDescriptorBoundle {
            front_motor: front,
            back_motor: back,
            serwo,
        }
    }

    pub fn setup_driver(
        &'a mut self,
        splitted: &'a SplitedPwm<'a>,
    ) -> Result<MotorDriver<'a>, MotorError> {
        let front_motor = splitted
            .create_channels::<{ Drv8210Input::NumOfInputs as usize }>(
                TimerSpeed::FastTimer,
                &mut self.front_motor.pins,
                self.front_motor.channels,
            )
            .map_err(|e| MotorError::from(e))?;

        let back_motor = splitted
            .create_channels::<{ Drv8210Input::NumOfInputs as usize }>(
                TimerSpeed::FastTimer,
                &mut self.back_motor.pins,
                self.back_motor.channels,
            )
            .map_err(|e| MotorError::from(e))?;

        let serwo = splitted
            .create_channels::<1>(
                TimerSpeed::SlowTimer,
                &mut self.serwo.control,
                self.serwo.channel,
            )
            .map_err(|e| MotorError::from(e))?;

        Ok(MotorDriver {
            front_motor,
            back_motor,
            serwo,
        })
    }
}

pub struct MotorDriver<'a> {
    front_motor: ([PwmChannelLowSpeed<'a>; 2], PwmDuty),
    back_motor: ([PwmChannelLowSpeed<'a>; 2], PwmDuty),
    serwo: ([PwmChannelLowSpeed<'a>; 1], PwmDuty),
}

impl<'a> MotorDriver<'a> {
    fn set_duty(
        channel: &PwmChannelLowSpeed<'a>,
        duty: PwmDuty,
        duty_pct: f32,
    ) -> Result<(), MotorError> {
        if duty_pct > 1f32 {
            return Err(MotorError::InputParameter);
        }

        let duty_exp = duty as u32;
        let duty_range = 2u32.pow(duty_exp);
        let duty_value = duty_range as f32 * duty_pct;

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
        let duties: [[f32; Drv8210Input::NumOfInputs as usize];
            Direction::NumOfDirections as usize] =
            [[power_percent, 0f32], [0f32, power_percent], [1f32, 1f32]];
        for (i, pwm_channel) in self.front_motor.0.iter().enumerate() {
            Self::set_duty(
                pwm_channel,
                self.front_motor.1,
                duties[direction as usize][i],
            )?;
        }
        for (i, pwm_channel) in self.back_motor.0.iter().enumerate() {
            Self::set_duty(
                pwm_channel,
                self.back_motor.1,
                duties[direction as usize][i],
            )?;
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
