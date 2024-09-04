use crate::pwm_split::{
    PwmChannel, PwmChannelLowSpeed, PwmDuty, SplitedPwm, SplitterError, TimerSpeed,
};
use common::safe_types::Angle;
use esp_hal::{
    analog::adc::{Adc, AdcPin, AdcChannel, AdcConfig, Attenuation, AdcCalLine},
    gpio::{OutputPin, AnalogPin},
    peripheral::Peripheral,
    peripherals::ADC1,
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
        SerwoMotorInternals {
            control: [control],
            channel: [channel],
        }
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

    pub fn setup_driver(
        &'a mut self,
        splitted: &'a SplitedPwm<'a>,
    ) -> Result<MotorDriver<'a, O>, MotorError> {
        let front_motor = splitted
            .create_channels::<O, { Drv8210Input::NumOfInputs as usize }>(
                TimerSpeed::FastTimer,
                &mut self.front_motor.pins,
                self.front_motor.channels,
            )
            .map_err(|e| MotorError::from(e))?;

        let back_motor = splitted
            .create_channels::<O, { Drv8210Input::NumOfInputs as usize }>(
                TimerSpeed::FastTimer,
                &mut self.back_motor.pins,
                self.back_motor.channels,
            )
            .map_err(|e| MotorError::from(e))?;

        let serwo = splitted
            .create_channels::<O, 1>(
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

pub struct MotorDriver<'a, O: OutputPin> {
    front_motor: ([PwmChannelLowSpeed<'a, O>; 2], PwmDuty),
    back_motor: ([PwmChannelLowSpeed<'a, O>; 2], PwmDuty),
    serwo: ([PwmChannelLowSpeed<'a, O>; 1], PwmDuty),
}

impl<'a, O: OutputPin> MotorDriver<'a, O> {
    fn set_duty(
        channel: &PwmChannelLowSpeed<'a, O>,
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

type MotorCurrentCal = AdcCalLine<ADC1>; 
type MiliAmperes = u32;
type MiliOhm = u32;
type MiliVolt = u32;

pub struct MotorCurrentObserver<'a, O, N> {
    x_pin: AdcPin<O, ADC1, MotorCurrentCal>,
    y_pin: AdcPin<N, ADC1, MotorCurrentCal>,
    adc: Adc<'a, ADC1>,
}

#[derive(Debug)]
pub enum Error {
    Read,
}

impl<'a, O, N> MotorCurrentObserver<'a, O, N>
where
    O: Peripheral<P = O> + AnalogPin + AdcChannel + 'a,
    N: Peripheral<P = N> + AnalogPin + AdcChannel + 'a,
{
    pub fn new(x_pin: O, y_pin: N, adc: ADC1) -> Self {
        let mut adc_config = AdcConfig::new();
        let adc0_pin = adc_config.enable_pin_with_cal::<_, MotorCurrentCal>(x_pin, Attenuation::Attenuation11dB);
        let adc1_pin = adc_config.enable_pin_with_cal::<_, MotorCurrentCal>(y_pin, Attenuation::Attenuation11dB);
        let adc = Adc::new(adc, adc_config);
        MotorCurrentObserver::<O,N>{x_pin: adc0_pin, y_pin: adc1_pin, adc}
    }

    fn read(&mut self) -> Result<(MiliAmperes, MiliAmperes), Error> {
        const R1: MiliOhm = 15;
        const R2: MiliOhm = 1000000;
        const R3: MiliOhm = 10000000;

        let convert = |meas: u16| -> MiliAmperes {(R2 * 10 * meas as MiliVolt) / (R1 * R3)};
        let front_motor = (nb::block!(self.adc.read_oneshot(&mut self.x_pin))).map(convert).map_err(|_| Error::Read)?;
        let back_motor = (nb::block!(self.adc.read_oneshot(&mut self.y_pin))).map(convert).map_err(|_| Error::Read)?;

        Ok((front_motor, back_motor))
    }

    pub fn current_in_range(&mut self) -> bool {
    const CURRENT_THRESHOLD: MiliAmperes = 1500;

    self.read()
        .map(|(front, back)| front <= CURRENT_THRESHOLD && back <= CURRENT_THRESHOLD)
        .unwrap_or(false)
    }
}
