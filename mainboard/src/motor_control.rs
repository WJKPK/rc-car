use heapless::Vec;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcChannel, Attenuation},
    clock::Clocks,
    gpio::{any_pin::AnyPin, AnalogPin},
    peripherals::{ADC1, LEDC},
    prelude::*,
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
    },
};

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Direction {
    Forward = 0,
    Backward = 1,
    NumOfDirections = 2
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Drv8210Input {
    In1 = 0,
    In2 = 1,
    NumOfInputs = 2
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorError {
    TimerSetup,
    ChannelInit,
    Duty,
}

pub struct MotorSpecific <'a, AdcPin>{
    in_1: AnyPin<'a>,
    in_2: AnyPin<'a>,
    adc_pin: AdcPin,
    pwm_channel: Vec<Channel<'a, LowSpeed, AnyPin<'a>>, {Drv8210Input::NumOfInputs as usize}>
}

impl<'a, AdcPin: AnalogPin + AdcChannel> MotorSpecific<'a, AdcPin> {
    pub fn new(in_1: AnyPin<'a>, in_2: AnyPin<'a>, adc_pin: AdcPin) -> Self {
        MotorSpecific { in_1, in_2, adc_pin, pwm_channel: Vec::new()}
    }

    fn init_channels(&'a mut self, ledc: &'a Ledc<'a>, timer: &'a Timer<'a, LowSpeed>, channels: [channel::Number; 2]) -> Result<(), MotorError> {
        let mut channels: [Channel<LowSpeed, AnyPin<'a>>; Drv8210Input::NumOfInputs as usize] = [
            ledc.get_channel(channels[Drv8210Input::In1 as usize], &mut self.in_1),
            ledc.get_channel(channels[Drv8210Input::In2 as usize], &mut self.in_2)
        ];

        for mut channel in channels {
            channel.configure(channel::config::Config {
                timer,
                duty_pct: 0,
                pin_config: channel::config::PinConfig::PushPull,
            }).map_err(|_| MotorError::ChannelInit)?;
            self.pwm_channel.push(channel).map_err(|_| MotorError::ChannelInit)?;
        }
        Ok(())
    }
}

pub struct MotorControlDriver<'a, AdcFront, AdcBack> {
    front_motor: MotorSpecific<'a, AdcFront>,
    back_motor: MotorSpecific<'a, AdcBack>,
    adc: ADC1,
    clocks: &'a Clocks<'a>,
    ledc: Ledc<'a>,
    timer: Option<Timer<'a, LowSpeed>>
}

impl <'a, AdcFront, AdcBack> MotorControlDriver<'a, AdcFront, AdcBack> 
where
    AdcFront: AnalogPin + AdcChannel,
    AdcBack: AnalogPin + AdcChannel
    {
    pub fn new(front: MotorSpecific<'a, AdcFront>, back: MotorSpecific<'a, AdcBack>, adc: ADC1, ledc: LEDC, clocks: &'a Clocks<'a>) -> Self {
        let mut ledc = Ledc::new(ledc, clocks);
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
        MotorControlDriver{ front_motor: front, back_motor: back, adc, clocks, ledc, timer: None}
    }

    pub fn setup_pwm(&'a mut self) -> Result<(), MotorError>{
        self.timer = Some(self.ledc.get_timer::<LowSpeed>(timer::Number::Timer0));
        match &mut self.timer {
            Some(x) => {
                x.configure(timer::config::Config {
                    duty: timer::config::Duty::Duty5Bit,
                    clock_source: timer::LSClockSource::APBClk,
                    frequency: 24.kHz(),
                }).map_err(|_| MotorError::TimerSetup)?;
                self.front_motor.init_channels(&self.ledc, x, [channel::Number::Channel0, channel::Number::Channel1])?;
                self.back_motor.init_channels(&self.ledc, x, [channel::Number::Channel2, channel::Number::Channel3])?;    
            }
            None => unreachable!()
        };
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
    fn launch_motor(&self, direction: Direction, power_percent: u8) -> Result<(), MotorError> {
        let duties: [[u8; Drv8210Input::NumOfInputs as usize]; Direction::NumOfDirections as usize] = [
            [power_percent, 0],
            [0, power_percent]
        ];
        for (i, pwm_channel) in self.front_motor.pwm_channel.iter().enumerate() {
            pwm_channel.set_duty(duties[i][direction as usize]).map_err(|_| MotorError::Duty)?;
        }
        for (i, pwm_channel) in self.back_motor.pwm_channel.iter().enumerate() {
            pwm_channel.set_duty(duties[i][direction as usize]).map_err(|_| MotorError::Duty)?;
        }
        Ok(())
    }
}
