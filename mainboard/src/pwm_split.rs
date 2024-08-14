use heapless::Vec;
use fugit::HertzU32;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, AdcChannel, Attenuation},
    clock::Clocks,
    gpio::{any_pin::AnyPin, AnalogPin, OutputPin},
    peripheral::Peripheral,
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

pub type PwmTimerLowSpeed<'a> = Timer<'a, LowSpeed>;
pub type PwmChannelLowSpeed<'a, O> = Channel<'a, LowSpeed, O>;
pub type PwmChannel = channel::Number;
pub type PwmTimer = timer::Number;
pub type PwmDuty = timer::config::Duty;

pub struct SplitedPwm<'a> {
    ledc: &'a Ledc<'a>,
    timer: PwmTimerLowSpeed<'a>,
    channel: Option<&'a mut Vec<PwmChannelLowSpeed<'a, AnyPin<'a>>, 5>>
}

impl<'a> SplitedPwm<'a> {
    pub fn new(ledc: &'a Ledc) -> Self {
        let timer = Self::timer_low_speed(ledc, PwmTimer::Timer0, PwmDuty::Duty5Bit, 24.kHz());
        SplitedPwm {
            ledc,
            timer,
            channel: None}
    }

    pub fn create_channels(&'a self, channels: &mut Vec::<PwmChannelLowSpeed<'a, AnyPin<'a>>, 5>, pin: AnyPin<'a>) {
        let channel = Self::channel_low_speed(self.ledc, &self.timer, channel::Number::Channel0, pin, 0);
    }

    fn timer_low_speed(ledc: &'a Ledc, number: PwmTimer, duty:PwmDuty, frequency:HertzU32)->PwmTimerLowSpeed<'a>{
        let mut lstimer = ledc.get_timer::<LowSpeed>(number);
        lstimer.configure(timer::config::Config {
            duty,
            clock_source: timer::LSClockSource::APBClk,
            frequency,
        }).unwrap();
        lstimer
    }

    fn channel_low_speed<O:  OutputPin>(ledc: &'a Ledc, timer:&'a PwmTimerLowSpeed, number: PwmChannel,
        output_pin: impl Peripheral<P = O> + 'a, init:u32)->PwmChannelLowSpeed<'a, O>{
        let mut chan = ledc.get_channel(number, output_pin);
        chan.configure(channel::config::Config {
            timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        }).unwrap();
        chan.set_duty_hw(init);
        chan
    }
}
