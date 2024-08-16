use array_init;
use esp_hal::{
    gpio::OutputPin,
    ledc::{
        channel::{self, Channel, ChannelIFace},
        timer::{self, Timer, TimerIFace},
        Ledc, LowSpeed,
    },
    peripheral::Peripheral,
    prelude::*,
};
use fugit::HertzU32;

#[repr(usize)]
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum TimerSpeed {
    SlowTimer,
    FastTimer,
    NumberOfTimers,
}

pub type PwmTimerLowSpeed<'a> = Timer<'a, LowSpeed>;
pub type PwmChannelLowSpeed<'a, O> = Channel<'a, LowSpeed, O>;
pub type PwmChannel = channel::Number;
pub type PwmTimer = timer::Number;
pub type PwmDuty = timer::config::Duty;

pub struct SplitedPwm<'a> {
    ledc: &'a Ledc<'a>,
    timer: [PwmTimerLowSpeed<'a>; TimerSpeed::NumberOfTimers as usize],
}

impl<'a> SplitedPwm<'a> {
    pub fn new(ledc: &'a Ledc) -> Self {
        let timer_fast = Self::timer_low_speed(ledc, PwmTimer::Timer0, PwmDuty::Duty5Bit, 24.kHz());
        let timer_slow = Self::timer_low_speed(ledc, PwmTimer::Timer1, PwmDuty::Duty14Bit, 50.Hz());

        SplitedPwm {
            ledc,
            timer: [timer_slow, timer_fast],
        }
    }

    pub fn create_channels<O, const N: usize>(
        &'a self,
        timer_type: TimerSpeed,
        pin: &'a mut [O; N],
        channel_number: [PwmChannel; N],
    ) -> ([PwmChannelLowSpeed<O>; N], PwmDuty)
    where
        O: Peripheral<P = O> + OutputPin + 'a,
    {
        let arr: [PwmChannelLowSpeed<O>; N] =
            array_init::from_iter(pin.iter_mut().enumerate().map(|(i, p)| {
                Self::channel_low_speed(
                    self.ledc,
                    &self.timer[timer_type as usize],
                    channel_number[i],
                    p,
                    100,
                )
            }))
            .unwrap();
        (arr, self.timer[timer_type as usize].get_duty().unwrap())
    }

    fn timer_low_speed(
        ledc: &'a Ledc,
        number: PwmTimer,
        duty: PwmDuty,
        frequency: HertzU32,
    ) -> PwmTimerLowSpeed<'a> {
        let mut lstimer = ledc.get_timer::<LowSpeed>(number);
        lstimer
            .configure(timer::config::Config {
                duty,
                clock_source: timer::LSClockSource::APBClk,
                frequency,
            })
            .unwrap();
        lstimer
    }

    fn channel_low_speed<O: OutputPin>(
        ledc: &'a Ledc,
        timer: &'a PwmTimerLowSpeed,
        number: PwmChannel,
        output_pin: impl Peripheral<P = O> + 'a,
        init: u32,
    ) -> PwmChannelLowSpeed<'a, O> {
        let mut chan = ledc.get_channel(number, output_pin);
        chan.configure(channel::config::Config {
            timer,
            duty_pct: 0,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
        chan.set_duty_hw(init);
        chan
    }
}
