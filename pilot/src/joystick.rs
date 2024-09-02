use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation, AdcChannel, AdcPin, AdcCalLine},
    gpio::{any_pin::AnyPin, AnalogPin},
    peripherals::ADC1, peripheral::Peripheral
}; use core::marker::PhantomData;
use nb;
use core::ops::{Neg,Div, Mul, Sub, Add};
use num_traits::float::Float;

type JoystickCal = AdcCalLine<ADC1>; 
type JoystickReaderConf = AdcConfig<ADC1>;

pub struct Joystick<'a, O, N> {
    x_pin: AdcPin<O, ADC1, JoystickCal>,
    y_pin: AdcPin<N, ADC1, JoystickCal>,
    adc: Adc<'a, ADC1>,
}
use comunication::{Angle, Percent};
use num_complex::Complex32;

#[derive(Debug)]
pub enum Error {
    Read,
}

impl<'a, O, N> Joystick<'a, O, N>
where
    O: Peripheral<P = O> + AnalogPin + AdcChannel + 'a,
    N: Peripheral<P = N> + AnalogPin + AdcChannel + 'a,
{
    pub fn new(x_pin: O, y_pin: N, adc: ADC1) -> Self {
        let mut adc_config = AdcConfig::new();
        let mut adc0_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(x_pin, Attenuation::Attenuation11dB);
        let mut adc1_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(y_pin, Attenuation::Attenuation11dB);
        let mut adc = Adc::new(adc, adc_config);
        Joystick::<O,N>{x_pin: adc0_pin, y_pin: adc1_pin, adc}
    }

    pub fn read(&mut self) -> Result<(u16, u16), Error> {
        let x_axis = (nb::block!(self.adc.read_oneshot(&mut self.x_pin))).map_err(|_| Error::Read)?;
        let y_axis = (nb::block!(self.adc.read_oneshot(&mut self.y_pin))).map_err(|_| Error::Read)?;
        Ok((x_axis, y_axis))
    }

    pub fn convert(measurements: (u16, u16)) -> (Option<Angle>, Option<Percent>) {
        const BASE_ANGLE: f32 = 640.0;
        const BASE_POWER: f32 = 660.0;
        const MAX_POWER_MAGNITUDE: f32 = 550.0;
        const PI_2: f32 = core::f32::consts::PI / 2.0f32;

        let (angle, power) = measurements;
        let (angle, power): (f32, f32) = (<u16 as Into<f32>>::into(angle) - BASE_ANGLE, <u16 as Into<f32>>::into(power) - BASE_POWER);

        let movement_vector = Complex32::new(angle.into(), power.into()).conj() * Complex32::new(0.0f32, -1.0f32);
        let (magnitude, mut angle) = movement_vector.to_polar();
        
        // Ugh, any beter way to do this?
        let power_sign = if power > 0.0f32 { -1.0f32 } else { 1.0f32 };
        if angle > PI_2 {
            angle = PI_2 - (angle % PI_2);
        }
        if angle < -PI_2 {
            angle = -PI_2 - (angle % PI_2);
        }

        let mut angle = ((90.0f32 * angle) / PI_2) / 2.5f32;
        let mut power: f32 = (power_sign *
            (magnitude / MAX_POWER_MAGNITUDE).clamp(0.0f32, 1.0f32)) * 100.0f32;

        if power.abs() < 30.0f32 {
            power = 0.0f32;
            angle = 0.0f32;
        }
        let angle_i8 = -1 * (angle as i32).max(i8::MIN as i32).min(i8::MAX as i32) as i8;
        defmt::info!("angle: {:?}, power: {:?}", angle_i8, power);
        (Angle::new(angle_i8), Percent::new(power))
    }
}

