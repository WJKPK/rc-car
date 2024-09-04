use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation, AdcChannel, AdcPin, AdcCalLine},
    gpio::AnalogPin,
    peripherals::ADC1, peripheral::Peripheral
};

use common::{
    safe_types::{Angle, Percent},
    rc_filter::FilterAdaptor
};
use nb;
use core::f32::consts::PI;
use num_complex::Complex32;
use num_traits::float::Float;

type JoystickCal = AdcCalLine<ADC1>; 

pub struct Joystick<'a, O, N> {
    x_pin: AdcPin<O, ADC1, JoystickCal>,
    y_pin: AdcPin<N, ADC1, JoystickCal>,
    adc: Adc<'a, ADC1>,
}

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
        let adc0_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(x_pin, Attenuation::Attenuation11dB);
        let adc1_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(y_pin, Attenuation::Attenuation11dB);
        let adc = Adc::new(adc, adc_config);
        Joystick::<O,N>{x_pin: adc0_pin, y_pin: adc1_pin, adc}
    }

    pub fn read(&mut self) -> Result<(u16, u16), Error> {
        let x_axis = (nb::block!(self.adc.read_oneshot(&mut self.x_pin))).map_err(|_| Error::Read)?;
        let y_axis = (nb::block!(self.adc.read_oneshot(&mut self.y_pin))).map_err(|_| Error::Read)?;
        Ok((x_axis, y_axis))
    }

    pub fn convert(measurements: (u16, u16), mut power_filter: impl FilterAdaptor, mut angle_filter: impl FilterAdaptor) -> (Option<Angle>, Option<Percent>) {
        const BASE_ANGLE: f32 = 640.0;
        const BASE_POWER: f32 = 660.0;
        const MAX_POWER_MAGNITUDE: f32 = 550.0;
        const HALF_PI: f32 = PI / 2.0;

        let (angle, power) = measurements;
        let (angle, power) = (f32::from(angle) - BASE_ANGLE, f32::from(power) - BASE_POWER);

        let movement_vector = Complex32::new(angle, power).conj() * Complex32::new(0.0, -1.0);
        let (magnitude, mut angle) = movement_vector.to_polar();

        if angle > HALF_PI {
            angle = HALF_PI - (angle % HALF_PI);
        }
        if angle < -HALF_PI {
            angle = -HALF_PI - (angle % HALF_PI);
        }

        let angle = angle_filter.process((90.0 * angle / HALF_PI) / 2.5);
        let power = power_filter.process(-((magnitude / MAX_POWER_MAGNITUDE).clamp(0.0, 1.0) * 100.0).copysign(power));

        defmt::info!("angle: {:?}, power: {:?}", angle, power);

        if power.abs() < 30.0 {
            (None, None)
        } else {
            let angle_i8 = (-angle as i8).clamp(i8::MIN, i8::MAX);
            defmt::info!("angle: {:?}, power: {:?}", angle_i8, power);
            (Angle::new(angle_i8), Percent::new(power))
        }
    }
}

