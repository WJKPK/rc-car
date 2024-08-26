use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation, AdcChannel, AdcPin, AdcCalLine},
    gpio::{any_pin::AnyPin, AnalogPin},
    peripherals::ADC1, peripheral::Peripheral
}; use core::marker::PhantomData;
use nb;
use core::ops::{Neg,Div, Mul, Sub, Add};

type JoystickCal = AdcCalLine<ADC1>; 
type JoystickReaderConf = AdcConfig<ADC1>;

pub struct Joystick<O, N> {
    x_pin: AdcPin<O, ADC1, JoystickCal>,
    y_pin: AdcPin<N, ADC1, JoystickCal>,
}
use comunication::{Angle, Percent};

impl<'a, O, N> Joystick<O, N>
where
    O: Peripheral<P = O> + AnalogPin + AdcChannel + 'a,
    N: Peripheral<P = N> + AnalogPin + AdcChannel + 'a,
{
    pub fn new(x_pin: O, y_pin: N, adc: ADC1) -> (Self, Reader<'a, O, N>) {
        let mut adc_config = AdcConfig::new();
        let mut adc0_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(x_pin, Attenuation::Attenuation11dB);
        let mut adc1_pin = adc_config.enable_pin_with_cal::<_, JoystickCal>(y_pin, Attenuation::Attenuation11dB);
        let mut adc = Adc::new(adc, adc_config);

        (Joystick::<O,N>{x_pin: adc0_pin, y_pin: adc1_pin}, Reader{adc, _phantom_o: PhantomData,
                _phantom_n: PhantomData,})
    }

    fn map_value<T>(input: u16, max_abs: T, min_abs: T) -> T
    where
        T: Copy
            + Sub<Output = T>
            + Neg<Output = T>
            + Add<T, Output = T>
            + Mul<T, Output = T>
            + Div<T, Output = T>
            + From<u16>
            + core::cmp::PartialOrd
    {
        const LOW_RANGE_MAX: u16 = 500;
        const HIGH_RANGE_MIN: u16 = 700;
        const LOW_RANGE_MIN: u16 = 0;
        const HIGH_RANGE_MAX: u16 = 1550;
        
        let input_t = T::from(input);
        let low_range_max_t = T::from(LOW_RANGE_MAX);
        let high_range_min_t = T::from(HIGH_RANGE_MIN);
        let low_range_min_t = T::from(LOW_RANGE_MIN);
        let high_range_max_t = T::from(HIGH_RANGE_MAX);
    
        let range_output = max_abs - min_abs;
    
        if input_t < low_range_max_t {
            -min_abs + (-range_output * -(input_t - low_range_max_t) / low_range_max_t)
        } else if input_t > high_range_min_t && input_t < high_range_max_t {
            min_abs + (range_output * (input_t - high_range_min_t) / (high_range_max_t - high_range_min_t))
        } else if input_t > high_range_max_t {
            max_abs
        } else {
            T::from(0u16)
        }
    }

    pub fn convert(measurements: (u16, u16)) -> (Option<Angle>, Option<Percent>) {
        let (angle, power) = measurements;
        let mut angle = Self::map_value(angle, 30i32, 0i32);
        let power = -1.0f32 * Self::map_value(power, 100.0f32, 100.0f32);
        if angle > i8::MAX.into() {
            angle = 0;
        }
        (Angle::new(angle.try_into().unwrap()), Percent::new(power))
    }
}

pub struct Reader<'a, O, N>
where
    O: Peripheral<P = O> + AnalogPin + AdcChannel + 'a,
    N: Peripheral<P = N> + AnalogPin + AdcChannel + 'a,
{
    adc: Adc<'a, ADC1>,
    _phantom_o: PhantomData<&'a O>,
    _phantom_n: PhantomData<&'a N>,
}

#[derive(Debug)]
pub enum Error {
    Read,
}

impl<'a, O, N> Reader<'a, O, N>
where
    O: Peripheral<P = O> + AnalogPin + AdcChannel + 'a,
    N: Peripheral<P = N> + AnalogPin + AdcChannel + 'a,
{
    pub fn read(&mut self, joystick: &mut Joystick<O, N>) -> Result<(u16, u16), Error> {
        let x_axis = (nb::block!(self.adc.read_oneshot(&mut joystick.x_pin))).map_err(|_| Error::Read)?;
        let y_axis = (nb::block!(self.adc.read_oneshot(&mut joystick.y_pin))).map_err(|_| Error::Read)?;
        Ok((x_axis, y_axis))
    }
}

