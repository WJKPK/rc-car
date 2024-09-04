use core::f32::consts::PI;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct RCLowPassFilter {
    alpha: f32,
    output: f32,
}

impl RCLowPassFilter {
    pub fn new(sample_rate: f32, cutoff_frequency: f32) -> Self {
        let rc = 1.0 / (2.0 * PI * cutoff_frequency);
        let dt = 1.0 / sample_rate;
        let alpha = dt / (rc + dt);
        
        RCLowPassFilter {
            alpha,
            output: 0.0,
        }
    }

    pub fn process(&mut self, input: f32) -> f32 {
        self.output += self.alpha * (input - self.output);
        self.output
    }
}

pub trait FilterAdaptor {
    fn process(&mut self, input: f32) -> f32;
}

impl FilterAdaptor for RCLowPassFilter {
    fn process(&mut self, input: f32) -> f32 {
        self.process(input)
    }
}


