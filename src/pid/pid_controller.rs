use std::fmt::Display;

use num_traits::Num;

use crate::servo::{Servo, ServoInput};

pub struct PidController {
    timestamp: f64,
    set_point: f64,
    last_error_term: Option<f64>,
    proportional: f64,
    integral: f64,
    derivative: f64,
}

impl PidController {
    pub fn new(set_point: f64) -> Self {
        Self {
            timestamp: 0.0,
            set_point: set_point,
            last_error_term: None,
            proportional: 0.0,
            integral: 0.0,
            derivative: 0.0,
        }
    }
}

impl Servo for PidController {
    fn read<T: Num>(
        &mut self,
        servo_input: &ServoInput<T>,
    ) -> Result<crate::servo::ControlValue, crate::servo::ReadInputError> {
        todo!();
    }
}

impl Display for PidController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", std::any::type_name::<Self>())
    }
}
