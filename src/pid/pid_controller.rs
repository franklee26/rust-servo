use std::fmt::Display;

use num_traits::Zero;

use crate::servo::{ControlValue, ReadInputError, Servo, ServoInput};

pub struct PidController {
    set_point: f64,
    last_control_value: Option<f64>,
    last_error_term: Option<f64>,
    last_last_error_term: Option<f64>,
    proportional: f64,
    integral: f64,
    derivative: f64,
}

impl PidController {
    pub fn new(set_point: f64) -> Self {
        Self {
            set_point,
            last_control_value: None,
            last_error_term: None,
            last_last_error_term: None,
            proportional: 0.0,
            integral: 0.0,
            derivative: 0.0,
        }
    }

    pub fn set_proportional_term(&mut self, proportional: f64) {
        self.proportional = proportional;
    }

    pub fn set_integral_term(&mut self, integral: f64) {
        self.integral = integral;
    }

    pub fn set_derivative_term(&mut self, derivative: f64) {
        self.derivative = derivative;
    }
}

impl Servo for PidController {
    fn read(
        &mut self,
        servo_input: &ServoInput,
    ) -> Result<crate::servo::ControlValue, crate::servo::ReadInputError> {
        if servo_input.delta_t.is_none() {
            // This means that there are no past measurements.
            // Only use proportional output
            let err_tk = self.set_point - servo_input.process_value;
            let value = self.proportional * err_tk;
            let control_value = ControlValue { value };

            self.last_control_value = Some(value);
            self.last_last_error_term = self.last_error_term;
            self.last_error_term = Some(err_tk);
            return Ok(control_value);
        }
        let delta_t = servo_input.delta_t.unwrap().as_secs_f64();
        if delta_t.is_zero() {
            return Err(ReadInputError::new(
                "Delta t cannot be zero for discrete PID approximation.",
            ));
        }

        let err_tk = self.set_point - servo_input.process_value;
        let err_tk_1 = self.last_error_term.unwrap_or(0.0);
        let err_tk_2 = self.last_last_error_term.unwrap_or(0.0);
        let u_tk_1 = self.last_control_value.unwrap_or(0.0);

        let value = u_tk_1
            + err_tk * (self.proportional + self.integral * delta_t + self.derivative / delta_t)
            + err_tk_1 * (-self.proportional - 2.0 * self.derivative / delta_t)
            + err_tk_2 * (self.derivative / delta_t);

        // Store error and output value terms for the next calculation
        self.last_control_value = Some(value);
        self.last_last_error_term = self.last_error_term;
        self.last_error_term = Some(err_tk);

        let control_value = ControlValue { value };

        Ok(control_value)
    }
}

impl Display for PidController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", std::any::type_name::<Self>())
    }
}
