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

pub struct PidControllerBuilder {
    set_point: Option<f64>,
    proportional: Option<f64>,
    integral: Option<f64>,
    derivative: Option<f64>,
}

impl PidControllerBuilder {
    fn new() -> Self {
        Self {
            set_point: None,
            proportional: None,
            integral: None,
            derivative: None,
        }
    }

    pub fn set_point(mut self, set_point: f64) -> Self {
        self.set_point = Some(set_point);
        self
    }

    pub fn proportional(mut self, proportional: f64) -> Self {
        self.proportional = Some(proportional);
        self
    }

    pub fn integral(mut self, integral: f64) -> Self {
        self.integral = Some(integral);
        self
    }

    pub fn derivative(mut self, derivative: f64) -> Self {
        self.derivative = Some(derivative);
        self
    }

    pub fn build(self) -> PidController {
        let Self {
            set_point,
            proportional,
            integral,
            derivative,
        } = self;
        PidController {
            set_point: set_point.unwrap_or_default(),
            last_control_value: None,
            last_error_term: None,
            last_last_error_term: None,
            proportional: proportional.unwrap_or_default(),
            integral: integral.unwrap_or_default(),
            derivative: derivative.unwrap_or_default(),
        }
    }
}

impl PidController {
    pub fn builder() -> PidControllerBuilder {
        PidControllerBuilder::new()
    }

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

    #[inline]
    pub fn set_proportional_term(&mut self, proportional: f64) {
        self.proportional = proportional;
    }

    #[inline]
    pub fn set_integral_term(&mut self, integral: f64) {
        self.integral = integral;
    }

    #[inline]
    pub fn set_derivative_term(&mut self, derivative: f64) {
        self.derivative = derivative;
    }

    #[inline]
    fn backup_error_and_control_values(&mut self, value: f64, err_tk: f64) {
        self.last_control_value = Some(value);
        self.last_last_error_term = self.last_error_term;
        self.last_error_term = Some(err_tk);
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

            self.backup_error_and_control_values(value, err_tk);
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
        self.backup_error_and_control_values(value, err_tk);

        let control_value = ControlValue { value };

        Ok(control_value)
    }
}

impl Display for PidController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", std::any::type_name::<Self>())
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use crate::servo::Servo;
    use crate::servo::ServoInput;

    use super::PidController;

    macro_rules! assert_control_value_result_eq {
        ($result: expr, $expected: expr) => {
            match $result {
                Ok(value) => assert_eq!(value.value, $expected),
                Err(_) => assert!(false),
            }
        };
    }

    // Borrowed from braincore/pid-rs example: https://github.com/braincore/pid-rs#example
    #[test]
    fn pid_controller_accepts_valid_inputs_as_expected() {
        let one_sec = Duration::from_secs(1);
        let mut test_servo_input = ServoInput {
            process_value: 10.0,
            delta_t: Some(one_sec),
        };
        let mut pid_controller = PidController::builder().set_point(15.0).build();

        pid_controller.set_proportional_term(10.0);
        let mut output = pid_controller.read(&test_servo_input);
        // Test that output is solely weighted by prop term where output = k_p * err = 10.0 * (15.0 - 10.0) = 50.0
        assert_control_value_result_eq!(output, 50.0);

        // Introduce integral term
        pid_controller.set_integral_term(1.0);
        output = pid_controller.read(&test_servo_input);
        assert_control_value_result_eq!(output, 55.0);

        // Finally, set derivative term
        pid_controller.set_derivative_term(2.0);
        test_servo_input.process_value = 15.0;
        output = pid_controller.read(&test_servo_input);
        assert_control_value_result_eq!(output, -5.0);
    }

    #[test]
    fn pid_controller_errors_on_zero_delta_t_measurements() {
        let zero_sec = Duration::from_secs(0);
        let test_servo_input = ServoInput {
            process_value: 1.0,
            delta_t: Some(zero_sec),
        };
        let mut pid_controller = PidController::builder().set_point(15.0).build();
        let output = pid_controller.read(&test_servo_input);
        match output {
            Ok(_) => unreachable!(),
            Err(e) => assert_eq!(
                e.to_string(),
                "Delta t cannot be zero for discrete PID approximation."
            ),
        }
    }
}
