use std::{fmt::Display, time::Duration};

pub struct ControlValue {
    pub value: f64,
}

pub struct ServoInput {
    pub process_value: f64,
    pub delta_t: Option<Duration>,
}

#[derive(Debug)]
pub struct ReadInputError {
    message: String,
}

impl ReadInputError {
    pub fn new(message: &str) -> Self {
        Self {
            message: message.to_string(),
        }
    }
}

impl std::fmt::Display for ReadInputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for ReadInputError {}

pub trait Servo: Display {
    fn read(&mut self, servo_input: &ServoInput) -> Result<ControlValue, ReadInputError>;
}
