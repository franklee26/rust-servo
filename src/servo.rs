use std::fmt::Display;

use num_traits::Num;

pub struct ControlValue {
    pub value: f64,
}

pub struct ServoInput<T: Num> {
    pub process_value: T,
    pub delta_t: Option<u64>,
}

#[derive(Debug)]
pub struct ReadInputError {}

impl std::fmt::Display for ReadInputError {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        todo!()
    }
}

impl std::error::Error for ReadInputError {}

pub trait Servo: Display {
    fn read<T: Num>(&mut self, servo_input: &ServoInput<T>)
        -> Result<ControlValue, ReadInputError>;
}
