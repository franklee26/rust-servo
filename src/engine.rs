use std::{fmt::Display, time::Instant};

use crate::servo::{ControlValue, ReadInputError, Servo, ServoInput};

pub struct Engine<T: Servo> {
    pub servo: T,
    pub last_read_timestamp: Option<Instant>,
    pub num_reads: u64,
}

impl<T: Servo + Display> Display for Engine<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Engine runner [{}] has {} total read(s)",
            self.servo, self.num_reads
        )
    }
}

pub struct Measurement {
    pub value: f64,
    pub timestamp: Option<Instant>,
}

impl Measurement {
    pub fn new() -> Self {
        Self {
            value: 0.0,
            timestamp: None,
        }
    }

    pub fn set_value(&mut self, value: f64, timestamp: Instant) {
        self.value = value;
        self.timestamp = Some(timestamp);
    }
}

impl<T: Servo> Engine<T> {
    pub fn new(servo: T) -> Self {
        Self {
            servo,
            last_read_timestamp: None,
            num_reads: 0,
        }
    }

    pub fn reset(&mut self) {
        self.last_read_timestamp = None;
        self.num_reads = 0;
    }

    pub fn next(&mut self, measurement: &Measurement) -> Result<ControlValue, ReadInputError> {
        let timestamp = measurement
            .timestamp
            .ok_or_else(|| ReadInputError::new("Measurement needs a timestamp"))?;
        let mut delta_t = None;
        if let Some(previous_timestamp) = self.last_read_timestamp {
            // If there was a previous measurement timestamp, then ensure that the new one comes after the previous
            if previous_timestamp >= timestamp {
                return Err(ReadInputError::new("Failed to execute engine on new measurement that is older than the previous measurement."));
            }
            delta_t = Some(timestamp.duration_since(previous_timestamp));
        }
        let servo_input = ServoInput {
            process_value: measurement.value,
            delta_t,
        };
        let servo_result = self.servo.read(&servo_input);

        self.num_reads += 1;
        self.last_read_timestamp = Some(timestamp);
        servo_result
    }
}

#[cfg(test)]
mod tests {
    use std::{fmt::Display, time::Instant};

    use super::Engine;
    use crate::{
        engine::Measurement,
        servo::{ControlValue, Servo},
    };

    struct TestController {}

    impl TestController {
        fn new() -> Self {
            Self {}
        }
    }

    impl Servo for TestController {
        fn read(
            &mut self,
            _servo_input: &crate::servo::ServoInput,
        ) -> Result<crate::servo::ControlValue, crate::servo::ReadInputError> {
            Ok(ControlValue { value: 0.0 })
        }
    }

    impl Display for TestController {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(f, "")
        }
    }

    #[test]
    fn should_instantiate_engine_correctly() {
        let servo = TestController::new();
        let engine = Engine::new(servo);
        assert!(engine.last_read_timestamp.is_none());
        assert_eq!(engine.num_reads, 0);
    }

    #[test]
    fn should_invoke_servo_engine_correctly() {
        let servo = TestController::new();
        let mut engine = Engine::new(servo);
        let mut measurement = Measurement::new();
        measurement.set_value(100.0, Instant::now());
        let _ = engine.next(&measurement);
        assert_eq!(engine.num_reads, 1);
    }

    #[test]
    fn should_invoke_and_reset_engine_correctly() {
        let servo = TestController::new();
        let mut engine = Engine::new(servo);
        let mut measurement = Measurement::new();
        let mut previous_timestamp = Instant::now();
        measurement.set_value(1.1, previous_timestamp);
        for _ in 0..5 {
            let _ = engine.next(&measurement);
            assert_eq!(engine.last_read_timestamp.unwrap(), previous_timestamp);
            previous_timestamp = Instant::now();
            measurement.set_value(1.0, previous_timestamp);
        }
        assert_eq!(engine.num_reads, 5);
        engine.reset();
        assert_eq!(engine.num_reads, 0);
    }

    #[test]
    fn engine_should_err_if_new_measurement_is_older_than_previous() {
        let servo = TestController::new();
        let mut engine = Engine::new(servo);

        let older = Instant::now();
        let newer = older.checked_add(std::time::Duration::from_secs(1));

        engine.last_read_timestamp = newer;

        let mut measurement = Measurement::new();
        measurement.set_value(1.0, older);
        match engine.next(&measurement) {
            Ok(_) => unreachable!(),
            Err(e) => assert_eq!(e.to_string(), "Failed to execute engine on new measurement that is older than the previous measurement.")
        }
    }
}
