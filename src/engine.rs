use std::{fmt::Display, time::Instant};

use num_traits::Num;

use crate::servo::{ControlValue, Servo, ServoInput};

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

    pub fn next<N: Num>(&mut self, process_value: N) -> ControlValue {
        let servo_input = ServoInput {
            process_value,
            delta_t: None,
        };
        let servo_result = self.servo.read(&servo_input);

        self.num_reads += 1;
        servo_result.unwrap()
    }
}

#[cfg(test)]
mod tests {
    use std::fmt::Display;

    use super::Engine;
    use crate::servo::{ControlValue, Servo};

    struct TestController {}

    impl TestController {
        fn new() -> Self {
            Self {}
        }
    }

    impl Servo for TestController {
        fn read<T: num_traits::Num>(
            &mut self,
            _servo_input: &crate::servo::ServoInput<T>,
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
        engine.next(1);
        engine.next(2);
        engine.next(3);
        assert_eq!(engine.num_reads, 3);
    }

    #[test]
    fn should_reset_engine_correctly() {
        let servo = TestController::new();
        let mut engine = Engine::new(servo);
        for _ in 0..5 {
            engine.next(0);
        }
        assert_eq!(engine.num_reads, 5);
        engine.reset();
        assert_eq!(engine.num_reads, 0);
    }
}
