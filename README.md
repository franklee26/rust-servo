# rust-servo 

`rust-servo` is a Rust implementation of various servomechanism algorithms. 

[![Latest Version]][crates.io] 
[![Documentation]][docs.rs]

## Example PID controller

```rust
use engine::{Engine, Measurement};
use pid::pid_controller::PidController;
use std::time::Instant;

use rand_distr::Distribution;
use rand_distr::Normal;
use std::io::Write;

pub mod engine;
pub mod pid;
pub mod servo;

// Example invocation of a PID controller
fn main() -> Result<(), std::io::Error> {
    // Setpoint set to 200.0 with arbitrary coefficients
    let controller = PidController::builder()
        .set_point(200.0)
        .derivative(0.19)
        .integral(0.1)
        .proportional(0.1)
        .build();

    let mut engine = Engine::new(controller);
    let mut measurement = Measurement::new();

    // Mock a first measurement
    measurement.set_value(50.0, Instant::now());

    loop {
        std::thread::sleep(std::time::Duration::from_millis(250));

        let output = engine.next(&measurement);
        match output {
            Ok(control_val) => {
                let val = control_val.value;

                // Arbitrarily add some gaussian noise to measurement
                let mut rng = rand::thread_rng();
                let normal = Normal::new(10.0, 5.0).unwrap();
                let noise = normal.sample(&mut rng);
                measurement.set_value(measurement.value + val + noise, Instant::now());
            }
            Err(err) => eprintln!("{err}"),
        }
    }
}
```

[Latest Version]: https://img.shields.io/crates/v/rust-servo.svg
[crates.io]: https://crates.io/crates/rust-servo
[Documentation]: https://docs.rs/rust-servo/badge.svg
[docs.rs]: https://docs.rs/rust-servo