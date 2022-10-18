use engine::{Engine, Measurement};
use pid::pid_controller::PidController;
use std::{fs::File, time::Instant};

use rand_distr::Distribution;
use rand_distr::Normal;
use std::io::Write;

pub mod engine;
pub mod pid;
pub mod servo;

// Example invocation of a PID controller
#[allow(dead_code)]
fn main() -> Result<(), std::io::Error> {
    // Capture data to data.txt
    let mut file = File::create("data.txt")?;
    let mut controller = PidController::new(200.0);

    controller.set_derivative_term(0.19);
    controller.set_integral_term(0.1);
    controller.set_proportional_term(0.1);

    let mut engine = Engine::new(controller);
    let mut measurement = Measurement::new();

    // Mock a first measurement
    measurement.set_value(50.0, Instant::now());

    for i in 0..900 {
        std::thread::sleep(std::time::Duration::from_millis(250));
        let output = engine.next(&measurement);
        match output {
            Ok(control_val) => {
                let val = control_val.value;
                let mut rng = rand::thread_rng();
                let normal = Normal::new(10.0, 5.0).unwrap();
                let noise = normal.sample(&mut rng);
                measurement.set_value(measurement.value + val + noise, Instant::now());
                write!(file, "{}\t{}\n", i, measurement.value)?;
            }
            Err(err) => eprintln!("{err}"),
        }
    }
    Ok(())
}
