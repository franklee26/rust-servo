use engine::{Engine, Measurement};
use pid::pid_controller::PidController;
use std::{fs::File, time::Instant};

use rand_distr::Distribution;
use rand_distr::Normal;
use std::io::Write;

pub mod engine;
pub mod pid;
pub mod servo;

#[allow(dead_code)]
fn main() -> Result<(), std::io::Error> {
    // Capture data to data.txt
    let mut file = File::create("data.txt")?;
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

    for i in 0..10 {
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
