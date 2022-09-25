use engine::Engine;
use pid::pid_controller::PidController;

pub mod engine;
pub mod pid;
pub mod servo;

// Dummy interface controller that measures an external process variable.
struct InterfaceController {
    out_amp: f64,
}
impl InterfaceController {
    fn new() -> Self {
        Self { out_amp: 0.0 }
    }
    fn read(&self) -> f64 {
        0.0
    }

    fn set(&mut self, amp: f64) {
        self.out_amp = amp;
    }
}
// Example invocation of a PID controller
#[allow(dead_code)]
fn main() {
    let goal_set_point = 100.0;
    let pid_controller = PidController::new(goal_set_point);

    // Prepare reader
    let mut controller = InterfaceController::new();
    // Spin up a servo engine to do our heavy lifting
    let mut engine = Engine::new(pid_controller);
    loop {
        // Read in the current process value
        let val = controller.read();

        // Calculate the control output based on the current process value
        let control_output = engine.next(val);

        // Adjust the controller
        controller.set(val + control_output.value);

        // Wait one second before re-reading
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}
