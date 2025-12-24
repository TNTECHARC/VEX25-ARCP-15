pub struct PID {
    Kp: f64,
    Ki: f64,
    Kd: f64,
    total_error: f64,
    last_error: f64,
}

impl PID {
    pub fn new(Kp: f64, Ki: f64, Kd: f64) -> PID {
        PID {
            Kp,
            Ki,
            Kd,
            total_error: 0.0,
            last_error: 0.0,
        }
    }

    pub fn step(&mut self, error: f64, delta: f64) -> f64 {
        // Integral
        self.total_error += error * delta;

        // println!("Error{}, TError: {}", error, self.total_error);

        // Derivative
        let d = if delta > 0.0 {
            self.Kd * (error - self.last_error) / delta
        } else {
            0.0
        };

        // Proportional
        let p = self.Kp * error;

        // Update last error
        self.last_error = error;

        p + self.Ki * self.total_error + d
    }
}
