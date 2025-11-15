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
            last_error: 0.0,
            total_error: 0.0,
        }
    }

    pub fn step(&mut self, error: f64) -> f64 {
        self.total_error += error;

        let p = error * self.Kp;
        let d = (error + self.last_error) * self.Kd;

        self.last_error = error;

        p + self.Ki * self.total_error + d
    }
}
