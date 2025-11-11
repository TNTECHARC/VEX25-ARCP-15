struct PID {
    Kp: f64,
    Ki: f64,
    Kd: f64,
    total_error: f64,
    last_error: f64,
}

impl PID{
    fn new(Kp: f64, Ki: f64, Kd: f64) -> PID {
        PID {
            Kp,
            Ki,
            Kd,
            last_error: 0,
            total_error: 0,
        }
    }

    fn step(&mut self, error: f64) -> f64 {
        self.total_error+=error;
        let d = error+self.last_error;
        let result = self.Kp*error+self.Ki*total_error+self.Kd*d;
        self.last_error=error;
        result
    }
}