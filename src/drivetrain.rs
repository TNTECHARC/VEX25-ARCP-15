use std::{cell::RefCell, rc::Rc, time::Instant};

use vexide::{prelude::*, smart::motor::BrakeMode};

use crate::{PID::PID, localization::Pose, utils::degree_wrap};

pub struct Drivetrain {
    pub left_motor_1: Motor,
    pub left_motor_2: Motor,
    pub left_motor_3: Motor,
    pub left_motor_4: Motor,

    pub right_motor_1: Motor,
    pub right_motor_2: Motor,
    pub right_motor_3: Motor,
    pub right_motor_4: Motor,

    pub pose: Rc<RefCell<Pose>>,
}

impl Drivetrain {
    pub async fn spin_to_angle(&mut self, target_angle: f64) {
        let mut turn_pid = PID::new(24.0, 0.0, 0.0);
        let rot = self.pose.borrow().rot;

        let mut last_loop = Instant::now();
        let start_time = Instant::now();

        let mut settling = 0;

        loop {
            let current_angle = self.pose.borrow().rot;

            let now = Instant::now();
            let delta = now.duration_since(last_loop).as_secs_f64();

            let error = degree_wrap(target_angle - current_angle);

            if f64::abs(error) <= 0.5 {
                if settling >= 200 {
                    self.brake(BrakeMode::Hold);
                    break;
                }

                settling += 20;
            } else {
                settling = 0;
            }

            let spin = turn_pid.step(error / 360.0, delta).clamp(-12.0, 12.0);

            println!("Error: {}, Spin: {}, Rot: {}", error, spin, current_angle);

            self.left_drive(spin);
            self.right_drive(-spin);

            last_loop = now;

            if now.duration_since(start_time).as_secs() >= 5 {
                self.brake(BrakeMode::Hold);
                println!("Spin to Rot: TIMEOUT KILL");
                break;
            }

            sleep(InertialSensor::UPDATE_INTERVAL).await;
        }
    }

    pub fn left_drive(&mut self, power: f64) {
        self.left_motor_1.set_voltage(power).ok();
        self.left_motor_2.set_voltage(power).ok();
        self.left_motor_3.set_voltage(power).ok();
        self.left_motor_4.set_voltage(power).ok();
    }

    pub fn right_drive(&mut self, power: f64) {
        self.right_motor_1.set_voltage(power).ok();
        self.right_motor_2.set_voltage(power).ok();
        self.right_motor_3.set_voltage(power).ok();
        self.right_motor_4.set_voltage(power).ok();
    }

    pub fn brake(&mut self, brake_type: BrakeMode) {
        self.left_motor_1.brake(brake_type).ok();
        self.left_motor_2.brake(brake_type).ok();
        self.left_motor_3.brake(brake_type).ok();
        self.left_motor_4.brake(brake_type).ok();
        self.right_motor_1.brake(brake_type).ok();
        self.right_motor_2.brake(brake_type).ok();
        self.right_motor_3.brake(brake_type).ok();
        self.right_motor_4.brake(brake_type).ok();
    }
}
