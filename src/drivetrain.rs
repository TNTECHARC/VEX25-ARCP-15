use std::{cell::RefCell, rc::Rc, time::Instant};

use vexide::{math::Angle, prelude::*, smart::motor::BrakeMode, time::LowResolutionTime};

use crate::{PID::PID, localization::Pose};

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
        let _rot = self.pose.borrow().rot;

        let mut last_loop = Instant::now();
        let start_time = Instant::now();

        let mut settling = 0;
        let target_angle_angle = Angle::from_degrees(target_angle);

        loop {
            let current_angle = self.pose.borrow().rot;

            let now = Instant::now();
            let delta = now.duration_since(last_loop).as_secs_f64();

            let error = (target_angle_angle - current_angle)
                .wrapped_half()
                .as_degrees();

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

            println!(
                "Error: {}, Spin: {}, Rot: {}",
                error,
                spin,
                current_angle.as_degrees()
            );

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

    pub fn motor_velocity(
        &self,
        motor: &Motor,
        timestamp: &mut LowResolutionTime,
        pos: &mut Angle,
    ) -> f64 {
        let new_angle = motor.position().unwrap_or_default();
        let new_time = motor.timestamp().unwrap_or(LowResolutionTime::now());

        let delta_angle = (new_angle - *pos).as_turns();
        let delta_time = new_time.duration_since(*timestamp).as_secs_f64();

        *pos = new_angle;
        *timestamp = new_time;

        if delta_time > 0.0 {
            delta_angle / delta_time
        } else {
            0.0
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
