#![no_main]
#![no_std]

use vexide::prelude::*;

struct Robot {
    left_motor_front: Motor,
    left_motor_fmid: Motor,
    left_motor_bmid: Motor,
    left_motor_back: Motor,

    right_motor_front: Motor,
    right_motor_fmid: Motor,
    right_motor_bmid: Motor,
    right_motor_back: Motor,

    score_mech_left: Motor,
    score_mech_right: Motor,

    intake_left: Motor,
    intake_right: Motor,

    inertial: InertialSensor,
   
    controller: Controller,
}


impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            let forward = state.left_stick.y();
            let turn = state.right_stick.x();

            let lside = forward + turn;
            let rside = forward - turn;

            self.inertial.heading().unwrap_or_default();


            self.left_drive(lside * 12.0);
            self.right_drive(rside * 12.0);

            if state.button_l1.is_pressed() {
                self.intake(12.0);
            } else if state.button_l2.is_pressed() {
                self.intake(-12.0);
            } else {
                self.intake(0.0);
            }

            sleep(core::time::Duration::from_millis(20)).await;
        }

    }
}

impl Robot {
    fn left_drive(&mut self, power: f64) {
        self.left_motor_front.set_voltage(power).ok();
        self.left_motor_fmid.set_voltage(power).ok();
        self.left_motor_bmid.set_voltage(power).ok();
        self.left_motor_back.set_voltage(power).ok();
    }

    fn right_drive(&mut self, power: f64) {
        self.right_motor_front.set_voltage(power).ok();
        self.right_motor_fmid.set_voltage(power).ok();
        self.right_motor_bmid.set_voltage(power).ok();
        self.right_motor_back.set_voltage(power).ok();
    }

    fn intake(&mut self, power: f64) {
        self.intake_left.set_voltage(power).ok();
        self.intake_right.set_voltage(power).ok();
        self.score_mech_left.set_voltage(power).ok();
        self.score_mech_right.set_voltage(power).ok();
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        left_motor_front: Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
        left_motor_fmid: Motor::new(peripherals.port_13, Gearset::Blue, Direction::Forward),
        left_motor_bmid: Motor::new(peripherals.port_14, Gearset::Blue, Direction::Reverse),
        left_motor_back: Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),

        right_motor_front: Motor::new(peripherals.port_2, Gearset::Blue, Direction::Forward),
        right_motor_fmid: Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        right_motor_bmid: Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        right_motor_back: Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),

        score_mech_left: Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        score_mech_right: Motor::new(peripherals.port_16, Gearset::Blue, Direction::Forward),

        intake_left: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
        intake_right: Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),

        inertial: InertialSensor::new(peripherals.port_8),

        controller: peripherals.primary_controller,
    };

    robot.compete().await;
}
