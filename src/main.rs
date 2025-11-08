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

            self.left_drive(lside * 12.0);
            self.right_drive(rside * 12.0);

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
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {
        left_motor_front: Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
        left_motor_fmid: Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        left_motor_bmid: Motor::new(peripherals.port_4, Gearset::Blue, Direction::Forward),
        left_motor_back: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
        right_motor_front: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),
        right_motor_fmid: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
        right_motor_bmid: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
        right_motor_back: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
        controller: peripherals.primary_controller,
    };

    robot.compete().await;
}
