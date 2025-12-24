#![feature(future_join)]

use std::{cell::RefCell, ffi::CString, fmt::Write, future::join, ops::Deref, rc::Rc};

use drivetrain::Drivetrain;
use localization::{Pose, odom_thread};
use vexide::{
    color::Color,
    display::{Font, FontFamily, FontSize, Text},
    math::Point2,
    prelude::*,
};

#[allow(non_snake_case)]
pub mod PID;
pub mod drivetrain;
pub mod localization;
pub mod utils;

struct Robot {
    display: Display,
    drivetrain: Drivetrain,

    score_mech_left: Motor,
    score_mech_right: Motor,

    intake_left: Motor,
    intake_right: Motor,

    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        loop {
            {
                self.display.erase(Color::BLACK);
                let pose = self.drivetrain.pose.borrow();
                let text = Text::from_string(
                    format!("X: {:.2}, Y: {:.2}, R: {:.2}", pose.x, pose.y, pose.rot),
                    Font::default(),
                    [10, 10],
                );
                self.display.draw_text(&text, Color::WHITE, None);
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }

        // loop {
        //     let state = self.controller.state().unwrap_or_default();

        //     let lside = state.left_stick.y();
        //     let rside = state.right_stick.y();

        //     // println!("{}", lside * 12.0);

        //     self.drivetrain.left_drive(lside * 12.0);
        //     self.drivetrain.right_drive(rside * 12.0);

        //     if state.button_l1.is_pressed() {
        //         self.intake(12.0);
        //     } else if state.button_l2.is_pressed() {
        //         self.intake(-12.0);
        //     } else {
        //         self.intake(0.0);
        //     }

        //     sleep(Controller::UPDATE_INTERVAL).await;
        // }
    }
}

impl Robot {
    fn intake(&mut self, power: f64) {
        self.intake_left.set_voltage(power).ok();
        self.intake_right.set_voltage(power).ok();
        self.score_mech_left.set_voltage(power).ok();
        self.score_mech_right.set_voltage(power).ok();
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let pose = Rc::new(RefCell::new(Pose {
        x: 0.0,
        y: 0.0,
        rot: 0.0,
    }));

    let robot = Robot {
        drivetrain: Drivetrain {
            left_motor_1: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
            left_motor_2: Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
            left_motor_3: Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
            left_motor_4: Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),

            right_motor_1: Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),
            right_motor_2: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
            right_motor_3: Motor::new(peripherals.port_19, Gearset::Blue, Direction::Forward),
            right_motor_4: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),

            pose: pose.clone(),
        },

        display: peripherals.display,

        score_mech_left: Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        score_mech_right: Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),

        intake_left: Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),
        intake_right: Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),

        controller: peripherals.primary_controller,
    };

    let mut imu_1 = InertialSensor::new(peripherals.port_14);
    let mut imu_2 = InertialSensor::new(peripherals.port_17);

    let tracking_wheel_1 = RotationSensor::new(peripherals.port_13, Direction::Forward);
    let tracking_wheel_2 = RotationSensor::new(peripherals.port_18, Direction::Forward);

    let _calib = join!(imu_1.calibrate(), imu_2.calibrate()).await;

    let _odom = spawn(odom_thread(
        imu_1,
        imu_2,
        tracking_wheel_1,
        tracking_wheel_2,
        pose,
    ));

    robot.compete().await;
}
