#![feature(future_join)]

use std::{
    cell::RefCell,
    fs::OpenOptions,
    future::join,
    io::{BufWriter, Write},
    rc::Rc,
    time::Instant,
};

use drivetrain::Drivetrain;
use localization::{Pose, odom_thread};
use vexide::{
    color::Color,
    display::{Font, Text},
    math::Angle,
    prelude::*,
};

#[allow(non_snake_case)]
pub mod PID;
pub mod drivetrain;
pub mod localization;
pub mod utils;

const TRENT: bool = false;
const BLACK: bool = false;

struct Robot {
    display: Display,
    drivetrain: Drivetrain,

    score_mech_wheel: Motor,
    score_mech_top: Motor,

    intake_left: Motor,
    intake_right: Motor,

    lift: AdiDigitalOut,
    matchload: AdiDigitalOut,
    flap: AdiDigitalOut,

    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();

            let lside;
            let rside;

            if TRENT {
                lside = state.left_stick.y();
                rside = state.right_stick.y();
            } else {
                let forward = state.left_stick.y();
                let turn = state.right_stick.x();

                lside = forward + turn;
                rside = forward - turn;
            }

            self.drivetrain.left_drive(lside * 12.0);
            self.drivetrain.right_drive(rside * 12.0);

            if TRENT {
                if state.button_r1.is_pressed() {
                    self.intake(12.0);
                } else if state.button_r2.is_pressed() {
                    self.intake(-12.0);
                } else {
                    self.intake(0.0);
                }

                if state.button_l1.is_pressed() {
                    self.score_mech(-12.0);
                } else if state.button_l2.is_pressed() {
                    self.score_mech(12.0);
                } else {
                    self.score_mech(0.0);
                }
            } else {
                if state.button_l1.is_pressed() {
                    self.intake(12.0);
                } else if state.button_l2.is_pressed() {
                    self.intake(-12.0);
                } else {
                    self.intake(0.0);
                }

                if state.button_r1.is_pressed() {
                    self.score_mech(-12.0);
                } else if state.button_r2.is_pressed() {
                    self.score_mech(12.0);
                } else {
                    self.score_mech(0.0);
                }
            }

            if state.button_x.is_now_pressed() {
                self.flap.toggle().ok();
            }

            if state.button_a.is_now_pressed() {
                self.matchload.set_low().ok();
            }

            if state.button_b.is_now_pressed() {
                self.matchload.set_high().ok();
            }

            if state.button_up.is_now_pressed() {
                self.lift.set_high().ok();
            }

            if state.button_down.is_now_pressed() {
                self.lift.set_low().ok();
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

impl Robot {
    fn intake(&mut self, power: f64) {
        self.intake_left.set_voltage(power).ok();
        self.intake_right.set_voltage(power).ok();
        self.score_mech_wheel.set_voltage(power).ok();
    }

    fn score_mech(&mut self, power: f64) {
        self.score_mech_top.set_voltage(power).ok();
    }

    fn log_data(&mut self, data: String) {
        // Open file in append mode, create if doesn't exist
        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open("velocity_data.txt")
            .expect("Unable to open velocity_data.txt");

        // Write data as one line with timestamp
        writeln!(file, "{}", data).expect("Unable to write to file");
    }

    fn display_text(&mut self, text: String, pos: [i16; 2]) {
        self.display.erase(Color::BLACK);

        // Split text by lines and render each line separately
        let lines: Vec<&str> = text.lines().collect();
        for (i, line) in lines.iter().enumerate() {
            let y_offset = [pos[0], pos[1] + (i as i16 * 16)]; // 16 pixels per line
            let text_line = Text::from_string(line.to_string(), Font::default(), y_offset);
            self.display.draw_text(&text_line, Color::WHITE, None);
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let pose = Rc::new(RefCell::new(Pose {
        x: 0.0,
        y: 0.0,
        rot: Angle::ZERO,
        timestamp: Instant::now(),
    }));

    // Black
    let robot = if BLACK {
        Robot {
            drivetrain: Drivetrain {
                left_motor_1: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                left_motor_2: Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
                left_motor_3: Motor::new(peripherals.port_2, Gearset::Blue, Direction::Reverse),
                left_motor_4: Motor::new(peripherals.port_15, Gearset::Blue, Direction::Forward),

                right_motor_1: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                right_motor_2: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Reverse),
                right_motor_3: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
                right_motor_4: Motor::new(peripherals.port_5, Gearset::Blue, Direction::Forward),

                pose: pose.clone(),
            },

            display: peripherals.display,

            score_mech_top: Motor::new(peripherals.port_7, Gearset::Blue, Direction::Reverse),
            score_mech_wheel: Motor::new(peripherals.port_6, Gearset::Blue, Direction::Forward),

            intake_left: Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
            intake_right: Motor::new(peripherals.port_16, Gearset::Blue, Direction::Reverse),

            lift: AdiDigitalOut::new(peripherals.adi_e),
            matchload: AdiDigitalOut::new(peripherals.adi_a),
            flap: AdiDigitalOut::new(peripherals.adi_f),

            controller: peripherals.primary_controller,
        }
    } else {
        Robot {
            drivetrain: Drivetrain {
                left_motor_1: Motor::new(peripherals.port_1, Gearset::Blue, Direction::Forward),
                left_motor_2: Motor::new(peripherals.port_11, Gearset::Blue, Direction::Forward),
                left_motor_3: Motor::new(peripherals.port_3, Gearset::Blue, Direction::Reverse),
                left_motor_4: Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),

                right_motor_1: Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
                right_motor_2: Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
                right_motor_3: Motor::new(peripherals.port_7, Gearset::Blue, Direction::Forward),
                right_motor_4: Motor::new(peripherals.port_8, Gearset::Blue, Direction::Forward),

                pose: pose.clone(),
            },

            display: peripherals.display,

            score_mech_top: Motor::new(peripherals.port_5, Gearset::Blue, Direction::Reverse),
            score_mech_wheel: Motor::new(peripherals.port_12, Gearset::Blue, Direction::Forward),

            intake_left: Motor::new(peripherals.port_17, Gearset::Blue, Direction::Forward),
            intake_right: Motor::new(peripherals.port_20, Gearset::Blue, Direction::Reverse),

            lift: AdiDigitalOut::new(peripherals.adi_a),
            matchload: AdiDigitalOut::new(peripherals.adi_e),
            flap: AdiDigitalOut::new(peripherals.adi_f),

            controller: peripherals.primary_controller,
        }
    };

    // let mut imu_1 = InertialSensor::new(peripherals.port_14);
    // let mut imu_2 = InertialSensor::new(peripherals.port_12);

    // let tracking_wheel_1 = RotationSensor::new(peripherals.port_15, Direction::Reverse);
    // let tracking_wheel_2 = RotationSensor::new(peripherals.port_16, Direction::Forward);

    // let _calib = join!(imu_1.calibrate(), imu_2.calibrate()).await;

    // let _odom = spawn(odom_thread(
    //     imu_1,
    //     imu_2,
    //     tracking_wheel_1,
    //     tracking_wheel_2,
    //     pose,
    // ));

    robot.compete().await;
}
