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
    time::LowResolutionTime,
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
        // Initialize motor velocity tracking variables for motors 1 & 3 only
        let mut lm_1_time = LowResolutionTime::now();
        let mut lm_3_time = LowResolutionTime::now();
        let mut rm_1_time = LowResolutionTime::now();
        let mut rm_3_time = LowResolutionTime::now();

        let mut lm_1_pos = self.drivetrain.left_motor_1.position().unwrap_or_default();
        let mut lm_3_pos = self.drivetrain.left_motor_3.position().unwrap_or_default();
        let mut rm_1_pos = self.drivetrain.right_motor_1.position().unwrap_or_default();
        let mut rm_3_pos = self.drivetrain.right_motor_3.position().unwrap_or_default();

        // Open file once for streaming with buffer
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open("velocity_data.txt")
            .expect("Unable to open velocity_data.txt");

        let mut writer = BufWriter::new(file);

        // Write header on first run
        writeln!(
            writer,
            "timestamp,pose_timestamp,left_voltage,right_voltage,left_velocity,right_velocity,x,y,rot"
        )
        .expect("Unable to write header");

        self.display.erase(Color::RED);

        let start = Instant::now();

        loop {
            let state = self.controller.state().unwrap_or_default();

            let lside = state.left_stick.y();
            let rside = state.right_stick.y();

            // Calculate motor velocities for motors 1 & 3 only
            let lm_1_vel = self.drivetrain.motor_velocity(
                &self.drivetrain.left_motor_1,
                &mut lm_1_time,
                &mut lm_1_pos,
            );
            let lm_3_vel = self.drivetrain.motor_velocity(
                &self.drivetrain.left_motor_3,
                &mut lm_3_time,
                &mut lm_3_pos,
            );
            let rm_1_vel = self.drivetrain.motor_velocity(
                &self.drivetrain.right_motor_1,
                &mut rm_1_time,
                &mut rm_1_pos,
            );
            let rm_3_vel = self.drivetrain.motor_velocity(
                &self.drivetrain.right_motor_3,
                &mut rm_3_time,
                &mut rm_3_pos,
            );

            // Calculate average velocities for each side
            let left_avg_vel = (lm_1_vel + lm_3_vel) / 2.0;
            let right_avg_vel = (rm_1_vel + rm_3_vel) / 2.0;

            {
                let pose = self.drivetrain.pose.borrow().clone();

                // Log data with timestamp
                let timestamp = std::time::Instant::now()
                    .duration_since(start)
                    .as_secs_f64();

                writeln!(
                    writer,
                    "{:},{:},{:.2},{:.2},{:.3},{:.3},{:.2},{:.2},{:.2}",
                    timestamp,
                    pose.timestamp.duration_since(start).as_secs_f64(),
                    lside * 12.0,
                    rside * 12.0,
                    left_avg_vel,
                    right_avg_vel,
                    pose.x,
                    pose.y,
                    pose.rot.as_degrees(),
                )
                .expect("Unable to write data");

                // Flush buffer occasionally to ensure data is written
                writer.flush().ok();

                self.display_text(
                    format!(
                        "X: {:.2}, Y: {:.2}, Rot: {:.2}",
                        pose.x,
                        pose.y,
                        pose.rot.as_degrees()
                    ),
                    [10, 10],
                );
            }

            self.drivetrain.left_drive(lside * 12.0);
            self.drivetrain.right_drive(rside * 12.0);

            if state.button_l1.is_pressed() {
                self.intake(12.0);
            } else if state.button_l2.is_pressed() {
                self.intake(-12.0);
            } else {
                self.intake(0.0);
            }

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

impl Robot {
    fn intake(&mut self, power: f64) {
        self.intake_left.set_voltage(power).ok();
        self.intake_right.set_voltage(power).ok();
        self.score_mech_left.set_voltage(power).ok();
        self.score_mech_right.set_voltage(power).ok();
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

    let tracking_wheel_1 = RotationSensor::new(peripherals.port_13, Direction::Reverse);
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
