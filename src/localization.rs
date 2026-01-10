use std::{cell::RefCell, f64::consts::PI, rc::Rc, time::Instant};

use vexide::{math::Angle, prelude::*};

const WHEEL_ANGLE: Angle = Angle::from_degrees(45.0);
const WHEEL_DIAMETER: f64 = 2.00;
const WHEEL_CIRCUM: f64 = WHEEL_DIAMETER * PI;

#[derive(Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub rot: Angle,
    pub timestamp: Instant,
}

pub async fn odom_thread(
    imu_1: InertialSensor,
    imu_2: InertialSensor,
    mut tracking_wheel_1: RotationSensor,
    mut tracking_wheel_2: RotationSensor,
    pose: Rc<RefCell<Pose>>,
) {
    tracking_wheel_1.reset_position().ok();
    tracking_wheel_2.reset_position().ok();

    let mut tw_1_distance = 0.0;
    let mut tw_2_distance = 0.0;

    loop {
        let h1 = imu_1.heading();
        let h2 = imu_2.heading();

        // TODO: This should probably be a kalman filter
        // Handles averaging IMUs + Failover
        let rotation = match (h1, h2) {
            (Ok(h1_a), Ok(h2_a)) => {
                let (h1_sin, h1_cos) = h1_a.sin_cos();
                let (h2_sin, h2_cos) = h2_a.sin_cos();

                let xrot = h1_cos + h2_cos;
                let yrot = h1_sin + h2_sin;

                Angle::atan2(yrot, xrot).wrapped_half()
            }

            (Ok(h1_a), Err(_)) => h1_a.wrapped_half(),

            (Err(_), Ok(h2_a)) => h2_a.wrapped_half(),

            _ => pose.borrow().rot,
        };

        let new_tw_1_dist = tracking_wheel_1.position().unwrap_or_default().as_turns();
        let new_tw_2_dist = tracking_wheel_2.position().unwrap_or_default().as_turns();

        let delta_tw_1 = new_tw_1_dist - tw_1_distance;
        let delta_tw_2 = new_tw_2_dist - tw_2_distance;

        tw_1_distance = new_tw_1_dist;
        tw_2_distance = new_tw_2_dist;

        let local_x = (delta_tw_1 - delta_tw_2) * WHEEL_ANGLE.cos() * WHEEL_CIRCUM;
        let local_y = (delta_tw_1 + delta_tw_2) * WHEEL_ANGLE.sin() * WHEEL_CIRCUM;

        let (rotation_sin, rotation_cos) = rotation.sin_cos();

        {
            let mut pose_mut = pose.borrow_mut();

            let timestamp = Instant::now();

            pose_mut.rot = rotation;
            // pose_mut.x += local_x * rotation_cos - local_y * rotation_sin;
            // pose_mut.y += local_x * rotation_sin + local_y * rotation_cos;
            pose_mut.x += local_x;
            pose_mut.y += local_y;
            pose_mut.timestamp = timestamp;
        }

        sleep(InertialSensor::UPDATE_INTERVAL).await;
    }
}
