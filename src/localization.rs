use std::{cell::RefCell, rc::Rc};

use vexide::prelude::*;

use crate::utils::{degree_wrap, rad_to_deg};

pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub rot: f64,
}

pub async fn odom_thread(
    mut imu_1: InertialSensor,
    mut imu_2: InertialSensor,
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
        // Handles adveraging IMUs + Failover
        let rotation = match (h1, h2) {
            (Ok(h1_a), Ok(h2_a)) => {
                let h1_r = h1_a.as_radians();
                let h2_r = h2_a.as_radians();

                let xrot = h1_r.cos() + h2_r.cos();
                let yrot = h1_r.sin() + h2_r.sin();

                degree_wrap(rad_to_deg(f64::atan2(yrot, xrot)))
            }

            (Ok(h1_a), Err(_)) => degree_wrap(h1_a.as_degrees()),

            (Err(_), Ok(h2_a)) => degree_wrap(h2_a.as_degrees()),

            _ => pose.borrow_mut().rot.clone(),
        };

        pose.borrow_mut().rot = rotation;

        sleep(InertialSensor::UPDATE_INTERVAL).await;
    }
}
