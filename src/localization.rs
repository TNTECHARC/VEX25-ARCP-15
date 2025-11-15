use core::cell::RefCell;

use vexide::prelude::*;

use crate::alloc::rc::Rc;

pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub rot: f64,
}

pub async fn odom_thread(
    imu_1: InertialSensor,
    imu_2: InertialSensor,
    pose: Rc<RefCell<Pose>>) {

    loop {
        let h1 = imu_1.heading().unwrap_or_default().to_degrees();
        let h2 = imu_2.heading().unwrap_or_default().to_degrees();

        pose.borrow_mut().rot = (h1 + h2) / 2.0;

        sleep(InertialSensor::UPDATE_INTERVAL).await;
    }
}
