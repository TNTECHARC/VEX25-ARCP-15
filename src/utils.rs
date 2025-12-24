use std::f64::consts::PI;

const CURVE: f64 = 12.0;

pub fn degree_wrap(value: f64) -> f64 {
    let mut val = value % 360.0;
    if val >= 180.0 {
        val -= 360.0;
    } else if val < -180.0 {
        val += 360.0;
    }
    val
}

#[inline(always)]
pub fn deg_to_rad(deg: f64) -> f64 {
    deg * (PI / 180.0)
}

#[inline(always)]
pub fn rad_to_deg(rad: f64) -> f64 {
    rad * (180.0 / PI)
}

// https://www.desmos.com/calculator/sdcgzah5ya
pub fn curve_joystick(input: f64) -> f64 {
    f64::exp(((f64::abs(input) - 100.0) * CURVE) / 1000.0) * input
}

pub fn slew(last_spin: f64, spin: f64, change: f64, delta: f64) -> f64 {
    if last_spin.abs() + change * delta < spin.abs() {
        last_spin + change.copysign(spin) * delta
    } else {
        spin
    }
}

// Like normal slew, but only limits while accelerating
pub fn slew_acc(last_spin: f64, spin: f64, change: f64, delta: f64) -> f64 {
    if last_spin.abs() < spin.abs() && last_spin.abs() + change * delta < spin.abs() {
        last_spin + change.copysign(spin) * delta
    } else {
        spin
    }
}
