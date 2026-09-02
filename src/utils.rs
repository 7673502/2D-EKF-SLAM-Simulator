use macroquad::prelude::*;

pub fn set_seed(seed: u64) {
    rand::srand(seed);
}

pub fn sample_normal_with_state(mean: f32, std_dev: f32, state: &rand::RandGenerator) -> f32 {
    if std_dev == 0.0 {
        return mean;
    }

    let u1 = state.gen_range(0.0f32, 1.0f32).max(1e-6);
    let u2 = state.gen_range(0.0f32, 1.0f32);

    let z0 = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();

    mean + std_dev * z0
}

/*
 * Box-Mueller transform to generate normally distributed values
 * https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
 */
pub fn sample_normal(mean: f32, std_dev: f32) -> f32 {
    if std_dev == 0.0 {
        return mean;
    }

    let u1 = rand::gen_range(0.0f32, 1.0f32).max(1e-6); // don't want to do ln of tiny numbers
    let u2 = rand::gen_range(0.0f32, 1.0f32);

    let z0 = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();

    mean + std_dev * z0
}

/*
 * helper that converts relative position of landmark (range and bearing)
 * to absolute (x, y) coordinates
 */
pub fn relative_to_absolute(
    robot_x: f32,
    robot_y: f32,
    robot_theta: f32,
    range: f32,
    bearing: f32,
) -> (f32, f32) {
    let absolute_angle = robot_theta + bearing;

    let x = robot_x + range * absolute_angle.cos();
    let y = robot_y + range * absolute_angle.sin();

    (x, y)
}

/*
 * helper that converts absolute position of landmark (x and y) to
 * tuple of form (range, bearing)
 */
pub fn absolute_to_relative(
    robot_x: f32,
    robot_y: f32,
    robot_theta: f32,
    landmark_x: f32,
    landmark_y: f32,
) -> (f32, f32) {
    // distance to landmark
    let distance_x = landmark_x - robot_x;
    let distance_y = landmark_y - robot_y;
    let range = (distance_x * distance_x + distance_y * distance_y).sqrt();

    // calculate relative angle
    let absolute_angle = f32::atan2(distance_y, distance_x);
    let mut bearing = absolute_angle - robot_theta;
    bearing = f32::atan2(bearing.sin(), bearing.cos()); // normalize to (-PI, PI]

    (range, bearing)
}
