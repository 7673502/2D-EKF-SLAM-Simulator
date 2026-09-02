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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_relative_to_absolute_at_origin_facing_east() {
        let (x, y) = relative_to_absolute(0.0, 0.0, 0.0, 5.0, 0.0);
        assert!((x - 5.0).abs() < 1e-5);
        assert!(y.abs() < 1e-5);
    }

    #[test]
    fn test_relative_to_absolute_facing_north() {
        let (x, y) = relative_to_absolute(0.0, 0.0, PI / 2.0, 5.0, 0.0);
        assert!(x.abs() < 1e-5);
        assert!((y - 5.0).abs() < 1e-5);
    }

    #[test]
    fn test_relative_to_absolute_facing_west_negative_bearing() {
        let (x, y) = relative_to_absolute(10.0, 20.0, PI, 5.0, -PI / 2.0);
        assert!((x - 10.0).abs() < 1e-5);
        assert!((y - 25.0).abs() < 1e-4);
    }

    #[test]
    fn test_absolute_to_relative_at_origin_facing_east() {
        let (r, b) = absolute_to_relative(0.0, 0.0, 0.0, 5.0, 0.0);
        assert!((r - 5.0).abs() < 1e-5);
        assert!(b.abs() < 1e-5);
    }

    #[test]
    fn test_absolute_to_relative_facing_north() {
        let (r, b) = absolute_to_relative(0.0, 0.0, PI / 2.0, 0.0, 5.0);
        assert!((r - 5.0).abs() < 1e-5);
        assert!(b.abs() < 1e-5);
    }

    #[test]
    fn test_absolute_to_relative_bearing_wraparound_negative_pi() {
        let (r, b) = absolute_to_relative(0.0, 0.0, 0.9 * PI, 0.0, -5.0);
        assert!((r - 5.0).abs() < 1e-5);
        let expected_b = 0.6 * PI;
        assert!((b - expected_b).abs() < 1e-5);
    }

    #[test]
    fn test_absolute_to_relative_bearing_wraparound_positive_pi() {
        let (r, b) = absolute_to_relative(0.0, 0.0, -0.9 * PI, 0.0, 5.0);
        assert!((r - 5.0).abs() < 1e-5);
        let expected_b = -0.6 * PI;
        assert!((b - expected_b).abs() < 1e-5);
    }

    #[test]
    fn test_coordinate_transforms_round_trip_relative_then_absolute() {
        let rx = 12.0;
        let ry = -8.0;
        let rtheta = 0.7;
        let initial_r = 15.0;
        let initial_b = -0.4;
        let (lx, ly) = relative_to_absolute(rx, ry, rtheta, initial_r, initial_b);
        let (r, b) = absolute_to_relative(rx, ry, rtheta, lx, ly);
        assert!((r - initial_r).abs() < 1e-4);
        assert!((b - initial_b).abs() < 1e-4);
    }

    #[test]
    fn test_coordinate_transforms_round_trip_absolute_then_relative() {
        let rx = -5.0;
        let ry = 14.0;
        let rtheta = 2.1;
        let target_lx = 10.0;
        let target_ly = -3.0;
        let (r, b) = absolute_to_relative(rx, ry, rtheta, target_lx, target_ly);
        let (lx, ly) = relative_to_absolute(rx, ry, rtheta, r, b);
        assert!((lx - target_lx).abs() < 1e-4);
        assert!((ly - target_ly).abs() < 1e-4);
    }

    #[test]
    fn test_absolute_to_relative_coincident_robot_and_landmark() {
        let (r, _b) = absolute_to_relative(3.0, 4.0, 0.5, 3.0, 4.0);
        assert_eq!(r, 0.0);
    }

    #[test]
    fn test_sample_normal_zero_standard_deviation_returns_exact_mean() {
        let val = sample_normal(42.5, 0.0);
        assert_eq!(val, 42.5);
    }

    #[test]
    fn test_sample_normal_empirical_mean_convergence() {
        set_seed(12345);
        let n = 10_000;
        let target_mean = 5.0;
        let stdev = 2.0;
        let mut sum = 0.0;
        for _ in 0..n {
            sum += sample_normal(target_mean, stdev);
        }
        let empirical_mean = sum / (n as f32);
        assert!((empirical_mean - target_mean).abs() < 0.1);
    }

    #[test]
    fn test_sample_normal_empirical_variance_convergence() {
        set_seed(67890);
        let n = 10_000;
        let mean = 0.0;
        let stdev = 2.0;
        let target_variance = stdev * stdev;
        let mut samples = Vec::with_capacity(n);
        let mut sum = 0.0;
        for _ in 0..n {
            let s = sample_normal(mean, stdev);
            samples.push(s);
            sum += s;
        }
        let empirical_mean = sum / (n as f32);
        let variance: f32 = samples
            .iter()
            .map(|&s| (s - empirical_mean).powi(2))
            .sum::<f32>()
            / (n as f32);
        assert!((variance - target_variance).abs() < 0.2);
    }

    #[test]
    fn test_sample_normal_nonzero_mean_and_variance() {
        set_seed(54321);
        let n = 10_000;
        let mean = 10.0;
        let stdev = 3.0;
        let mut sum = 0.0;
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            let s = sample_normal(mean, stdev);
            samples.push(s);
            sum += s;
        }
        let empirical_mean = sum / (n as f32);
        let variance: f32 = samples
            .iter()
            .map(|&s| (s - empirical_mean).powi(2))
            .sum::<f32>()
            / (n as f32);
        assert!((empirical_mean - mean).abs() < 0.15);
        assert!((variance - 9.0).abs() < 0.35);
    }
}
