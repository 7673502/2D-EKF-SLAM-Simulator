use macroquad::prelude::Color;
use nalgebra::{DMatrix, DVector, Matrix2, Matrix2x3, Vector2, stack};
use std::collections::HashMap;

use crate::config::Config;
use crate::simulation::Observation;
use crate::slam::Slam;
use crate::utils::{absolute_to_relative, relative_to_absolute};

pub struct EkfSlam {
    pub state: DVector<f32>,
    pub covariance: DMatrix<f32>,
    pub observed_landmarks: HashMap<usize, usize>, // maps ids to state index
}

impl EkfSlam {
    pub const COLOR: Color = Color::new(0.0, 0.7, 1.0, 0.5);

    pub fn new() -> Self {
        Self {
            state: DVector::from_element(3, 0.0), // initial state vector contains robot x, y, angle
            covariance: DMatrix::identity(3, 3) * 0.01, // size is 3 + 2L where L is the number of landmarks
            observed_landmarks: HashMap::new(),
        }
    }

    /*
     * ekf landmark initialization step for full observations
     */
    fn initialize_landmark(&mut self, observation: &Observation, cfg: &Config) {
        let old_len = self.state.nrows(); // old length of state vector
        let (x, y) = relative_to_absolute(
            self.state[0],
            self.state[1],
            self.state[2],
            observation.range,
            observation.bearing,
        );

        // update hashmap
        self.observed_landmarks.insert(observation.id, old_len);

        // take ownership of state because resize_vertically requires value, not reference
        let mut state = std::mem::take(&mut self.state);

        // update state vector
        state = state.resize_vertically(old_len + 2, 0.0);
        state[old_len] = x;
        state[old_len + 1] = y;
        self.state = state; // return ownership

        // calculate new values for covariance
        let theta = self.state[2];
        let absolute_angle = theta + observation.bearing;

        // jacobian of landmark position with respect to robot state
        let g_r = Matrix2x3::new(
            1.0,
            0.0,
            -observation.range * absolute_angle.sin(),
            0.0,
            1.0,
            observation.range * absolute_angle.cos(),
        );

        // jacobian of landmark position with respect to observation
        let g_y = Matrix2::new(
            absolute_angle.cos(),
            -observation.range * absolute_angle.sin(),
            absolute_angle.sin(),
            observation.range * absolute_angle.cos(),
        );

        // covariance of landmark
        let p_rr = self.covariance.fixed_view::<3, 3>(0, 0);

        // sensor noise
        let r = Matrix2::new(
            cfg.est_stdev_range.powi(2),
            0.0,
            0.0,
            cfg.est_stdev_bearing.powi(2),
        );

        // landmark covariance
        let p_ll = (g_r * p_rr * g_r.transpose()) + (g_y * r * g_y.transpose());

        // robot-map covariance
        let p_rx = self.covariance.view((0, 0), (3, old_len));

        // landmark-map covariance
        let p_lx = g_r * p_rx;

        // take ownership of covariance
        let mut covariance = std::mem::take(&mut self.covariance);

        // update covariance
        covariance = covariance.resize(old_len + 2, old_len + 2, 0.0);

        covariance
            .view_mut((old_len, 0), (2, old_len))
            .copy_from(&p_lx);
        covariance
            .view_mut((0, old_len), (old_len, 2))
            .copy_from(&p_lx.transpose());
        covariance
            .view_mut((old_len, old_len), (2, 2))
            .copy_from(&p_ll);

        self.covariance = covariance; // return ownership
    }

    /*
     * ekf correction step
     */
    fn correct_landmark(&mut self, observation: &Observation, landmark_index: usize, cfg: &Config) {
        let robot_x = self.state[0];
        let robot_y = self.state[1];

        let landmark_x = self.state[landmark_index];
        let landmark_y = self.state[landmark_index + 1];

        // predicted measurement and innovation
        let (predicted_range, predicted_bearing) =
            absolute_to_relative(robot_x, robot_y, self.state[2], landmark_x, landmark_y);
        let range_difference = observation.range - predicted_range;
        let bearing_difference = f32::atan2(
            (observation.bearing - predicted_bearing).sin(),
            (observation.bearing - predicted_bearing).cos(),
        );

        // innovation vector
        let z = Vector2::new(range_difference, bearing_difference);

        // distance to landmark
        let distance_x = landmark_x - robot_x;
        let distance_y = landmark_y - robot_y;
        let distance_sq = (distance_x * distance_x + distance_y * distance_y).max(1e-6);
        let distance = distance_sq.sqrt();

        // jacobian with respect to robot
        let h_r = Matrix2x3::new(
            -distance_x / distance,
            -distance_y / distance,
            0.0,
            distance_y / distance_sq,
            -distance_x / distance_sq,
            -1.0,
        );

        // jacobian with respect to landmark
        let h_l = Matrix2::new(
            distance_x / distance,
            distance_y / distance,
            -distance_y / distance_sq,
            distance_x / distance_sq,
        );

        // innovation covariance calculation
        let p_rr = self.covariance.fixed_view::<3, 3>(0, 0); // robot-robot covariance
        let p_ll = self
            .covariance
            .fixed_view::<2, 2>(landmark_index, landmark_index); // landmark-landmark covariance
        let p_rl = self.covariance.fixed_view::<3, 2>(0, landmark_index); // robot-landmark covariance
        let p_lr = p_rl.transpose(); // landmark-robot covariance

        // sensor noise
        let r = Matrix2::new(
            cfg.est_stdev_range.powi(2),
            0.0,
            0.0,
            cfg.est_stdev_bearing.powi(2),
        );

        // block matrices
        let h_block = stack![h_r, h_l];
        let p_block = stack![
            p_rr, p_rl;
            p_lr, p_ll
        ];
        let h_t_block = h_block.transpose();

        // innovation matrix
        let z_matrix = h_block * p_block * h_t_block + r;

        // calculate product of covariance with jacobian transpose (PH^T)
        let total_map_size = self.state.nrows();
        let p_cols_robot = self.covariance.view((0, 0), (total_map_size, 3));
        let p_cols_landmark = self
            .covariance
            .view((0, landmark_index), (total_map_size, 2));
        let p_ht = (p_cols_robot * h_r.transpose()) + (p_cols_landmark * h_l.transpose());

        // Kalman gain
        let k = p_ht * z_matrix.try_inverse().unwrap();

        // update state and covariance
        self.state = &self.state + &k * z;
        self.covariance = &self.covariance - &k * z_matrix * k.transpose();

        // force matrix to be symmetric to (hopefully) prevent covariance from exploding
        self.covariance = (&self.covariance + self.covariance.transpose()) / 2.0;

        // normalize angle
        self.state[2] = f32::atan2(self.state[2].sin(), self.state[2].cos());
    }
}

impl Slam for EkfSlam {
    /*
     * follows the EKF sparse prediction equations from
     * https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
     */
    fn predict(
        &mut self,
        linear_velocity: f32,
        angular_velocity: f32,
        delta_time: f32,
        cfg: &Config,
    ) {
        debug_assert!(
            self.covariance.is_square(),
            "Covariance must be square matrix."
        );

        let theta = self.state[2];
        let theta_half = theta + 0.5 * angular_velocity * delta_time; // approximate heading of the robot at the middle of the frame

        // update position estimate
        self.state[0] += linear_velocity * delta_time * theta_half.cos();
        self.state[1] += linear_velocity * delta_time * theta_half.sin();
        self.state[2] += angular_velocity * delta_time;

        // normalize angle to (-PI, PI]
        self.state[2] = f32::atan2(self.state[2].sin(), self.state[2].cos());

        // jacobian of the motion model function
        let f_x = nalgebra::Matrix3::new(
            1.0,
            0.0,
            -linear_velocity * delta_time * theta_half.sin(),
            0.0,
            1.0,
            linear_velocity * delta_time * theta_half.cos(),
            0.0,
            0.0,
            1.0,
        );

        // covariance of control noise
        let sigma_linear_velocity = cfg.est_stdev_linear * linear_velocity.abs() + 0.01; // add 0.01 so noise doesn't vanish at 0 speed
        let sigma_angular_velocity = cfg.est_stdev_angular * angular_velocity.abs() + 0.01;
        let n = nalgebra::Matrix2::new(
            (sigma_linear_velocity).powi(2),
            0.0,
            0.0,
            (sigma_angular_velocity).powi(2),
        );

        // jacobian of control noise (assumes noise is on controls, not state
        // and noise is indepentend between linear velocity and angular velocity)
        let f_n = nalgebra::Matrix3x2::new(
            theta_half.cos() * delta_time,
            0.0,
            theta_half.sin() * delta_time,
            0.0,
            0.0,
            delta_time,
        );

        // update robot covariance block
        let p_rr = self.covariance.fixed_view::<3, 3>(0, 0);
        let new_p_rr = (f_x * p_rr * f_x.transpose()) + (f_n * n * f_n.transpose());
        self.covariance
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&new_p_rr);

        let map_size = self.covariance.ncols() - 3;

        if map_size > 0 {
            // update robot-map cross-covariance
            let p_rm = self.covariance.view((0, 3), (3, map_size)).into_owned();
            let new_p_rm = f_x * p_rm;
            self.covariance
                .view_mut((0, 3), (3, map_size))
                .copy_from(&new_p_rm);

            // update map-robot cross-covariance
            self.covariance
                .view_mut((3, 0), (map_size, 3))
                .copy_from(&new_p_rm.transpose());
        }
    }

    /*
     * Follows EKF sparse prediction equations from
     * https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
     */
    fn update(&mut self, observations: &[Observation], cfg: &Config) {
        for observation in observations.iter() {
            match self.observed_landmarks.get(&observation.id) {
                Some(&landmark_index) => {
                    self.correct_landmark(observation, landmark_index, cfg);
                }
                None => {
                    self.initialize_landmark(observation, cfg);
                }
            }
        }
    }

    fn get_state(&self) -> (f32, f32, f32) {
        (self.state[0], self.state[1], self.state[2])
    }

    fn get_landmarks(&self) -> Vec<(usize, f32, f32)> {
        let mut landmarks = Vec::new();

        for (id, &index) in &self.observed_landmarks {
            landmarks.push((*id, self.state[index], self.state[index + 1]));
        }

        landmarks
    }

    fn color(&self) -> Color {
        Self::COLOR
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_ekf_new_initial_state_vector_size_and_values() {
        let ekf = EkfSlam::new();
        assert_eq!(ekf.state.len(), 3);
        assert_eq!(ekf.state[0], 0.0);
        assert_eq!(ekf.state[1], 0.0);
        assert_eq!(ekf.state[2], 0.0);
    }

    #[test]
    fn test_ekf_new_initial_covariance_symmetry_and_scale() {
        let ekf = EkfSlam::new();
        assert_eq!(ekf.covariance.nrows(), 3);
        assert_eq!(ekf.covariance.ncols(), 3);
        for i in 0..3 {
            for j in 0..3 {
                if i == j {
                    assert!((ekf.covariance[(i, j)] - 0.01).abs() < 1e-6);
                } else {
                    assert_eq!(ekf.covariance[(i, j)], 0.0);
                }
            }
        }
    }

    #[test]
    fn test_ekf_new_observed_landmarks_empty() {
        let ekf = EkfSlam::new();
        assert!(ekf.observed_landmarks.is_empty());
    }

    #[test]
    fn test_ekf_get_state_returns_robot_pose() {
        let mut ekf = EkfSlam::new();
        ekf.state[0] = 1.0;
        ekf.state[1] = 2.0;
        ekf.state[2] = 0.5;
        let (x, y, theta) = ekf.get_state();
        assert_eq!(x, 1.0);
        assert_eq!(y, 2.0);
        assert_eq!(theta, 0.5);
    }

    #[test]
    fn test_ekf_get_landmarks_empty_on_fresh_instance() {
        let ekf = EkfSlam::new();
        assert!(ekf.get_landmarks().is_empty());
    }

    #[test]
    fn test_ekf_predict_pure_translation_straight_forward() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.predict(10.0, 0.0, 1.0, &cfg);
        assert!((ekf.state[0] - 10.0).abs() < 1e-4);
        assert!(ekf.state[1].abs() < 1e-4);
        assert!(ekf.state[2].abs() < 1e-4);
    }

    #[test]
    fn test_ekf_predict_pure_translation_with_heading() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.state[2] = FRAC_PI_2;
        ekf.predict(10.0, 0.0, 1.0, &cfg);
        assert!(ekf.state[0].abs() < 1e-4);
        assert!((ekf.state[1] - 10.0).abs() < 1e-4);
    }

    #[test]
    fn test_ekf_predict_pure_rotation_in_place() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.predict(0.0, 1.0, 1.0, &cfg);
        assert!(ekf.state[0].abs() < 1e-4);
        assert!(ekf.state[1].abs() < 1e-4);
        assert!((ekf.state[2] - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_ekf_predict_stationary_preserves_mean_increases_covariance() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        let initial_trace = ekf.covariance.trace();
        ekf.predict(0.0, 0.0, 1.0, &cfg);
        assert_eq!(ekf.state[0], 0.0);
        assert_eq!(ekf.state[1], 0.0);
        assert_eq!(ekf.state[2], 0.0);
        assert!(ekf.covariance.trace() > initial_trace);
    }

    #[test]
    fn test_ekf_predict_heading_wraparound_positive_pi() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.state[2] = PI - 0.1;
        ekf.predict(0.0, 0.5, 1.0, &cfg);
        assert!(ekf.state[2] > -PI && ekf.state[2] <= PI);
        assert!(ekf.state[2] < 0.0);
    }

    #[test]
    fn test_ekf_predict_heading_wraparound_negative_pi() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.state[2] = -PI + 0.1;
        ekf.predict(0.0, -0.5, 1.0, &cfg);
        assert!(ekf.state[2] > -PI && ekf.state[2] <= PI);
        assert!(ekf.state[2] > 0.0);
    }

    #[test]
    fn test_ekf_predict_preserves_covariance_symmetry() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.predict(5.0, 0.5, 0.5, &cfg);
        for i in 0..ekf.covariance.nrows() {
            for j in 0..ekf.covariance.ncols() {
                assert!((ekf.covariance[(i, j)] - ekf.covariance[(j, i)]).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn test_ekf_predict_preserves_covariance_positive_definiteness() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.predict(10.0, 0.2, 0.5, &cfg);
        let eigen = ekf.covariance.symmetric_eigen();
        assert!(eigen.eigenvalues.iter().all(|&e| e > 0.0));
    }

    #[test]
    fn test_ekf_predict_invariant_landmark_positions() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 0,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        let lx_before = ekf.state[3];
        let ly_before = ekf.state[4];
        ekf.predict(5.0, 0.2, 1.0, &cfg);
        assert_eq!(ekf.state[3], lx_before);
        assert_eq!(ekf.state[4], ly_before);
    }

    #[test]
    fn test_ekf_predict_invariant_landmark_covariance_submatrix() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 0,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        let p_ll_before = ekf.covariance.fixed_view::<2, 2>(3, 3).into_owned();
        ekf.predict(5.0, 0.2, 1.0, &cfg);
        let p_ll_after = ekf.covariance.fixed_view::<2, 2>(3, 3);
        assert_eq!(p_ll_before, p_ll_after);
    }

    #[test]
    fn test_ekf_predict_propagates_robot_landmark_cross_covariance() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 0,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        let p_rm_before = ekf.covariance.view((0, 3), (3, 2)).into_owned();
        ekf.predict(5.0, 0.2, 1.0, &cfg);
        let p_rm_after = ekf.covariance.view((0, 3), (3, 2)).into_owned();
        assert_ne!(p_rm_before, p_rm_after);
    }

    #[test]
    fn test_ekf_predict_preserves_state_and_covariance_dimensions() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 0,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        ekf.predict(2.0, 0.1, 0.5, &cfg);
        assert_eq!(ekf.state.len(), 5);
        assert_eq!(ekf.covariance.shape(), (5, 5));
    }

    #[test]
    fn test_ekf_initialize_single_landmark_increases_state_size() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 10,
                range: 20.0,
                bearing: 0.5,
            },
            &cfg,
        );
        assert_eq!(ekf.state.len(), 5);
    }

    #[test]
    fn test_ekf_initialize_single_landmark_mean_matches_observation_geometry() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.state[0] = 5.0;
        ekf.state[1] = 7.0;
        ekf.state[2] = 0.2;
        let obs = Observation {
            id: 1,
            range: 15.0,
            bearing: 0.3,
        };
        let (expected_x, expected_y) = relative_to_absolute(5.0, 7.0, 0.2, 15.0, 0.3);
        ekf.initialize_landmark(&obs, &cfg);
        assert!((ekf.state[3] - expected_x).abs() < 1e-4);
        assert!((ekf.state[4] - expected_y).abs() < 1e-4);
    }

    #[test]
    fn test_ekf_initialize_single_landmark_increases_covariance_dimensions() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        assert_eq!(ekf.covariance.shape(), (5, 5));
    }

    #[test]
    fn test_ekf_initialize_single_landmark_covariance_block_symmetry() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.4,
            },
            &cfg,
        );
        assert!((ekf.covariance[(3, 4)] - ekf.covariance[(4, 3)]).abs() < 1e-5);
    }

    #[test]
    fn test_ekf_initialize_single_landmark_cross_covariance_consistency() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.4,
            },
            &cfg,
        );
        for r in 0..3 {
            for c in 3..5 {
                assert!((ekf.covariance[(r, c)] - ekf.covariance[(c, r)]).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn test_ekf_initialize_multiple_landmarks_sequential_mapping() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        ekf.initialize_landmark(
            &Observation {
                id: 2,
                range: 20.0,
                bearing: 0.5,
            },
            &cfg,
        );
        assert_eq!(ekf.state.len(), 7);
        assert_eq!(ekf.covariance.shape(), (7, 7));
        assert_eq!(ekf.observed_landmarks.len(), 2);
    }

    #[test]
    fn test_ekf_correct_perfect_measurement_reduces_pose_covariance_trace() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        ekf.initialize_landmark(&obs, &cfg);
        ekf.predict(1.0, 0.0, 1.0, &cfg);
        let pose_cov_trace_before = ekf.covariance.fixed_view::<3, 3>(0, 0).trace();
        let (r, b) = absolute_to_relative(
            ekf.state[0],
            ekf.state[1],
            ekf.state[2],
            ekf.state[3],
            ekf.state[4],
        );
        ekf.correct_landmark(
            &Observation {
                id: 1,
                range: r,
                bearing: b,
            },
            3,
            &cfg,
        );
        let pose_cov_trace_after = ekf.covariance.fixed_view::<3, 3>(0, 0).trace();
        assert!(pose_cov_trace_after < pose_cov_trace_before);
    }

    #[test]
    fn test_ekf_correct_perfect_measurement_reduces_landmark_covariance_trace() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        ekf.initialize_landmark(&obs, &cfg);
        let lm_trace_before = ekf.covariance.fixed_view::<2, 2>(3, 3).trace();
        ekf.correct_landmark(&obs, 3, &cfg);
        let lm_trace_after = ekf.covariance.fixed_view::<2, 2>(3, 3).trace();
        assert!(lm_trace_after < lm_trace_before);
    }

    #[test]
    fn test_ekf_correct_bearing_innovation_wraparound_boundary() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.state[2] = PI - 0.05;
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: -PI + 0.05,
        };
        ekf.initialize_landmark(&obs, &cfg);
        ekf.correct_landmark(&obs, 3, &cfg);
        assert!(!ekf.state[0].is_nan() && !ekf.state[1].is_nan() && !ekf.state[2].is_nan());
    }

    #[test]
    fn test_ekf_correct_innovation_adjusts_robot_pose_towards_landmark() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        ekf.predict(5.0, 0.0, 1.0, &cfg);
        let obs_shorter = Observation {
            id: 1,
            range: 4.0,
            bearing: 0.0,
        };
        let x_before = ekf.state[0];
        ekf.correct_landmark(&obs_shorter, 3, &cfg);
        assert!(ekf.state[0] > x_before);
    }

    #[test]
    fn test_ekf_correct_preserves_covariance_symmetry() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.2,
        };
        ekf.initialize_landmark(&obs, &cfg);
        ekf.correct_landmark(
            &Observation {
                id: 1,
                range: 9.8,
                bearing: 0.22,
            },
            3,
            &cfg,
        );
        for i in 0..5 {
            for j in 0..5 {
                assert!((ekf.covariance[(i, j)] - ekf.covariance[(j, i)]).abs() < 1e-5);
            }
        }
    }

    #[test]
    fn test_ekf_correct_preserves_positive_definiteness() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.2,
        };
        ekf.initialize_landmark(&obs, &cfg);
        ekf.correct_landmark(
            &Observation {
                id: 1,
                range: 9.8,
                bearing: 0.22,
            },
            3,
            &cfg,
        );
        let eigen = ekf.covariance.symmetric_eigen();
        assert!(eigen.eigenvalues.iter().all(|&e| e > 0.0));
    }

    #[test]
    fn test_ekf_correct_cross_correlation_updates_unobserved_landmarks() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        ekf.initialize_landmark(
            &Observation {
                id: 2,
                range: 15.0,
                bearing: 0.5,
            },
            &cfg,
        );
        let l2_x_before = ekf.state[5];
        let l2_y_before = ekf.state[6];
        ekf.correct_landmark(
            &Observation {
                id: 1,
                range: 8.0,
                bearing: 0.1,
            },
            3,
            &cfg,
        );
        assert!(ekf.state[5] != l2_x_before || ekf.state[6] != l2_y_before);
    }

    #[test]
    fn test_ekf_correct_near_zero_range_numerical_stability() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.initialize_landmark(
            &Observation {
                id: 1,
                range: 1e-4,
                bearing: 0.0,
            },
            &cfg,
        );
        ekf.correct_landmark(
            &Observation {
                id: 1,
                range: 1e-4,
                bearing: 0.0,
            },
            3,
            &cfg,
        );
        assert!(!ekf.state[0].is_nan() && !ekf.state[3].is_nan());
    }

    #[test]
    fn test_ekf_update_routes_unseen_id_to_initialization() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.update(
            &[Observation {
                id: 10,
                range: 10.0,
                bearing: 0.0,
            }],
            &cfg,
        );
        assert!(ekf.observed_landmarks.contains_key(&10));
        assert_eq!(ekf.state.len(), 5);
    }

    #[test]
    fn test_ekf_update_routes_seen_id_to_correction() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.update(
            &[Observation {
                id: 10,
                range: 10.0,
                bearing: 0.0,
            }],
            &cfg,
        );
        ekf.update(
            &[Observation {
                id: 10,
                range: 10.1,
                bearing: 0.01,
            }],
            &cfg,
        );
        assert_eq!(ekf.state.len(), 5);
    }

    #[test]
    fn test_ekf_update_processes_multiple_mixed_observations() {
        let cfg = Config::default();
        let mut ekf = EkfSlam::new();
        ekf.update(
            &[Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            }],
            &cfg,
        );
        ekf.update(
            &[
                Observation {
                    id: 1,
                    range: 10.0,
                    bearing: 0.0,
                },
                Observation {
                    id: 2,
                    range: 15.0,
                    bearing: 0.5,
                },
            ],
            &cfg,
        );
        assert_eq!(ekf.state.len(), 7);
        assert_eq!(ekf.observed_landmarks.len(), 2);
    }
}
