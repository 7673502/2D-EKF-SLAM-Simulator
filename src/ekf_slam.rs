use nalgebra::{DMatrix, DVector, Matrix2x3, Matrix2};
use std::{collections::HashMap};
use crate::simulation::Observation;
use crate::config::Config;

pub struct EkfSlam {
    pub state: DVector<f32>,
    pub covariance: DMatrix<f32>,
    pub observed_landmarks: HashMap<usize, usize>, // maps ids to state index
}

impl EkfSlam {
    pub fn new() -> Self {
        Self {
            state: DVector::from_element(3, 0.0), // initial state vector contains robot x, y, angle
            covariance: DMatrix::identity(3, 3) * 0.01, // size is 3 + 2L where L is the number of landmarks
            observed_landmarks: HashMap::new(),
        }
    }
    
    /*
     * follows the EKF sparse prediction equations from
     * https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
     */
    pub fn predict(&mut self, linear_velocity: f32, angular_velocity: f32, delta_time: f32) {
        debug_assert!(self.covariance.is_square(), "Covariance must be square matrix.");

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
            1.0, 0.0, -linear_velocity * delta_time * theta_half.sin(),
            0.0, 1.0, linear_velocity * delta_time * theta_half.cos(),
            0.0, 0.0, 1.0
        );

        // covariance of control noise
        let sigma_linear_velocity = 0.1 * linear_velocity.abs() + 0.01; // add 0.01 so noise doesn't vanish at 0 speed
        let sigma_angular_velocity = 0.1 * angular_velocity.abs() + 0.01;
        let n = nalgebra::Matrix2::new(
            (sigma_linear_velocity).powi(2), 0.0,
            0.0, (sigma_angular_velocity).powi(2)
        );
        
        // jacobian of control noise (assumes noise is on controls, not state 
        // and noise is indepentend between linear velocity and angular velocity)
        let f_n = nalgebra::Matrix3x2::new(
            theta_half.cos() * delta_time, 0.0,
            theta_half.sin() * delta_time, 0.0,
            0.0, delta_time
        );
        
        // update robot covariance block
        let p_rr = self.covariance.fixed_view::<3, 3>(0, 0);
        let new_p_rr = (f_x * p_rr * f_x.transpose()) + (f_n * n * f_n.transpose());
        self.covariance.fixed_view_mut::<3, 3>(0, 0).copy_from(&new_p_rr);
        
        
        let map_size = self.covariance.ncols() - 3;

        if map_size > 0 {
            // update robot-map cross-covariance
            let p_rm = self.covariance.view((0, 3), (3, map_size)).into_owned();
            let new_p_rm = f_x * p_rm;
            self.covariance.view_mut((0, 3), (3, map_size)).copy_from(&new_p_rm);

            // update map-robot cross-covariance
            self.covariance.view_mut((3, 0), (map_size, 3)).copy_from(&new_p_rm.transpose());
        }
    }

    /*
     * Follows EKF sparse prediction equations from
     * https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf
     */
    pub fn update(&mut self, observation: &Observation, cfg: &Config) {
        match self.observed_landmarks.get(&observation.id) {
            Some(&landmark_index) => {
                self.correct_landmark(observation, landmark_index);
            }
            None => {
                self.initialize_landmark(observation, cfg);
            }
        }
    }
    
    /*
     * landmark initialization for full observations
     */
    fn initialize_landmark(&mut self, observation: &Observation, cfg: &Config) {
        let old_len = self.state.nrows(); // old length of state vector
        let (x, y) = self.relative_to_absolute(observation.range, observation.bearing);

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
            1.0, 0.0, -observation.range * absolute_angle.sin(),
            0.0, 1.0, observation.range * absolute_angle.cos()
        );

        // jacobian of landmark position with respect to observation
        let g_y = Matrix2::new(
            absolute_angle.cos(), -observation.range * absolute_angle.sin(),
            absolute_angle.sin(), observation.range * absolute_angle.cos()
        );

        // covariance of landmark
        let p_rr = self.covariance.fixed_view::<3, 3>(0, 0);
        
        // sensor noise
        let r = Matrix2::new(
            cfg.sigma_range.powi(2), 0.0,
            0.0, cfg.sigma_bearing.powi(2)
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

        covariance.view_mut((old_len, 0), (2, old_len)).copy_from(&p_lx);
        covariance.view_mut((0, old_len), (old_len, 2)).copy_from(&p_lx.transpose());
        covariance.view_mut((old_len, old_len), (2, 2)).copy_from(&p_ll);

        self.covariance = covariance; // return ownership
    }
    
    fn correct_landmark(&self, observation: &Observation, landmark_index: usize) {
        // TODO
    }
    
    /*
     * helper that converts relative position of landmark (range and bearing)
     * to absolute (x, y) coordinates
     */
    fn relative_to_absolute(&self, range: f32, bearing: f32) -> (f32, f32) {
        let robot_x = self.state[0];
        let robot_y = self.state[1];
        let robot_theta = self.state[2];

        // absolute angle to landmark
        let absolute_angle = robot_theta + bearing;
        
        let x = robot_x + range * absolute_angle.cos();
        let y = robot_y + range * absolute_angle.sin();
        
        (x, y)
    }
    
    /*
     * helper that converts absolute position of landmark (x and y) to
     * tuple of form (range, bearing)
     */
    fn absolute_to_relative(&self, x: f32, y: f32) -> (f32, f32) {
        let robot_x = self.state[0];
        let robot_y = self.state[1];
        let robot_theta = self.state[2];
        
        // distance to landmark
        let distance_x = x - robot_x;
        let distance_y = y - robot_y;
        let range = (distance_x * distance_x + distance_y * distance_y).sqrt();
        
        // calculate relative angle
        let absolute_angle = f32::atan2(distance_y, distance_x);
        let mut bearing = absolute_angle - robot_theta;
        bearing = f32::atan2(bearing.sin(), bearing.cos()); // normalize to (-PI, PI]

        (range, bearing)
    }

}
