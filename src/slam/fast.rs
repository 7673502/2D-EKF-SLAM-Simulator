use macroquad::prelude::Color;
use nalgebra::{Matrix2, Vector2};
use std::collections::HashMap;

use crate::config::Config;
use crate::simulation::Observation;
use crate::slam::Slam;
use crate::utils::{absolute_to_relative, relative_to_absolute, sample_normal};

#[derive(Clone)]
pub struct LandmarkEstimate {
    pub mu: Vector2<f32>,
    pub sigma: Matrix2<f32>,
}

#[derive(Clone)]
pub struct Particle {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub weight: f32,
    pub landmarks: HashMap<usize, LandmarkEstimate>,
}

pub struct FastSlam {
    pub particles: Vec<Particle>,
    pub num_particles: usize,
}

impl Particle {
    fn initialize_landmark(&mut self, observation: &Observation, cfg: &Config) {
        let absolute_angle = self.theta + observation.bearing;
        let (landmark_x, landmark_y) = relative_to_absolute(
            self.x,
            self.y,
            self.theta,
            observation.range,
            observation.bearing,
        );

        // jacobian of landmark position with respect to observation
        let g_y = Matrix2::new(
            absolute_angle.cos(),
            -observation.range * absolute_angle.sin(),
            absolute_angle.sin(),
            observation.range * absolute_angle.cos(),
        );

        // sensor noise
        let r = Matrix2::new(
            cfg.est_stdev_range.powi(2),
            0.0,
            0.0,
            cfg.est_stdev_bearing.powi(2),
        );

        // landmark covariance
        let p_ll = g_y * r * g_y.transpose();

        // create and insert the landmark
        let new_landmark = LandmarkEstimate {
            mu: Vector2::new(landmark_x, landmark_y),
            sigma: p_ll,
        };

        self.landmarks.insert(observation.id, new_landmark);
    }

    fn correct_landmark(&mut self, observation: &Observation, cfg: &Config) {
        if let Some(landmark) = self.landmarks.get_mut(&observation.id) {
            // compute distances
            let distance_x = landmark.mu.x - self.x;
            let distance_y = landmark.mu.y - self.y;
            let distance_sq = (distance_x * distance_x) + (distance_y * distance_y);
            let distance = distance_sq.sqrt();

            let (predicted_range, predicted_bearing) =
                absolute_to_relative(self.x, self.y, self.theta, landmark.mu.x, landmark.mu.y);

            let range_difference = observation.range - predicted_range;
            let bearing_difference = f32::atan2(
                (observation.bearing - predicted_bearing).sin(),
                (observation.bearing - predicted_bearing).cos(),
            );

            // innovation vector
            let z = Vector2::new(range_difference, bearing_difference);

            // jacobian with respect to landmark
            let h_l = Matrix2::new(
                distance_x / distance,
                distance_y / distance,
                -distance_y / distance_sq,
                distance_x / distance_sq,
            );

            // sensor noise
            let r = Matrix2::new(
                cfg.est_stdev_range.powi(2),
                0.0,
                0.0,
                cfg.est_stdev_bearing.powi(2),
            );

            // landmark-landmark covariance
            let p_ll = landmark.sigma;

            // innovation matrix
            let z_matrix = h_l * p_ll * h_l.transpose() + r;

            let z_inverse = z_matrix.try_inverse().unwrap();

            // weight update
            let determinant = z_matrix.determinant().max(1e-6);
            let exponent = -0.5 * (z.transpose() * z_inverse * z)[(0, 0)];
            let weight_update =
                (1.0 / (2.0 * std::f32::consts::PI * determinant.sqrt())) * exponent.exp();
            self.weight *= weight_update.max(1e-20);

            // ekf update
            // Kalman gain
            let k = p_ll * h_l.transpose() * z_inverse;

            // update state
            landmark.mu += k * z;

            // update covariance
            landmark.sigma = (Matrix2::identity() - k * h_l) * p_ll;
        }
    }
}

impl FastSlam {
    pub const COLOR: Color = Color::new(1.0, 0.0, 0.0, 0.5);

    pub fn new(num_particles: usize) -> Self {
        let particles = vec![
            Particle {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
                weight: 1.0,
                landmarks: HashMap::new(),
            };
            num_particles
        ];

        Self {
            particles,
            num_particles,
        }
    }

    fn resample(&mut self) {
        let total_weight: f32 = self.particles.iter().map(|particle| particle.weight).sum();

        // safety check for if weights collapsed
        if total_weight < 1e-10 {
            for particle in &mut self.particles {
                particle.weight = 1.0;
            }
            return;
        }

        let mut new_particles = Vec::with_capacity(self.num_particles);
        let step = total_weight / (self.num_particles as f32);
        let mut position = macroquad::rand::gen_range(0.0, step);
        let mut cumulative_weight = 0.0;
        let mut current_index = 0;

        for _ in 0..self.num_particles {
            while position > cumulative_weight + self.particles[current_index].weight {
                cumulative_weight += self.particles[current_index].weight;
                current_index = (current_index + 1) % self.num_particles;
            }

            let mut particle = self.particles[current_index].clone();
            particle.weight = 1.0;
            new_particles.push(particle);
            position += step;
        }
        self.particles = new_particles;
    }
}

impl Slam for FastSlam {
    fn predict(
        &mut self,
        linear_velocity: f32,
        angular_velocity: f32,
        delta_time: f32,
        cfg: &Config,
    ) {
        for particle in &mut self.particles {
            let noisy_linear_velocity = linear_velocity
                + sample_normal(
                    0.0,
                    (cfg.est_stdev_linear * linear_velocity.abs()).max(0.01),
                );
            let noisy_angular_velocity = angular_velocity
                + sample_normal(
                    0.0,
                    (cfg.est_stdev_angular * angular_velocity.abs()).max(0.01),
                );

            let theta_half = particle.theta + 0.5 * noisy_angular_velocity * delta_time;

            // update position estimate
            particle.x += noisy_linear_velocity * delta_time * theta_half.cos();
            particle.y += noisy_linear_velocity * delta_time * theta_half.sin();
            particle.theta += noisy_angular_velocity * delta_time;

            // normalize angle to (-PI, PI]
            particle.theta = f32::atan2(particle.theta.sin(), particle.theta.cos());
        }
    }

    fn update(&mut self, observations: &[Observation], cfg: &Config) {
        for observation in observations {
            for particle in &mut self.particles {
                if particle.landmarks.contains_key(&observation.id) {
                    particle.correct_landmark(observation, cfg);
                } else {
                    particle.initialize_landmark(observation, cfg);
                }
            }
        }
        self.resample();
    }

    fn get_state(&self) -> (f32, f32, f32) {
        let mut x = 0.0;
        let mut y = 0.0;
        let mut dir_x = 0.0;
        let mut dir_y = 0.0;
        let mut total_weight = 0.0;

        for particle in &self.particles {
            x += particle.x * particle.weight;
            y += particle.y * particle.weight;

            dir_x += particle.theta.cos() * particle.weight;
            dir_y += particle.theta.sin() * particle.weight;

            total_weight += particle.weight;
        }

        if total_weight < 1e-10 {
            return (0.0, 0.0, 0.0);
        }

        (x / total_weight, y / total_weight, f32::atan2(dir_y, dir_x))
    }

    fn get_landmarks(&self) -> Vec<(usize, f32, f32)> {
        let mut total_weight = 0.0;
        let mut hashmap: HashMap<usize, (f32, f32)> = std::collections::HashMap::new();
        let mut landmarks = Vec::new();

        for particle in &self.particles {
            total_weight += particle.weight;

            for (id, landmark) in &particle.landmarks {
                hashmap
                    .entry(*id)
                    .and_modify(|(x, y)| {
                        *x += landmark.mu.x * particle.weight;
                        *y += landmark.mu.y * particle.weight;
                    })
                    .or_insert((
                        landmark.mu.x * particle.weight,
                        landmark.mu.y * particle.weight,
                    ));
            }
        }

        for (id, landmark) in &mut hashmap {
            landmarks.push((*id, landmark.0 / total_weight, landmark.1 / total_weight))
        }

        landmarks
    }

    fn color(&self) -> macroquad::prelude::Color {
        Self::COLOR
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    #[test]
    fn test_fast_slam_new_exact_particle_count() {
        let slam = FastSlam::new(50);
        assert_eq!(slam.particles.len(), 50);
        assert_eq!(slam.num_particles, 50);
    }

    #[test]
    fn test_fast_slam_new_initial_unit_weights_and_zero_poses() {
        let slam = FastSlam::new(10);
        for p in &slam.particles {
            assert_eq!(p.weight, 1.0);
            assert_eq!(p.x, 0.0);
            assert_eq!(p.y, 0.0);
            assert_eq!(p.theta, 0.0);
        }
    }

    #[test]
    fn test_fast_slam_new_empty_landmarks_per_particle() {
        let slam = FastSlam::new(5);
        for p in &slam.particles {
            assert!(p.landmarks.is_empty());
        }
    }

    #[test]
    fn test_particle_initialize_landmark_mean_from_geometry() {
        let mut particle = Particle {
            x: 5.0,
            y: 10.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 15.0,
            bearing: 0.0,
        };
        particle.initialize_landmark(&obs, &cfg);
        let lm = particle.landmarks.get(&1).unwrap();
        assert!((lm.mu.x - 20.0).abs() < 1e-4);
        assert!((lm.mu.y - 10.0).abs() < 1e-4);
    }

    #[test]
    fn test_particle_initialize_landmark_covariance_symmetric_positive_definite() {
        let mut particle = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.5,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.2,
        };
        particle.initialize_landmark(&obs, &cfg);
        let lm = particle.landmarks.get(&1).unwrap();
        assert!((lm.sigma[(0, 1)] - lm.sigma[(1, 0)]).abs() < 1e-5);
        let eigen = lm.sigma.symmetric_eigen();
        assert!(eigen.eigenvalues.iter().all(|&e| e > 0.0));
    }

    #[test]
    fn test_particle_correct_landmark_decreases_covariance_trace() {
        let mut particle = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        particle.initialize_landmark(&obs, &cfg);
        let trace_before = particle.landmarks.get(&1).unwrap().sigma.trace();
        particle.correct_landmark(&obs, &cfg);
        let trace_after = particle.landmarks.get(&1).unwrap().sigma.trace();
        assert!(trace_after < trace_before);
    }

    #[test]
    fn test_particle_correct_landmark_bearing_innovation_wraparound() {
        let mut particle = Particle {
            x: 0.0,
            y: 0.0,
            theta: PI - 0.05,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs1 = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        particle.initialize_landmark(&obs1, &cfg);
        let obs2 = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.01,
        };
        particle.correct_landmark(&obs2, &cfg);
        let lm = particle.landmarks.get(&1).unwrap();
        assert!(!lm.mu.x.is_nan() && !lm.mu.y.is_nan());
        assert!(!particle.weight.is_nan() && particle.weight > 0.0);
    }

    #[test]
    fn test_particle_correct_landmark_high_weight_for_accurate_measurement() {
        let mut p_accurate = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let mut p_inaccurate = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        p_accurate.initialize_landmark(&obs, &cfg);
        p_inaccurate.initialize_landmark(&obs, &cfg);
        p_accurate.correct_landmark(&obs, &cfg);
        p_inaccurate.correct_landmark(
            &Observation {
                id: 1,
                range: 25.0,
                bearing: 1.0,
            },
            &cfg,
        );
        assert!(p_accurate.weight > p_inaccurate.weight);
    }

    #[test]
    fn test_particle_correct_landmark_low_weight_for_divergent_measurement() {
        let mut particle = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        particle.initialize_landmark(&obs, &cfg);
        particle.correct_landmark(
            &Observation {
                id: 1,
                range: 100.0,
                bearing: PI,
            },
            &cfg,
        );
        assert!(particle.weight < 1e-4);
    }

    #[test]
    fn test_particle_correct_landmark_avoids_zero_or_nan_weight() {
        let mut particle = Particle {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            weight: 1.0,
            landmarks: HashMap::new(),
        };
        let cfg = Config::default();
        let obs = Observation {
            id: 1,
            range: 10.0,
            bearing: 0.0,
        };
        particle.initialize_landmark(&obs, &cfg);
        particle.correct_landmark(
            &Observation {
                id: 1,
                range: 1000.0,
                bearing: PI,
            },
            &cfg,
        );
        assert!(!particle.weight.is_nan() && particle.weight > 0.0);
    }

    #[test]
    fn test_fast_slam_predict_disperses_particle_poses_with_noise() {
        let cfg = Config::default();
        let mut slam = FastSlam::new(50);
        slam.predict(10.0, 0.5, 1.0, &cfg);
        let min_x = slam
            .particles
            .iter()
            .map(|p| p.x)
            .fold(f32::INFINITY, f32::min);
        let max_x = slam
            .particles
            .iter()
            .map(|p| p.x)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(max_x > min_x);
    }

    #[test]
    fn test_fast_slam_predict_stationary_minimum_dispersion() {
        let cfg = Config::default();
        let mut slam = FastSlam::new(20);
        slam.predict(0.0, 0.0, 0.01, &cfg);
        for p in &slam.particles {
            assert!(p.x.abs() < 0.1);
            assert!(p.y.abs() < 0.1);
        }
    }

    #[test]
    fn test_fast_slam_predict_heading_normalization_across_particles() {
        let cfg = Config::default();
        let mut slam = FastSlam::new(30);
        for p in &mut slam.particles {
            p.theta = PI - 0.1;
        }
        slam.predict(0.0, 2.0, 1.0, &cfg);
        for p in &slam.particles {
            assert!(p.theta > -PI && p.theta <= PI);
        }
    }

    #[test]
    fn test_fast_slam_resample_preserves_exact_particle_count() {
        let mut slam = FastSlam::new(50);
        slam.resample();
        assert_eq!(slam.particles.len(), 50);
    }

    #[test]
    fn test_fast_slam_resample_replicates_dominant_weight_particle() {
        let mut slam = FastSlam::new(10);
        for p in &mut slam.particles {
            p.weight = 1e-8;
        }
        slam.particles[3].weight = 1000.0;
        slam.particles[3].x = 77.0;
        slam.resample();
        let count_dominant = slam
            .particles
            .iter()
            .filter(|p| (p.x - 77.0).abs() < 1e-3)
            .count();
        assert!(count_dominant >= 8);
    }

    #[test]
    fn test_fast_slam_resample_eliminates_negligible_weight_particles() {
        let mut slam = FastSlam::new(10);
        for i in 0..10 {
            slam.particles[i].x = i as f32;
            if i == 0 {
                slam.particles[i].weight = 100.0;
            } else {
                slam.particles[i].weight = 1e-12;
            }
        }
        slam.resample();
        let count_eliminated = slam
            .particles
            .iter()
            .filter(|p| (p.x - 5.0).abs() < 1e-3)
            .count();
        assert_eq!(count_eliminated, 0);
    }

    #[test]
    fn test_fast_slam_resample_resets_all_weights_to_unity() {
        let mut slam = FastSlam::new(20);
        for (i, p) in slam.particles.iter_mut().enumerate() {
            p.weight = (i + 1) as f32;
        }
        slam.resample();
        for p in &slam.particles {
            assert!((p.weight - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn test_fast_slam_resample_weight_collapse_safety_fallback() {
        let mut slam = FastSlam::new(10);
        for p in &mut slam.particles {
            p.weight = 0.0;
        }
        slam.resample();
        assert_eq!(slam.particles.len(), 10);
        for p in &slam.particles {
            assert_eq!(p.weight, 1.0);
        }
    }

    #[test]
    fn test_fast_slam_get_state_weighted_mean_position() {
        let mut slam = FastSlam::new(2);
        slam.particles[0].x = 10.0;
        slam.particles[0].y = 0.0;
        slam.particles[0].weight = 1.0;
        slam.particles[1].x = 20.0;
        slam.particles[1].y = 10.0;
        slam.particles[1].weight = 3.0;
        let (x, y, _) = slam.get_state();
        assert!((x - 17.5).abs() < 1e-4);
        assert!((y - 7.5).abs() < 1e-4);
    }

    #[test]
    fn test_fast_slam_get_state_circular_mean_heading_near_branch_cut() {
        let mut slam = FastSlam::new(2);
        slam.particles[0].theta = PI - 0.1;
        slam.particles[0].weight = 1.0;
        slam.particles[1].theta = -PI + 0.1;
        slam.particles[1].weight = 1.0;
        let (_, _, heading) = slam.get_state();
        assert!((heading.abs() - PI).abs() < 0.15);
    }

    #[test]
    fn test_fast_slam_get_state_zero_weight_safe_fallback() {
        let mut slam = FastSlam::new(5);
        for p in &mut slam.particles {
            p.weight = 0.0;
        }
        let state = slam.get_state();
        assert_eq!(state, (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_fast_slam_get_landmarks_weighted_mean_aggregation() {
        let mut slam = FastSlam::new(2);
        let cfg = Config::default();
        slam.particles[0].weight = 1.0;
        slam.particles[0].initialize_landmark(
            &Observation {
                id: 1,
                range: 10.0,
                bearing: 0.0,
            },
            &cfg,
        );
        slam.particles[1].weight = 3.0;
        slam.particles[1].initialize_landmark(
            &Observation {
                id: 1,
                range: 20.0,
                bearing: 0.0,
            },
            &cfg,
        );
        let landmarks = slam.get_landmarks();
        assert_eq!(landmarks.len(), 1);
        assert_eq!(landmarks[0].0, 1);
        assert!((landmarks[0].1 - 17.5).abs() < 1e-4);
    }
}
