use std::collections::HashMap;
use nalgebra::{Matrix2, Vector2};
use macroquad::prelude::Color;

use crate::slam::Slam;
use crate::simulation::Observation;
use crate::config::Config;
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

pub struct FastSlam2 {
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
            observation.bearing
        );

        // jacobian of landmark position with respect to observation
        let g_y = Matrix2::new(
            absolute_angle.cos(), -observation.range * absolute_angle.sin(),
            absolute_angle.sin(), observation.range * absolute_angle.cos()
        );

        // sensor noise
        let r = Matrix2::new(
            cfg.est_stdev_range.powi(2), 0.0,
            0.0, cfg.est_stdev_bearing.powi(2)
        );

        // landmark covariance
        let p_ll = g_y * r * g_y.transpose();

        // create and insert the landmark
        let new_landmark = LandmarkEstimate {
            mu: Vector2::new(landmark_x, landmark_y),
            sigma: p_ll
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

            let (predicted_range, predicted_bearing) = absolute_to_relative(
                self.x,
                self.y,
                self.theta,
                landmark.mu.x,
                landmark.mu.y
            );

            let range_difference = observation.range - predicted_range;
            let bearing_difference = f32::atan2(
                (observation.bearing - predicted_bearing).sin(),
                (observation.bearing - predicted_bearing).cos()
            );

            // innovation vector
            let z = Vector2::new(range_difference, bearing_difference);

            // jacobian with respect to landmark
            let h_l = Matrix2::new(
                distance_x / distance, distance_y / distance,
                -distance_y / distance_sq, distance_x / distance_sq
            );

            // sensor noise
            let r = Matrix2::new(
                cfg.est_stdev_range.powi(2), 0.0,
                0.0, cfg.est_stdev_bearing.powi(2)
            );

            // landmark-landmark covariance
            let p_ll = landmark.sigma;

            // innovation matrix
            let z_matrix = h_l * p_ll * h_l.transpose() + r;

            let z_inverse = z_matrix.try_inverse().unwrap();

            // weight update
            let determinant = z_matrix.determinant().max(1e-6);
            let exponent = -0.5 * (z.transpose() * z_inverse * z)[(0, 0)];
            let weight_update = (1.0 / (2.0 * std::f32::consts::PI * determinant.sqrt())) * exponent.exp();
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

impl FastSlam2 {
    pub const COLOR: Color = Color::new(0.0, 0.0, 1.0, 0.5);

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
            for particle in &mut self.particles { particle.weight = 1.0; }
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
    }
}

impl Slam for FastSlam2 {
    fn predict(&mut self, linear_velocity: f32, angular_velocity: f32, delta_time: f32, cfg: &Config) {
        for particle in &mut self.particles {
            let noisy_linear_velocity = linear_velocity + sample_normal(0.0, (cfg.est_stdev_linear * linear_velocity.abs()).max(0.01));
            let noisy_angular_velocity  = angular_velocity + sample_normal(0.0, (cfg.est_stdev_angular * angular_velocity.abs()).max(0.01));

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

        if total_weight < 1e-10 { return (0.0, 0.0, 0.0); }

        (x / total_weight, y / total_weight, f32::atan2(dir_y, dir_x))
    }

    fn get_landmarks(&self) -> Vec<(usize, f32, f32)> {
        let mut total_weight = 0.0;
        let mut hashmap: HashMap<usize, (f32, f32)> = std::collections::HashMap::new();
        let mut landmarks = Vec::new();

        for particle in &self.particles {
            total_weight += particle.weight;

            for (id, landmark) in &particle.landmarks {
                hashmap.entry(*id)
                    .and_modify(|(x, y)| {
                        *x += landmark.mu.x * particle.weight;
                        *y += landmark.mu.y * particle.weight;
                    })
                    .or_insert((landmark.mu.x * particle.weight, landmark.mu.y * particle.weight));
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
