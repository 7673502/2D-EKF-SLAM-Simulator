use std::collections::HashMap;
use nalgebra::{Matrix2, Vector2};

use crate::slam::Slam;
use crate::simulation::Observation;
use crate::config::Config;

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

impl FastSlam {
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
}

impl Slam for FastSlam {
    fn predict(&mut self, linear_vel: f32, angular_vel: f32, dt: f32, cfg: &Config) {
        todo!()
    }

    fn update(&mut self, observations: &[Observation], cfg: &Config) {
        todo!()
    }

    fn get_state(&self) -> (f32, f32, f32) {
        todo!()
    }

    fn get_landmarks(&self) -> Vec<(usize, f32, f32)> {
        todo!()
    }

    fn color(&self) -> macroquad::prelude::Color {
        todo!()
    }
}
