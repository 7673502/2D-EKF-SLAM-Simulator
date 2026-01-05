use macroquad::prelude::*;
use rand_distr::{Normal, Distribution};
use ::rand::{rng, rngs::ThreadRng};

use crate::config::Config;

pub struct Robot {
    pub x: f32,
    pub y: f32,
    pub dir: f32,
    pub linear_velocity: f32,
    pub angular_velocity: f32,
    prev_linear_velocity: f32,
    prev_angular_velocity: f32,
    rng: ThreadRng,
    normal: Normal<f32>,
}

pub struct Landmark {
    pub id: usize,
    pub x: f32,
    pub y: f32,
}

impl Robot {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            dir: 0.0,
            linear_velocity: 0.0,
            angular_velocity: 0.0,
            prev_linear_velocity: 0.0,
            prev_angular_velocity: 0.0,
            rng: rng(),
            normal: Normal::new(0.0, 1.0).unwrap(),
        }
    }

    pub fn update(&mut self, delta_time: f32, cfg: &Config, obstructions: &[Rect]) {
        // bound velocity
        self.linear_velocity = self.linear_velocity.clamp(-cfg.max_linear_speed, cfg.max_linear_speed);
        self.angular_velocity = self.angular_velocity.clamp(-cfg.max_angular_speed, cfg.max_angular_speed);

        // apply decay
        self.linear_velocity *= (-cfg.drag_linear * delta_time).exp();
        self.angular_velocity *= (-cfg.drag_angular * delta_time).exp();
        
        // add noise to velocity; uses separate variable to keep struct's velocities clean
        let noisy_linear_velocity = self.linear_velocity + cfg.alpha_linear  * self.linear_velocity.abs() * self.normal.sample(&mut self.rng);
        let noisy_angular_velocity = self.angular_velocity + cfg.alpha_angular * self.angular_velocity.abs() * self.normal.sample(&mut self.rng);

        // update direction
        self.dir += 0.5 * (noisy_angular_velocity + self.prev_angular_velocity) * delta_time;
        self.dir = (self.dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        self.x += (0.5 * (noisy_linear_velocity + self.prev_linear_velocity) * delta_time) * self.dir.cos();
        self.y += (0.5 * (noisy_linear_velocity + self.prev_linear_velocity) * delta_time) * self.dir.sin();

        // detect obstruction
        for obstruction in obstructions.iter() {
            let closest_x = self.x.clamp(obstruction.x, obstruction.x + obstruction.w);
            let closest_y = self.y.clamp(obstruction.y, obstruction.y + obstruction.h);
            
            // distance from closest point on obstruction to center of robot
            let distance_x = self.x - closest_x;
            let distance_y = self.y - closest_y;
            let distance_sq = distance_x * distance_x + distance_y * distance_y;
            
            if distance_sq < cfg.robot_radius * cfg.robot_radius {
                let distance = distance_sq.sqrt();
                
                if distance > 0.0 {
                    self.x = closest_x + cfg.robot_radius * (distance_x / distance);
                    self.y = closest_y + cfg.robot_radius * (distance_y / distance);
                }
            }
        } 

        // needed for calculating x, y, and dir on next frame
        self.prev_linear_velocity = noisy_linear_velocity;
        self.prev_angular_velocity = noisy_angular_velocity;
    }
}
