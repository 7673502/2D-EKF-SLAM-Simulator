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

pub struct Observation {
    pub id: usize,
    pub range: f32,
    pub bearing: f32,
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
        let noisy_linear_velocity = self.linear_velocity + cfg.real_stdev_linear  * self.linear_velocity.abs() * self.normal.sample(&mut self.rng);
        let noisy_angular_velocity = self.angular_velocity + cfg.real_stdev_angular * self.angular_velocity.abs() * self.normal.sample(&mut self.rng);

        // update direction
        self.dir += 0.5 * (noisy_angular_velocity + self.prev_angular_velocity) * delta_time;
        self.dir = f32::atan2(self.dir.sin(), self.dir.cos()); // normalize to (-PI, PI]
        
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
    
    pub fn sense(&mut self, landmarks: &[Landmark], obstructions: &[Rect], cfg: &Config) -> Vec<Observation> {
        let mut observations = Vec::new();
        
        for landmark in landmarks.iter() {
            let distance_x = landmark.x - self.x;
            let distance_y = landmark.y - self.y;
            
            let gt_range = (distance_x * distance_x + distance_y * distance_y).sqrt();
            
            if gt_range < cfg.sensor_range {
                let mut blocked = false; // flag for if current landmark is out of line of sight

                for obstruction in obstructions.iter() {
                    if self.liang_barsky(landmark, obstruction) { 
                        blocked = true;
                        break;
                    }
                }

                if blocked { continue; }

                // absolute angle of landmark from robot
                let absolute_angle = f32::atan2(distance_y, distance_x);
                let relative_angle = absolute_angle - self.dir;
                
                // normalize ground truth bearing to (-PI, PI]
                let gt_bearing = f32::atan2(relative_angle.sin(), relative_angle.cos());
                
                let noisy_range = (gt_range + cfg.real_stdev_range * self.normal.sample(&mut self.rng)).max(0.0);
                let mut noisy_bearing = gt_bearing + cfg.real_stdev_bearing * self.normal.sample(&mut self.rng);
                noisy_bearing = f32::atan2(noisy_bearing.sin(), noisy_bearing.cos()); // normalization
                
                observations.push(
                    Observation {
                        id: landmark.id,
                        range: noisy_range,
                        bearing: noisy_bearing
                    }
                )
            }
        }

        observations
    }

    /*
     * Liang Barsky algorithm to check if segment intersects rectangle
     * https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
     */
    fn liang_barsky(&self, landmark: &Landmark, rect: &Rect) -> bool {
        let x_min = rect.x;
        let y_min = rect.y;
        let x_max = rect.x + rect.w;
        let y_max = rect.y + rect.h;

        let x1 = self.x;
        let y1 = self.y;
        let x2 = landmark.x;
        let y2 = landmark.y;

        let p = [-(x2 - x1), x2 - x1, -(y2 - y1), y2 - y1];
        let q = [x1 - x_min, x_max - x1, y1 - y_min, y_max - y1];

        let mut u1: f32 = 0.0;
        let mut u2: f32 = 1.0;

        for i in 0..4 {
            let p_current = p[i];
            let q_current = q[i];

            if p_current == 0.0 {
                if q_current < 0.0 {
                    return false;
                }
            } else {
                let t = q_current / p_current;

                if p_current < 0.0 {
                    if t > u2 { return false; }
                    if t > u1 { u1 = t; }
                } else {
                    if t < u1 { return false; }
                    if t < u2 { u2 = t; }
                }
            }
        }

        u1 <= u2
    }
}
