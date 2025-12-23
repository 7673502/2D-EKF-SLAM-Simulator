use macroquad::prelude::*;
use ::rand::rng;
use rand_distr::{Normal, Distribution};

/*
 * Constants
 */
const LINEAR_ACC: f32 = 250.0;
const ANGULAR_ACC: f32 = 25.0;
const DECAY_FACTOR: f32 = 0.02; // velocity falloff
const ROBOT_RADIUS: f32 = 24.0;

// speed caps
const MAX_LINEAR_SPEED: f32 = 100.0;
const MAX_ANGULAR_SPEED: f32 = 25.0;

// process noise coefficients
const ALPHA_LINEAR: f32 = 0.05;
const ALPHA_ANGULAR: f32 = 0.01;

fn window_conf() -> Conf {
    Conf {
        window_title: "2D EKF SLAM Simulator".to_owned(),
        window_width: 800,
        window_height: 600,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // position states
    let mut x: f32 = screen_width() / 4.0;
    let mut y: f32 = screen_height() / 2.0; 
    let mut linear_velocity: f32 = 0.0;

    // direction states
    let mut dir: f32 = 0.0;
    let mut angular_velocity: f32 = 0.0;
    
    // noise generation
    let mut rng = rng();
    let normal = Normal::new(0.0, 1.0).unwrap();
    let mut epsilon = |alpha: f32| alpha * normal.sample(&mut rng);
    
    let mut prev_gt_linear_velocity: f32 = 0.0;
    let mut prev_gt_angular_velocity: f32 = 0.0;

    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();

        // keyboard controls
        if is_key_down(KeyCode::Up) {
            linear_velocity += LINEAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            linear_velocity -= LINEAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            angular_velocity += ANGULAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            angular_velocity -= ANGULAR_ACC * delta_time;
        }
        
        // bound velocity
        linear_velocity = linear_velocity.clamp(-MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        angular_velocity = angular_velocity.clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        // apply decay
        linear_velocity *= DECAY_FACTOR.powf(delta_time);
        angular_velocity *= DECAY_FACTOR.powf(delta_time);
        
        let gt_linear_velocity: f32 = linear_velocity + epsilon(ALPHA_LINEAR * linear_velocity.abs());
        let gt_angular_velocity: f32 = angular_velocity + epsilon(ALPHA_ANGULAR * angular_velocity.abs());

        // update direction
        dir += 0.5 * (gt_angular_velocity + prev_gt_angular_velocity) * delta_time;
        dir = (dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        x += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.cos();
        y += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.sin();
        x = x.clamp(0.0 + ROBOT_RADIUS, screen_width() / 2.0 - ROBOT_RADIUS);
        y = y.clamp(0.0 + ROBOT_RADIUS, screen_height() - ROBOT_RADIUS);

        // draw "robot"; segment shows direction
        draw_circle(x, y, ROBOT_RADIUS, BLUE);
        draw_line(x, y, x + ROBOT_RADIUS * dir.cos(), y + ROBOT_RADIUS * dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", x, y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", dir), 25.0, 100.0, 36.0, WHITE);
        

        draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, WHITE);
        
        // needed for calculating x, y, and dir on next frame
        prev_gt_linear_velocity = gt_linear_velocity;
        prev_gt_angular_velocity = gt_angular_velocity;

        next_frame().await
    }
}
