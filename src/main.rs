use macroquad::prelude::*;

// constants
const LINEAR_ACC: f32 = 2000.0;
const ANGULAR_ACC: f32 = 25.0;
const MAX_LINEAR_SPEED: f32 = 5000.0;
const MAX_ANGULAR_SPEED: f32 = 25.0;
const DECAY_FACTOR: f32 = 0.05;
const ROBOT_RADIUS: f32 = 48.0;

#[macroquad::main("2D EKF SLAM Simulator")]
async fn main() {
    // position states
    let mut x: f32 = screen_width() / 2.0;
    let mut y: f32 = screen_height() / 2.0; 
    let mut linear_velocity: f32 = 0.0;

    // direction states
    let mut dir: f32 = 0.0;
    let mut angular_velocity: f32 = 0.0;


    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();

        let initial_linear_velocity: f32 = linear_velocity;
        let initial_angular_velocity: f32 = angular_velocity;

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
        
        // update direction
        dir += 0.5 * (angular_velocity + initial_angular_velocity) * delta_time;
        dir = (dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        x += (0.5 * (linear_velocity + initial_linear_velocity) * delta_time) * dir.cos();
        y += (0.5 * (linear_velocity + initial_linear_velocity) * delta_time) * dir.sin();
        x = x.clamp(0.0, screen_width());
        y = y.clamp(0.0, screen_height());

        // draw "robot"; segment shows direction
        draw_circle(x, y, ROBOT_RADIUS, BLUE);
        draw_line(x, y, x + ROBOT_RADIUS * dir.cos(), y + ROBOT_RADIUS * dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", x, y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", dir), 25.0, 100.0, 36.0, WHITE);
        
        // apply decay
        linear_velocity *= DECAY_FACTOR.powf(delta_time);
        angular_velocity *= DECAY_FACTOR.powf(delta_time);

        next_frame().await
    }
}
