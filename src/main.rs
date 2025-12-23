use macroquad::prelude::*;
use ::rand::rng;
use rand_distr::{Normal, Distribution};

mod config;
use config::Config;

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
    let cfg = Config::default();

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
    
    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<(f32, f32)> = Vec::new();


    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();

        // movement
        if is_key_down(KeyCode::Up) {
            linear_velocity += cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            linear_velocity -= cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            angular_velocity += cfg.angular_acc * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            angular_velocity -= cfg.angular_acc * delta_time;
        }
        
        // adding landmarks and obstructions
        let mouse_x: f32 = mouse_position().0;
        let mouse_y: f32 = mouse_position().1;
        if is_mouse_button_released(MouseButton::Left) {
            // delete the obstruction if mouse is touching it
            let mut removed = false;
            for i in 0..obstructions.len() {
                let obstruction = obstructions[i];
                if mouse_x < obstruction.x + obstruction.w &&
                   mouse_x > obstruction.x &&
                   mouse_y < obstruction.y + obstruction.h &&
                   mouse_y > obstruction.y {
                    obstructions.remove(i);
                    removed = true;                 
                    break;
                }
            }
            if !removed && mouse_x < (screen_width()) / 2.0 - cfg.obstruction_width {
                obstructions.push(Rect::new(mouse_x, mouse_y, cfg.obstruction_width, cfg.obstruction_height));
            }
        }
        if is_mouse_button_released(MouseButton::Right) {
            let mut removed = false;
            for i in 0..landmarks.len() {
                let landmark = landmarks[i];
                if mouse_x < landmark.0 + cfg.landmark_radius &&
                   mouse_x > landmark.0 - cfg.landmark_radius &&
                   mouse_y < landmark.1 + cfg.landmark_radius &&
                   mouse_y > landmark.1 - cfg.landmark_radius {
                    landmarks.remove(i);
                    removed = true;                 
                    break;
                }
            }
            if !removed && mouse_x < screen_width() / 2.0 - cfg.landmark_radius {
                landmarks.push(mouse_position());
            }
        }
        
        // bound velocity
        linear_velocity = linear_velocity.clamp(-cfg.max_linear_speed, cfg.max_linear_speed);
        angular_velocity = angular_velocity.clamp(-cfg.max_angular_speed, cfg.max_angular_speed);

        // apply decay
        linear_velocity *= cfg.decay_factor.powf(delta_time);
        angular_velocity *= cfg.decay_factor.powf(delta_time);
        
        let gt_linear_velocity: f32 = linear_velocity + epsilon(cfg.alpha_linear * linear_velocity.abs());
        let gt_angular_velocity: f32 = angular_velocity + epsilon(cfg.alpha_angular * angular_velocity.abs());

        // update direction
        dir += 0.5 * (gt_angular_velocity + prev_gt_angular_velocity) * delta_time;
        dir = (dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        x += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.cos();
        y += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.sin();
        x = x.clamp(0.0 + cfg.robot_radius, screen_width() / 2.0 - cfg.robot_radius);
        y = y.clamp(0.0 + cfg.robot_radius, screen_height() - cfg.robot_radius);

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, GRAY);
        }
        for landmark in landmarks.iter() {
            draw_circle(landmark.0, landmark.1, cfg.landmark_radius, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(x, y, cfg.robot_radius, BLUE);
        draw_line(x, y, x + cfg.robot_radius * dir.cos(), y + cfg.robot_radius * dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", x, y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", dir), 25.0, 100.0, 36.0, WHITE);
        
        // dividing line between ground truth world and robot's perceived world
        draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, WHITE);


        // needed for calculating x, y, and dir on next frame
        prev_gt_linear_velocity = gt_linear_velocity;
        prev_gt_angular_velocity = gt_angular_velocity;

        next_frame().await
    }
}
