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

// converts ground truth x and y to graphical coordinates
fn gt_to_screen(x: f32, y: f32) -> (f32, f32) {
    (screen_width() / 4.0 + x, screen_height() / 2.0 - y)
}

// converts graphical x and y to ground truth
fn screen_to_gt(x: f32, y: f32) -> (f32, f32) {
    (x - screen_width() / 4.0, screen_height() / 2.0 - y)
}

#[macroquad::main(window_conf)]
async fn main() {
    let cfg = Config::default();

    // position states
    let mut gt_x: f32 = 0.0;
    let mut gt_y: f32 = 0.0; 
    let mut gt_linear_velocity: f32 = 0.0;

    // direction states
    let mut gt_dir: f32 = 0.0;
    let mut gt_angular_velocity: f32 = 0.0;
    
    // noise generation
    let mut rng = rng();
    let normal = Normal::new(0.0, 1.0).unwrap();
    let mut epsilon = |alpha: f32, velocity: f32| alpha * velocity * normal.sample(&mut rng);
    
    let mut prev_gt_linear_velocity: f32 = 0.0;
    let mut prev_gt_angular_velocity: f32 = 0.0;
    
    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<(f32, f32)> = Vec::new();


    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();
        let (effective_gt_x, effective_gt_y) = gt_to_screen(gt_x, gt_y);

        // movement
        if is_key_down(KeyCode::Up) {
            gt_linear_velocity += cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            gt_linear_velocity -= cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            gt_angular_velocity -= cfg.angular_acc * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            gt_angular_velocity += cfg.angular_acc * delta_time;
        }
        
        // adding landmarks and obstructions
        let effective_mouse_x: f32 = mouse_position().0;
        let effective_mouse_y: f32 = mouse_position().1;
        let (mouse_x, mouse_y) = screen_to_gt(effective_mouse_x, effective_mouse_y);
        if is_mouse_button_released(MouseButton::Left) {
            // delete the obstruction if mouse is touching it
            let mut removed = false;
            for i in 0..obstructions.len() {
                let obstruction = obstructions[i];
                if mouse_x < obstruction.x + obstruction.w / 2.0 &&
                   mouse_x > obstruction.x - obstruction.w / 2.0 &&
                   mouse_y < obstruction.y + obstruction.h / 2.0 &&
                   mouse_y > obstruction.y - obstruction.h / 2.0 {
                    obstructions.remove(i);
                    removed = true;                 
                    break;
                }
            }
            if !removed && mouse_x < (screen_width()) / 4.0 - cfg.obstruction_width / 2.0 {
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
            if !removed && mouse_x < screen_width() / 4.0 - cfg.landmark_radius {
                landmarks.push((mouse_x, mouse_y));
            }
        }
        
        // bound velocity
        gt_linear_velocity = gt_linear_velocity.clamp(-cfg.max_linear_speed, cfg.max_linear_speed);
        gt_angular_velocity = gt_angular_velocity.clamp(-cfg.max_angular_speed, cfg.max_angular_speed);

        // apply decay
        gt_linear_velocity *= cfg.decay_factor.powf(delta_time);
        gt_angular_velocity *= cfg.decay_factor.powf(delta_time);
        
        let gt_linear_velocity: f32 = gt_linear_velocity + epsilon(cfg.alpha_linear, gt_linear_velocity.abs());
        let gt_angular_velocity: f32 = gt_angular_velocity + epsilon(cfg.alpha_angular, gt_angular_velocity.abs());

        // update direction
        gt_dir += 0.5 * (gt_angular_velocity + prev_gt_angular_velocity) * delta_time;
        gt_dir = (gt_dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        gt_x += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * gt_dir.cos();
        gt_y += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * gt_dir.sin();
        gt_x = gt_x.clamp(cfg.robot_radius - screen_width() / 4.0, screen_width() / 4.0 - cfg.robot_radius);
        gt_y = gt_y.clamp(cfg.robot_radius - screen_height() / 2.0, screen_height() / 2.0 - cfg.robot_radius);

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            let (effective_rect_x, effective_rect_y) = gt_to_screen(obstruction.x - obstruction.w / 2.0, obstruction.y + obstruction.h / 2.0);
            draw_rectangle(effective_rect_x, effective_rect_y, obstruction.w, obstruction.h, GRAY);
        }
        for landmark in landmarks.iter() {
            let (effective_landmark_x, effective_landmark_y) = gt_to_screen(landmark.0, landmark.1);
            draw_circle(effective_landmark_x, effective_landmark_y, cfg.landmark_radius, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(effective_gt_x, effective_gt_y, cfg.robot_radius, BLUE);
        draw_line(effective_gt_x, effective_gt_y, effective_gt_x + cfg.robot_radius * gt_dir.cos(), effective_gt_y - cfg.robot_radius * gt_dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", gt_x, gt_y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", gt_dir), 25.0, 100.0, 36.0, WHITE);
        
        // dividing line between ground truth world and robot's perceived world
        draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, WHITE);


        // needed for calculating x, y, and dir on next frame
        prev_gt_linear_velocity = gt_linear_velocity;
        prev_gt_angular_velocity = gt_angular_velocity;

        next_frame().await
    }
}
