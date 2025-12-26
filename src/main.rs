use macroquad::prelude::*;

mod geometry;
mod config;
mod simulation;
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
    
    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<(f32, f32)> = Vec::new();

    let mut robot = simulation::Robot::new();

    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();
        let (effective_gt_x, effective_gt_y) = geometry::gt_to_screen(robot.x, robot.y);

        // movement
        if is_key_down(KeyCode::Up) {
            robot.linear_velocity += cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            robot.linear_velocity -= cfg.linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            robot.angular_velocity -= cfg.angular_acc * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            robot.angular_velocity += cfg.angular_acc * delta_time;
        }
        
        robot.update(delta_time, &cfg, &obstructions);
        
        // adding landmarks and obstructions
        let effective_mouse_x: f32 = mouse_position().0;
        let effective_mouse_y: f32 = mouse_position().1;
        let (mouse_x, mouse_y) = geometry::screen_to_gt(effective_mouse_x, effective_mouse_y);

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

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            let (effective_rect_x, effective_rect_y) = geometry::gt_to_screen(obstruction.x - obstruction.w / 2.0, obstruction.y + obstruction.h / 2.0);
            draw_rectangle(effective_rect_x, effective_rect_y, obstruction.w, obstruction.h, GRAY);
        }
        for landmark in landmarks.iter() {
            let (effective_landmark_x, effective_landmark_y) = geometry::gt_to_screen(landmark.0, landmark.1);
            draw_circle(effective_landmark_x, effective_landmark_y, cfg.landmark_radius, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(effective_gt_x, effective_gt_y, cfg.robot_radius, BLUE);
        draw_line(effective_gt_x, effective_gt_y, effective_gt_x + cfg.robot_radius * robot.dir.cos(), effective_gt_y - cfg.robot_radius * robot.dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", robot.x, robot.y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", robot.dir), 25.0, 100.0, 36.0, WHITE);
        
        // dividing line between ground truth world and robot's perceived world
        draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, WHITE);

        next_frame().await
    }
}
