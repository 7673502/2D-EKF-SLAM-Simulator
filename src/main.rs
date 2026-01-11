use macroquad::prelude::*;

mod utils;
mod slam;
mod simulation;
mod config;
use config::Config;

use crate::simulation::Landmark;
use crate::slam::{EkfSlam, FastSlam, Slam};


fn window_conf() -> Conf {
    Conf {
        window_title: "2D SLAM Simulator".to_owned(),
        window_width: 800,
        window_height: 600,
        window_resizable: true,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let cfg = Config::default();
    
    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<Landmark> = Vec::new();

    let mut robot = simulation::Robot::new();
    let mut ekf_slam = EkfSlam::new();
    let mut fast_slam = FastSlam::new(1000);

    loop {
        let viewport_height = screen_height();
        let viewport_width = screen_width();
        
        let gt_camera = Camera2D {
            target: vec2(robot.x, robot.y),
            zoom: vec2(2.0 / cfg.horizontal_units, 2.0 / -cfg.horizontal_units * viewport_width / viewport_height),
            viewport: Some((0, 0, viewport_width as i32, viewport_height as i32)),
            ..Default::default()
        };
        
        let delta_time: f32 = get_frame_time();

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
        
        // ground truth robot update
        robot.update(delta_time, &cfg, &obstructions);

        // ekf prediction step
        ekf_slam.predict(robot.linear_velocity, robot.angular_velocity, delta_time, &cfg);
        fast_slam.predict(robot.linear_velocity, robot.angular_velocity, delta_time, &cfg);
        
        // ekf correction step
        let observations = robot.sense(&landmarks,&obstructions, &cfg);
        ekf_slam.update(&observations, &cfg);
        fast_slam.update(&observations, &cfg);

        // adding landmarks and obstructions
        let mouse_screen = mouse_position();
        let mouse_world = gt_camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

        if mouse_screen.0 < viewport_width {
            if is_mouse_button_released(MouseButton::Left) {
                // delete the obstruction if mouse is touching it
                let mut removed = false;
                for i in 0..obstructions.len() {
                    if obstructions[i].contains(mouse_world) {
                        obstructions.remove(i);
                        removed = true;
                        break;
                    }
                }
                if !removed {
                    obstructions.push(
                        Rect::new(
                            mouse_world.x - cfg.obstruction_width / 2.0,
                            mouse_world.y - cfg.obstruction_height / 2.0,
                            cfg.obstruction_width,
                            cfg.obstruction_height
                        )
                    );
                }
            }
            if is_mouse_button_released(MouseButton::Right) {
                let mut removed = false;
                for (i, landmark) in landmarks.iter().enumerate() {
                    if mouse_world.x < landmark.x + cfg.landmark_radius &&
                       mouse_world.x > landmark.x - cfg.landmark_radius &&
                       mouse_world.y < landmark.y + cfg.landmark_radius &&
                       mouse_world.y > landmark.y - cfg.landmark_radius {
                        landmarks.remove(i);
                        removed = true;
                        break;
                    }
                }
                if !removed {
                    let id = landmarks.last().map(|l| l.id + 1).unwrap_or(0);
                    landmarks.push(
                        Landmark {
                            id,
                            x: mouse_world.x, 
                            y: mouse_world.y 
                        }
                    );
                }
            }
        }

        // background
        clear_background(Color::new(0.1, 0.1, 0.1, 1.0));
        
        /*
         * ground truth world
         */
        set_camera(&gt_camera);
        
        // vertical gridlines
        let vertical_units = (cfg.horizontal_units / viewport_width) * viewport_height; // number of units present in viewport vertically
        let num_horizontal = (cfg.horizontal_units / cfg.grid_unit).floor() as i32 + 2;
        let start_vertical_gridline = (robot.x - (robot.x % cfg.grid_unit)) - ((cfg.horizontal_units / 2.0) - ((cfg.horizontal_units / 2.0) % cfg.grid_unit)) - cfg.grid_unit;
        for i in 0..num_horizontal {
            draw_line(
                start_vertical_gridline + cfg.grid_unit * i as f32,
                robot.y + vertical_units / 2.0 + 1.0,
                start_vertical_gridline + cfg.grid_unit * i as f32,
                robot.y - vertical_units / 2.0 - 1.0,
                if (start_vertical_gridline + cfg.grid_unit * i as f32).abs() < 0.1 { 2.0 } else { 1.0 },
                LIGHTGRAY
            );
        }

        // horizontal gridlines
        let num_vertical = (vertical_units / cfg.grid_unit).floor() as i32 + 2;
        let start_horizontal_gridline = (robot.y - (robot.y % cfg.grid_unit)) - ((vertical_units / 2.0) - ((vertical_units / 2.0) % cfg.grid_unit)) - cfg.grid_unit;
        for i in 0..num_vertical {
            draw_line(
                robot.x + cfg.horizontal_units / 2.0 + 1.0,
                start_horizontal_gridline + cfg.grid_unit * i as f32,
                robot.x - cfg.horizontal_units / 2.0 - 1.0,
                start_horizontal_gridline + cfg.grid_unit * i as f32,
                if (start_horizontal_gridline + cfg.grid_unit * i as f32).abs() < 0.1 { 2.0 } else { 1.0 },
                LIGHTGRAY
            );
        }

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, LIGHTGRAY);
            draw_rectangle(obstruction.x + 4.0, obstruction.y + 4.0, obstruction.w - 8.0, obstruction.h - 8.0, GRAY);
        }
        for landmark in landmarks.iter() {
            draw_circle(landmark.x, landmark.y, cfg.landmark_radius, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(robot.x, robot.y, cfg.robot_radius, SKYBLUE);
        draw_circle(robot.x, robot.y, cfg.robot_radius - 2.0, DARKPURPLE);
        draw_line(robot.x, robot.y, robot.x + cfg.robot_radius * robot.theta.cos(), robot.y + cfg.robot_radius * robot.theta.sin(), 2.0, SKYBLUE);

        // draw EKF "ghost"
        let (ekf_x, ekf_y, ekf_dir) = ekf_slam.get_state();
        draw_circle(ekf_x, ekf_y, cfg.robot_radius, ekf_slam.color());
        draw_line(ekf_x, ekf_y, ekf_x + cfg.robot_radius * ekf_dir.cos(), ekf_y + cfg.robot_radius * ekf_dir.sin(), 4.0, Color::new(0.0, 0.0, 0.0, 0.5));

        // EKF landmark estimates
        for landmark in ekf_slam.get_landmarks() {
            draw_circle(landmark.1, landmark.2, cfg.landmark_radius, ekf_slam.color());
        }

        // draw FastSLAM "ghost"
        let (fast_x, fast_y, fast_dir) = fast_slam.get_state();
        draw_circle(fast_x, fast_y, cfg.robot_radius, fast_slam.color());
        draw_line(fast_x, fast_y, fast_x + cfg.robot_radius * fast_dir.cos(), fast_y + cfg.robot_radius * fast_dir.sin(), 4.0, Color::new(0.0, 0.0, 0.0, 0.5));

        next_frame().await
    }
}
