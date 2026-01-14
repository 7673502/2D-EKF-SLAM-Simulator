use macroquad::prelude::*;

mod config; mod simulation;
mod slam;
mod ui;
mod utils;

use config::Config;
use simulation::Landmark;
use slam::{EkfSlam, FastSlam, Slam};

// loads font
const FONT_BYTES: &[u8] = include_bytes!("../assets/fonts/GoogleSansCode-Medium.ttf");

fn window_conf() -> Conf {
    Conf {
        window_title: "2D SLAM Simulator".to_owned(),
        window_width: 800,
        window_height: 600,
        window_resizable: true,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let cfg = Config::default();
    
    // font
    let font = load_ttf_font_from_bytes(FONT_BYTES)
        .unwrap();

    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<Landmark> = Vec::new();

    let mut robot = simulation::Robot::new();
    let mut ekf_slam = EkfSlam::new();
    let mut fast_slam = FastSlam::new(100);

    loop {
        let viewport_height = screen_height();
        let viewport_width = screen_width();
        
        let gt_camera = Camera2D {
            target: vec2(robot.x, robot.y),
            zoom: vec2(2.0 / cfg.horizontal_units, 2.0 / -cfg.horizontal_units * viewport_width / viewport_height),
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
         * simulation rendering
         */
        set_camera(&gt_camera);
        
        // gridlines
        ui::draw_gridlines(robot.x, robot.y, viewport_width, viewport_height, cfg.horizontal_units, cfg.grid_unit);

        // shadows
        ui::draw_robot_shadow(robot.x, robot.y, robot.theta, cfg.robot_radius);
        ui::draw_obstructions_shadows(&obstructions);

        // draw obstructions and landmarks
        ui::draw_obstructions(&obstructions);
        ui::draw_landmarks(&landmarks, cfg.landmark_radius);

        // draw "robot"
        ui::draw_robot(robot.x, robot.y, robot.theta, cfg.robot_radius, BLUE, WHITE);

        // SLAM "ghosts"
        ui::draw_slam_state(&ekf_slam, cfg.robot_radius * 1.5);
        ui::draw_slam_state(&fast_slam, cfg.robot_radius * 1.5);


        // draw landmark estimates
        if cfg.show_landmark_estimates { 
            ui::draw_slam_landmarks(&ekf_slam, cfg.landmark_radius);
            ui::draw_slam_landmarks(&fast_slam, cfg.landmark_radius);
        }

        /*
         * UI
         */
        set_default_camera();
        draw_text_ex("testing", 30.0, 40.0, TextParams { font: Some(&font), font_size: 20, ..Default::default() });

        next_frame().await
    }
}
