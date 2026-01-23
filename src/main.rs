use macroquad::prelude::*;

mod app;
mod config;
mod utils;
mod simulation;
mod slam; 

use app::{hud, renderer, user_settings};
use config::Config;
use user_settings::UserSettings;
use simulation::Landmark;
use slam::{EkfSlam, FastSlam, Slam};

use crate::app::{hud::is_cog_hovered, input};

// loads font
const FONT_BYTES: &[u8] = include_bytes!("../assets/fonts/GoogleSansCode-Medium.ttf");

fn window_conf() -> Conf {
    Conf {
        window_title: "2D SLAM Simulator".to_owned(),
        window_width: 800,
        window_height: 600,
        window_resizable: false,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let cfg = Config::default();

    // settings
    let mut pause = false;
    let mut user_settings: UserSettings = Default::default();
    
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
        /*
         * setup
         */
        let viewport_height = screen_height();
        let viewport_width = screen_width();
        
        let gt_camera = Camera2D {
            target: vec2(robot.x, robot.y),
            zoom: vec2(2.0 / cfg.horizontal_units, 2.0 / -cfg.horizontal_units * viewport_width / viewport_height),
            ..Default::default()
        };
        
        let delta_time: f32 = get_frame_time();

        /*
         * user input
         */
        if is_cog_hovered() && is_mouse_button_released(MouseButton::Left) {
            pause = !pause;
        } else if !pause {
            input::movement_input(&mut robot, &cfg, delta_time);
            input::obstructions_input(&gt_camera, &mut obstructions, &cfg);
            input::landmarks_input(&gt_camera, &mut landmarks, &cfg);
        }
        
        /*
         * update logic
         */
        if !pause {
            // ground truth robot update
            robot.update(delta_time, &cfg, &obstructions);

            // ekf prediction step
            ekf_slam.predict(robot.linear_velocity, robot.angular_velocity, delta_time, &cfg);
            fast_slam.predict(robot.linear_velocity, robot.angular_velocity, delta_time, &cfg);
            
            // ekf correction step
            let observations = robot.sense(&landmarks,&obstructions, &cfg);
            ekf_slam.update(&observations, &cfg);
            fast_slam.update(&observations, &cfg);
        }
        
        /*
         * simulation rendering
         */
        clear_background(Color::new(0.1, 0.1, 0.1, 1.0));
        set_camera(&gt_camera);
        
        // gridlines
        renderer::draw_gridlines(robot.x, robot.y, viewport_width, viewport_height, cfg.horizontal_units, cfg.grid_unit);

        // shadows
        renderer::draw_landmarks_shadows(&landmarks, cfg.landmark_radius);
        renderer::draw_robot_shadow(robot.x, robot.y, cfg.robot_radius);
        renderer::draw_obstructions_shadows(&obstructions);

        // draw obstructions and landmarks
        renderer::draw_obstructions(&obstructions);
        renderer::draw_landmarks(&landmarks, cfg.landmark_radius);

        // draw "robot"
        renderer::draw_robot(robot.x, robot.y, robot.theta, cfg.robot_radius, BLUE, WHITE);

        // SLAM "ghosts"
        renderer::draw_slam_state(&ekf_slam, cfg.robot_radius * 1.5);
        renderer::draw_slam_state(&fast_slam, cfg.robot_radius * 1.5);


        // draw landmark estimates
        if user_settings.show_ekf_state { 
            renderer::draw_slam_landmarks(&ekf_slam, cfg.landmark_radius);
        }

        if user_settings.show_fast_state { 
            renderer::draw_slam_landmarks(&fast_slam, cfg.landmark_radius);
        }

        /*
         * HUD
         */
        set_default_camera();

        if pause { hud::draw_settings(&font, &mut user_settings); }
        hud::draw_legend(&font);
        
        hud::draw_cog();

        next_frame().await
    }
}
