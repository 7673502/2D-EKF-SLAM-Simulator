use macroquad::prelude::*;

mod simulation;
mod config;
use config::Config;


fn window_conf() -> Conf {
    Conf {
        window_title: "2D EKF SLAM Simulator".to_owned(),
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
    let mut landmarks: Vec<(f32, f32)> = Vec::new();

    let mut robot = simulation::Robot::new();

    loop {
        let viewport_height = screen_height();
        let viewport_width = screen_width() / 2.0;
        
        let camera = Camera2D {
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
        
        robot.update(delta_time, &cfg, &obstructions);
        
        // adding landmarks and obstructions
        let mouse_screen = mouse_position();
        let mouse_world = camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

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
                for i in 0..landmarks.len() {
                    let landmark = landmarks[i];
                    if mouse_world.x < landmark.0 + cfg.landmark_radius &&
                       mouse_world.x > landmark.0 - cfg.landmark_radius &&
                       mouse_world.y < landmark.1 + cfg.landmark_radius &&
                       mouse_world.y > landmark.1 - cfg.landmark_radius {
                        landmarks.remove(i);
                        removed = true;
                        break;
                    }
                }
                if !removed {
                    landmarks.push((mouse_world.x, mouse_world.y));
                }
            }
        }

        // background
        clear_background(BLACK);
        
        /*
         * ground truth world
         */
        set_camera(&camera);
        
        // vertical gridlines
        let vertical_units = (cfg.horizontal_units / viewport_width) * viewport_height; // number of units present in viewport vertically
        let num_horizontal = (cfg.horizontal_units / cfg.grid_unit).floor() as i32 + 2;
        let start_vertical_gridline = (robot.x - (robot.x % cfg.grid_unit)) - ((cfg.horizontal_units / 2.0) - ((cfg.horizontal_units / 2.0) % cfg.grid_unit));
        for i in 0..num_horizontal {
            draw_line(
                start_vertical_gridline + cfg.grid_unit * i as f32,
                robot.y + vertical_units / 2.0 + 1.0,
                start_vertical_gridline + cfg.grid_unit * i as f32,
                robot.y - vertical_units / 2.0 - 1.0,
                if start_vertical_gridline + cfg.grid_unit * i as f32 == 0.0 {4.0} else {2.0},
                LIGHTGRAY
            );
        }

        // horizontal gridlines
        let num_vertical = (vertical_units / cfg.grid_unit).floor() as i32 + 2;
        let start_horizontal_gridline = (robot.y - (robot.y % cfg.grid_unit)) - ((vertical_units / 2.0) - ((vertical_units / 2.0) % cfg.grid_unit));
        for i in 0..num_vertical {
            draw_line(
                robot.x + cfg.horizontal_units / 2.0 + 1.0,
                start_horizontal_gridline + cfg.grid_unit * i as f32,
                robot.x - cfg.horizontal_units / 2.0 - 1.0,
                start_horizontal_gridline + cfg.grid_unit * i as f32,
                if start_horizontal_gridline + cfg.grid_unit * i as f32 == 0.0 {4.0} else {2.0},
                LIGHTGRAY
            );
        }

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, GRAY);
        }
        for landmark in landmarks.iter() {
            draw_circle(landmark.0, landmark.1, cfg.landmark_radius, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(robot.x, robot.y, cfg.robot_radius, BLUE);
        draw_line(robot.x, robot.y, robot.x + cfg.robot_radius * robot.dir.cos(), robot.y + cfg.robot_radius * robot.dir.sin(), 4.0, WHITE);

        /*
         * UI
         */
        set_default_camera();
        draw_line(viewport_width, 0.0, viewport_width, viewport_height, 4.0, WHITE);
        draw_text(&format!("pos: ({:.0}, {:.0})", robot.x, robot.y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", robot.dir), 25.0, 100.0, 36.0, WHITE);
        draw_text(&format!("lin vel: {:.2}", robot.linear_velocity), 25.0, 150.0, 36.0, WHITE);
        draw_text(&format!("ang vel: {:.2}", robot.angular_velocity), 25.0, 200.0, 36.0, WHITE);

        next_frame().await
    }
}
