use macroquad::prelude::*;

use crate::simulation::{Landmark};
use crate::slam::Slam;

pub fn draw_gridlines(
    robot_x: f32,
    robot_y: f32,
    viewport_width: f32,
    viewport_height: f32,
    horizontal_units: f32,
    grid_unit: f32
) {
    // vertical gridlines
    let vertical_units = (horizontal_units / viewport_width) * viewport_height; // number of units present in viewport vertically
    let num_horizontal = (horizontal_units / grid_unit).floor() as i32 + 2;
    let start_vertical_gridline = (robot_x - (robot_x % grid_unit)) - ((horizontal_units / 2.0) - ((horizontal_units / 2.0) % grid_unit)) - grid_unit;
    for i in 0..num_horizontal {
        draw_line(
            start_vertical_gridline + grid_unit * i as f32,
            robot_y + vertical_units / 2.0 + 1.0,
            start_vertical_gridline + grid_unit * i as f32,
            robot_y - vertical_units / 2.0 - 1.0,
            if (start_vertical_gridline + grid_unit * i as f32).abs() < 0.1 { 2.0 } else { 1.0 },
            LIGHTGRAY
        );
    }

    // horizontal gridlines
    let num_vertical = (vertical_units / grid_unit).floor() as i32 + 2;
    let start_horizontal_gridline = (robot_y - (robot_y % grid_unit)) - ((vertical_units / 2.0) - ((vertical_units / 2.0) % grid_unit)) - grid_unit;
    for i in 0..num_vertical {
        draw_line(
            robot_x + horizontal_units / 2.0 + 1.0,
            start_horizontal_gridline + grid_unit * i as f32,
            robot_x - horizontal_units / 2.0 - 1.0,
            start_horizontal_gridline + grid_unit * i as f32,
            if (start_horizontal_gridline + grid_unit * i as f32).abs() < 0.1 { 2.0 } else { 1.0 },
            LIGHTGRAY
        );
    }
}

pub fn draw_obstructions(obstructions: &[Rect]) {
    for obstruction in obstructions.iter() {
        draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, LIGHTGRAY);
        draw_rectangle(obstruction.x + 4.0, obstruction.y + 4.0, obstruction.w - 8.0, obstruction.h - 8.0, GRAY);
    }
}

pub fn draw_landmarks(landmarks: &[Landmark], landmark_radius: f32) {
    for landmark in landmarks.iter() {
        draw_circle(landmark.x, landmark.y, landmark_radius, WHITE);
    }
}

pub fn draw_robot(x: f32, y: f32, theta: f32, radius: f32, fill_color: Color, outline_color: Color) {
    // shadow
    draw_circle(x - 5.0, y - 5.0, radius, Color::new(0.0, 0.0, 0.0, 1.0));

    draw_circle(x, y, radius, fill_color);
    draw_circle(x + 0.5 * radius * theta.cos(), y + 0.5 * radius * theta.sin(), radius / 6.0, outline_color);
}

pub fn draw_slam_state(slam: &dyn Slam, radius: f32) {
    let (x, y, theta) = slam.get_state();
    draw_circle(x, y, radius, slam.color());
    draw_line(x, y, x + radius * theta.cos(), y + radius * theta.sin(), 4.0, Color::new(0.0, 0.0, 0.0, 0.5));
}

pub fn draw_slam_landmarks(slam: &dyn Slam, radius: f32) {
    for landmark in slam.get_landmarks() {
        draw_circle(landmark.1, landmark.2, radius, slam.color());
    }
}
