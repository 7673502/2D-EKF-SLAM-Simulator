use macroquad::prelude::*;

use crate::simulation::{Landmark};
use crate::slam::Slam;

const SHADOW_OFFSET: f32 = 15.0;
const SHADOW_COLOR: Color = Color::new(0.0, 0.0, 0.0, 1.0);

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
            1.0,
            Color::new(0.4, 0.4, 0.4, 1.0)
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
            1.0,
            Color::new(0.4, 0.4, 0.4, 1.0)
        );
    }
}

pub fn draw_obstructions_shadows(obstructions: &[Rect]) {
    for obstruction in obstructions.iter() {
        draw_rectangle(obstruction.x - SHADOW_OFFSET, obstruction.y - SHADOW_OFFSET, obstruction.w, obstruction.h, SHADOW_COLOR);
        draw_triangle(
            vec2(obstruction.x - SHADOW_OFFSET, obstruction.y + obstruction.h - SHADOW_OFFSET),
            vec2(obstruction.x, obstruction.y + obstruction.h),
            vec2(obstruction.x, obstruction.y + obstruction.h - SHADOW_OFFSET),
            SHADOW_COLOR
        );
        draw_triangle(
            vec2(obstruction.x + obstruction.w - SHADOW_OFFSET, obstruction.y - SHADOW_OFFSET),
            vec2(obstruction.x + obstruction.w, obstruction.y),
            vec2(obstruction.x + obstruction.w - SHADOW_OFFSET, obstruction.y),
            SHADOW_COLOR
        );
    }
}

pub fn draw_obstructions(obstructions: &[Rect]) {
    for obstruction in obstructions.iter() {
        draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, GRAY);
    }
}

pub fn draw_landmarks(landmarks: &[Landmark], landmark_radius: f32) {
    for landmark in landmarks.iter() {
        draw_circle(landmark.x, landmark.y, landmark_radius, WHITE);
    }
}

pub fn draw_robot_shadow(x: f32, y: f32, theta: f32, radius: f32) {
    // shadow
    draw_rectangle_ex(x - SHADOW_OFFSET / 2.0, y - SHADOW_OFFSET / 2.0, radius, radius * 2.0, DrawRectangleParams { offset: vec2(0.5, 0.5), rotation: std::f32::consts::FRAC_PI_4, color: SHADOW_COLOR });
    draw_circle(x - SHADOW_OFFSET, y - SHADOW_OFFSET, radius, SHADOW_COLOR);
}

pub fn draw_robot(x: f32, y: f32, theta: f32, radius: f32, fill_color: Color, outline_color: Color) {
    draw_circle(x, y, radius, fill_color);
    draw_circle(x + 0.5 * radius * theta.cos(), y + 0.5 * radius * theta.sin(), radius / 6.0, outline_color);
}

pub fn draw_slam_state(slam: &dyn Slam, radius: f32) {
    let (x, y, theta) = slam.get_state();
    draw_circle(x, y, radius, slam.color());
    draw_line(x, y, x + radius * theta.cos(), y + radius * theta.sin(), 1.0, Color::new(0.0, 0.0, 0.0, 0.5));
}

pub fn draw_slam_landmarks(slam: &dyn Slam, radius: f32) {
    for landmark in slam.get_landmarks() {
        draw_circle(landmark.1, landmark.2, radius, slam.color());
    }
}
