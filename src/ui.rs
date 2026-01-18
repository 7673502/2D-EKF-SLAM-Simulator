use macroquad::prelude::*;

use crate::simulation::{Landmark};
use crate::slam::{EkfSlam, FastSlam, Slam};

const SHADOW_OFFSET: f32 = 16.0;
const FONT_SIZE: u16 = 20;
const LINE_SPACING: f32 = 30.0;

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
        for i in 0..SHADOW_OFFSET as i32 {
            draw_rectangle(
                obstruction.x - (i as f32),
                obstruction.y - (i as f32),
                obstruction.w,
                obstruction.h,
                Color::new(0.0, 0.0, 0.0, 1.0 / (i as f32 + 1.0))
            );
        }
    }
}

pub fn draw_obstructions(obstructions: &[Rect]) {
    for obstruction in obstructions.iter() {
        draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, GRAY);
    }
}

pub fn draw_landmarks_shadows(landmarks: &[Landmark], landmark_radius: f32) {
    for landmark in landmarks.iter() {
        for i in 0..(SHADOW_OFFSET as i32 / 2) {
            draw_circle(
                landmark.x - (i as f32),
                landmark.y - (i as f32),
                landmark_radius,
                Color::new(0.0, 0.0, 0.0, 1.0 / (i as f32 + 1.0))
            );
        }
    }
}

pub fn draw_landmarks(landmarks: &[Landmark], landmark_radius: f32) {
    for landmark in landmarks.iter() {
        draw_circle(landmark.x, landmark.y, landmark_radius, WHITE);
    }
}

pub fn draw_robot_shadow(x: f32, y: f32, radius: f32) {
    // shadow
    for i in 0..(SHADOW_OFFSET as i32 / 2) {
        draw_circle(
            x - (i as f32),
            y - (i as f32),
            radius,
            Color::new(0.0, 0.0, 0.0, 1.0 / (i as f32 + 1.0))
        );
    }
}

pub fn draw_robot(x: f32, y: f32, theta: f32, radius: f32, fill_color: Color, eye_color: Color) {
    draw_circle(x, y, radius, fill_color);
    draw_circle(x + 0.5 * radius * (theta - 0.8).cos(), y + 0.5 * radius * (theta - 0.8).sin(), radius / 6.0, eye_color);
    draw_circle(x + 0.5 * radius * (theta + 0.8).cos(), y + 0.5 * radius * (theta + 0.8).sin(), radius / 6.0, eye_color);
}

pub fn draw_slam_state(slam: &dyn Slam, radius: f32) {
    let (x, y, theta) = slam.get_state();
    let thickness = radius / 2.0;
    draw_circle_lines(x, y, radius, thickness, slam.color());

    let triangle_radius = 0.3 * thickness; // distance from centroid to vertex
    draw_poly(
        x + (thickness / 2.0 + radius - 0.25 * triangle_radius) * theta.cos(),
        y + (thickness / 2.0 + radius - 0.25 * triangle_radius) * theta.sin(),
        3,
        0.3 * thickness,
        theta.to_degrees(),
        Color::new(0.1, 0.1, 0.1, 0.5)
    );
}

pub fn draw_slam_landmarks(slam: &dyn Slam, radius: f32) {
    for landmark in slam.get_landmarks() {
        draw_circle(landmark.1, landmark.2, radius, slam.color());
    }
}

pub fn draw_legend(font: &Font) {
    let right_offset = screen_width() - 115.0;
    let top_offset = screen_height() - 20.0;


    let algorithms = [
        ("FastSLAM", FastSlam::COLOR),
        ("EKF-SLAM", EkfSlam::COLOR)
    ];

    for (i, (name, color)) in algorithms.iter().enumerate() {
        draw_text_ex(
            name,
            right_offset,
            top_offset - (i as f32) * LINE_SPACING,
            TextParams {
                font: Some(font),
                font_size: FONT_SIZE,
                ..Default::default()
            }
        );
        draw_rectangle(
            right_offset - 25.0,
            top_offset - 17.5 - (i as f32) * LINE_SPACING,
            20.0,
            20.0,
            *color
        );
    }
}

pub fn draw_settings(offset: f32) {
    // rectangle width, height
    let w = 200.0;
    let h = 6.0 * LINE_SPACING;

    draw_rectangle_ex(
        offset + w / 2.0,
        screen_height() / 2.0,
        w,
        h,
        DrawRectangleParams {
            offset: vec2(0.5, 0.5),
            color: Color::new(0.05, 0.05, 0.05, 0.9),
            ..Default::default()
        }
    );

}

