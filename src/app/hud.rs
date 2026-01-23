use macroquad::prelude::*;
use crate::app::user_settings::UserSettings;
use crate::slam::{EkfSlam, FastSlam};
use super::{FONT_SIZE, LINE_SPACING};

const COG_X: f32 = 20.0;
const COG_Y: f32 = 20.0;
const COG_R: f32 = 5.0;
const COG_THICKNESS: f32 = 5.0;

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
                color: LIGHTGRAY,
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

pub fn draw_settings(font: &Font, user_settings: &mut UserSettings) {
    let offset = screen_width() / 4.0;
    let padding = 30.0;

    // panel width, height, position
    let w = 400.0;
    let h = 8.0 * LINE_SPACING;
    let panel_center_x = offset + w / 2.0;
    let panel_center_y = screen_height() / 2.0;

    // text
    let mut text = [
        ("EKF-SLAM State", &mut user_settings.show_ekf_state),
        ("FastSLAM State", &mut user_settings.show_fast_state),
        ("GraphSLAM State", &mut user_settings.show_graph_state),
        ("EKF-SLAM landmarks", &mut user_settings.show_ekf_landmarks),
        ("FastSLAM landmarks", &mut user_settings.show_fast_landmarks),
        ("GraphSLAM landmarks", &mut user_settings.show_graph_landmarks),
    ];

    draw_rectangle_ex(
        panel_center_x,
        panel_center_y,
        w,
        h,
        DrawRectangleParams {
            offset: vec2(0.5, 0.5),
            color: Color::new(0.05, 0.05, 0.05, 0.9),
            ..Default::default()
        }
    );

    draw_text_ex(
        "Visibility Menu",
        offset + padding + 80.0,
        panel_center_y - 2.625 * LINE_SPACING - 7.5,
        TextParams {
            font: Some(font),
            font_size: FONT_SIZE,
            ..Default::default()
        }
    );

    for (i, (label, value)) in text.iter_mut().enumerate() {
        // checkbox position
        let checkbox_x = offset + padding;
        let checkbox_y = panel_center_y - (2.0 - i as f32) * LINE_SPACING;
        let checkbox_size = 20.0;

        // check if hovered
        let (mouse_x, mouse_y) = mouse_position();
        let is_hovered = mouse_x < panel_center_x + w / 2.0 - padding &&
                         mouse_x > checkbox_x - checkbox_size / 2.0 &&
                         mouse_y < checkbox_y + checkbox_size / 2.0 &&
                         mouse_y > checkbox_y - checkbox_size / 2.0;

        if is_hovered && is_mouse_button_released(MouseButton::Left) {
            **value = !**value;
        }

        let color = if is_hovered { WHITE } else { LIGHTGRAY };

        // draw box + label
        if **value {
            draw_line(
                checkbox_x - checkbox_size / 4.0,
                checkbox_y + checkbox_size / 4.0,
                checkbox_x + checkbox_size / 4.0,
                checkbox_y - checkbox_size / 4.0,
                2.0,
                color
            );
        }
        draw_rectangle_lines_ex(
            checkbox_x,
            checkbox_y,
            checkbox_size,
            checkbox_size,
            2.0,
            DrawRectangleParams { 
                offset: vec2(0.5, 0.5),
                color,
                ..Default::default()
            }
        );
        draw_text_ex(
            label,
            offset + padding + checkbox_size,
            panel_center_y - (1.5 - i as f32) * LINE_SPACING - 7.5,
            TextParams {
                font: Some(font),
                font_size: FONT_SIZE,
                color,
                ..Default::default()
            }
        );

    }
}

pub fn draw_cog() {
    let effective_radius = COG_R + COG_THICKNESS;
    let color = if is_cog_hovered() { DARKGRAY } else { LIGHTGRAY };
    
    draw_circle_lines(COG_X, COG_Y, COG_R, COG_THICKNESS, color);

    for i in 0..8 {
        let angle = std::f32::consts::FRAC_PI_4 * (i as f32);
        draw_rectangle_ex(
            COG_X + effective_radius * angle.cos(),
            COG_Y + effective_radius * angle.sin(),
            COG_THICKNESS,
            COG_THICKNESS,
            DrawRectangleParams { 
                offset: vec2(0.5, 0.5),
                rotation: angle,
                color
            }
        )
    }
}

pub fn is_cog_hovered() -> bool {
    let (mouse_x, mouse_y) = mouse_position();
    let effective_radius = COG_R + COG_THICKNESS;

    mouse_x > COG_X - effective_radius &&
    mouse_y > COG_Y - effective_radius &&
    mouse_x < COG_X + effective_radius && 
    mouse_y < COG_Y + effective_radius
}

