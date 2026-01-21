use macroquad::prelude::*;
use crate::config::{Config};
use crate::simulation::{Landmark, Robot};

pub fn movement_input(robot: &mut Robot, cfg: &Config, delta_time: f32) {
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
}

pub fn obstructions_input(
    camera: &Camera2D,
    obstructions: &mut Vec<Rect>,
    cfg: &Config)
{
    let mouse_screen = mouse_position();
    let mouse_world = camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

    if is_mouse_button_released(MouseButton::Left) {
        // delete the obstruction if mouse is touching it
        let mut removed = false;
        for i in 0..obstructions.len() {
            if obstructions[i].contains(mouse_world) {
                obstructions.remove(i); removed = true;
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
}

pub fn landmarks_input(
    gt_camera: &Camera2D,
    landmarks: &mut Vec<Landmark>,
    cfg: &Config
) {
    let mouse_screen = mouse_position();
    let mouse_world = gt_camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

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
