use crate::config::Config;
use crate::simulation::{Landmark, Robot};
use macroquad::prelude::*;

pub fn movement_input(robot: &mut Robot, cfg: &Config, delta_time: f32) {
    // movement
    if is_key_down(KeyCode::Up) || is_key_down(KeyCode::W) {
        robot.linear_velocity += cfg.linear_acc * delta_time;
    }
    if is_key_down(KeyCode::Down) || is_key_down(KeyCode::S) {
        robot.linear_velocity -= cfg.linear_acc * delta_time;
    }
    if is_key_down(KeyCode::Right) || is_key_down(KeyCode::D) {
        robot.angular_velocity -= cfg.angular_acc * delta_time;
    }
    if is_key_down(KeyCode::Left) || is_key_down(KeyCode::A) {
        robot.angular_velocity += cfg.angular_acc * delta_time;
    }
}

pub fn obstructions_input(camera: &Camera2D, obstructions: &mut Vec<Rect>, cfg: &Config) {
    let mouse_screen = mouse_position();
    let mouse_world = camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

    if !(is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift))
        && is_mouse_button_released(MouseButton::Left)
    {
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
            obstructions.push(Rect::new(
                mouse_world.x - cfg.obstruction_width / 2.0,
                mouse_world.y - cfg.obstruction_height / 2.0,
                cfg.obstruction_width,
                cfg.obstruction_height,
            ));
        }
    }
}

pub fn landmarks_input(gt_camera: &Camera2D, landmarks: &mut Vec<Landmark>, cfg: &Config) {
    let mouse_screen = mouse_position();
    let mouse_world = gt_camera.screen_to_world(vec2(mouse_screen.0, mouse_screen.1));

    if (is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift))
        && is_mouse_button_released(MouseButton::Left)
    {
        let mut removed = false;
        for (i, landmark) in landmarks.iter().enumerate() {
            if mouse_world.x < landmark.x + cfg.landmark_radius
                && mouse_world.x > landmark.x - cfg.landmark_radius
                && mouse_world.y < landmark.y + cfg.landmark_radius
                && mouse_world.y > landmark.y - cfg.landmark_radius
            {
                landmarks.remove(i);
                removed = true;
                break;
            }
        }
        if !removed {
            let id = landmarks.last().map(|l| l.id + 1).unwrap_or(0);
            landmarks.push(Landmark {
                id,
                x: mouse_world.x,
                y: mouse_world.y,
            });
        }
    }
}

pub fn zoom_input(
    horizontal_units: &mut f32,
    min_horizontal_units: f32,
    max_horizontal_units: f32,
) {
    apply_zoom(
        horizontal_units,
        mouse_wheel().1,
        min_horizontal_units,
        max_horizontal_units,
    );
}

pub(crate) fn apply_zoom(
    horizontal_units: &mut f32,
    wheel_delta: f32,
    min_horizontal_units: f32,
    max_horizontal_units: f32,
) {
    *horizontal_units =
        (*horizontal_units - wheel_delta).clamp(min_horizontal_units, max_horizontal_units);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zoom_input_clamps_to_min_horizontal_units() {
        let mut units = 100.0;
        apply_zoom(&mut units, 0.0, 500.0, 2500.0);
        assert_eq!(units, 500.0);

        let mut units_scrolling = 510.0;
        apply_zoom(&mut units_scrolling, 20.0, 500.0, 2500.0);
        assert_eq!(units_scrolling, 500.0);
    }

    #[test]
    fn test_zoom_input_clamps_to_max_horizontal_units() {
        let mut units = 3000.0;
        apply_zoom(&mut units, 0.0, 500.0, 2500.0);
        assert_eq!(units, 2500.0);

        let mut units_scrolling = 2490.0;
        apply_zoom(&mut units_scrolling, -20.0, 500.0, 2500.0);
        assert_eq!(units_scrolling, 2500.0);
    }
}
