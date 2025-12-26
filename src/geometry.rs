use macroquad::prelude::*;

// converts ground truth x and y to graphical coordinates
pub fn gt_to_screen(x: f32, y: f32) -> (f32, f32) {
    (screen_width() / 4.0 + x, screen_height() / 2.0 - y)
}

// converts graphical x and y to ground truth
pub fn screen_to_gt(x: f32, y: f32) -> (f32, f32) {
    (x - screen_width() / 4.0, screen_height() / 2.0 - y)
}
