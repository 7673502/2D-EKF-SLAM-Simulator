use macroquad::prelude::*;

/*
 * Box-Mueller transform to generate normally distributed values
 * https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
 */
pub fn sample_normal(mean: f32, std_dev: f32) -> f32 {
    let u1 = rand::gen_range(0.0f32, 1.0f32).max(1e-6); // don't want to do ln of tiny numbers
    let u2 = rand::gen_range(0.0f32, 1.0f32);

    let z0 = (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();

    mean + std_dev * z0 
}
