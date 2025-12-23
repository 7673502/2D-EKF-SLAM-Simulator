pub struct Config {
    pub linear_acc: f32,
    pub angular_acc: f32,
    pub decay_factor: f32,
    pub robot_radius: f32,

    // speed caps
    pub max_linear_speed: f32,
    pub max_angular_speed: f32,

    // process noise coefficients
    pub alpha_linear: f32,
    pub alpha_angular: f32,

    // landmark size
    pub landmark_radius: f32,

    // obstruction size
    pub obstruction_width: f32,
    pub obstruction_height: f32,
}

impl Config {
    pub fn default() -> Self {
        Self {
            linear_acc: 250.0,
            angular_acc: 25.0,
            decay_factor: 0.02,
            robot_radius: 24.0,
            max_linear_speed: 100.0,
            max_angular_speed: 25.0,
            alpha_linear: 0.05,
            alpha_angular: 0.01,
            landmark_radius: 4.0,
            obstruction_width: 50.0,
            obstruction_height: 50.0,
        }
    }
}
