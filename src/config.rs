pub struct Config {
    pub linear_acc: f32,
    pub angular_acc: f32,
    pub robot_radius: f32,

    // speed caps
    pub max_linear_speed: f32,
    pub max_angular_speed: f32,

    // process noise scaling factors
    pub alpha_linear: f32,
    pub alpha_angular: f32,
    
    // decay factor (friction) scalings
    pub drag_linear: f32,
    pub drag_angular: f32,

    // landmark size
    pub landmark_radius: f32,

    // obstruction size
    pub obstruction_width: f32,
    pub obstruction_height: f32,
}

impl Config {
    pub fn default() -> Self {
        Self {
            linear_acc: 64.0,
            angular_acc: 6.0,
            robot_radius: 24.0,
            max_linear_speed: 50.0,
            max_angular_speed: 1.5,
            alpha_linear: 0.03,
            alpha_angular: 0.01,
            drag_linear: 1.5,
            drag_angular: 4.0,
            landmark_radius: 4.0,
            obstruction_width: 50.0,
            obstruction_height: 50.0,
        }
    }
}
