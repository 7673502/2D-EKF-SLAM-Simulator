pub struct Config {
    pub linear_acc: f32,
    pub angular_acc: f32,
    pub robot_radius: f32,

    // speed caps
    pub max_linear_speed: f32,
    pub max_angular_speed: f32,

    // process noise scaling factors
    pub real_stdev_linear: f32,
    pub real_stdev_angular: f32,
    
    // sensor constants
    pub sensor_range: f32,
    pub real_stdev_range: f32,
    pub real_stdev_bearing: f32,
    
    // decay factor (friction) scalings
    pub drag_linear: f32,
    pub drag_angular: f32,

    // landmark size
    pub landmark_radius: f32,

    // obstruction size
    pub obstruction_width: f32,
    pub obstruction_height: f32,

    // camera
    pub horizontal_units: f32, // number of units horizontally for camera viewport

    // grid
    pub grid_unit: f32,

    // belief standard deviations for EKF SLAM
    pub est_stdev_linear: f32,
    pub est_stdev_angular: f32,
    pub est_stdev_range: f32,
    pub est_stdev_bearing: f32,
}

impl Config {
    pub fn default() -> Self {
        Self {
            linear_acc: 96.0,
            angular_acc: 6.0,
            robot_radius: 24.0,
            max_linear_speed: 150.0,
            max_angular_speed: 1.5,
            real_stdev_linear: 0.03,
            real_stdev_angular: 0.01,
            sensor_range: 200.0,
            real_stdev_range: 5.0,
            real_stdev_bearing: 0.05,
            drag_linear: 1.9,
            drag_angular: 4.0,
            landmark_radius: 6.0,
            obstruction_width: 50.0,
            obstruction_height: 50.0,
            horizontal_units: 500.0,
            grid_unit: 50.0,
            est_stdev_linear: 0.03,
            est_stdev_angular: 0.01,
            est_stdev_range: 5.0,
            est_stdev_bearing: 0.05,
        }
    }
}
