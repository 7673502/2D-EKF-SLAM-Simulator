pub struct UserSettings {
    // states
    pub show_ekf_state: bool,
    pub show_fast_state: bool,
    pub show_graph_state: bool,

    // landmark estimates
    pub show_ekf_landmarks: bool,
    pub show_fast_landmarks: bool,
    pub show_graph_landmarks: bool,
}

impl Default for UserSettings {
    fn default() -> Self {
        Self {
            show_ekf_state: true,
            show_fast_state: true,
            show_ekf_landmarks: true,
            show_fast_landmarks: true,
            show_graph_state: true,
            show_graph_landmarks: true,
        }
    }
}

