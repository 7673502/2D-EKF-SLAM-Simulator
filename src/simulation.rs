use crate::config::Config;
use crate::utils::sample_normal;
use macroquad::prelude::*;

pub struct Robot {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub linear_velocity: f32,
    pub angular_velocity: f32,
    prev_linear_velocity: f32,
    prev_angular_velocity: f32,
}

pub struct Observation {
    pub id: usize,
    pub range: f32,
    pub bearing: f32,
}

pub struct Landmark {
    pub id: usize,
    pub x: f32,
    pub y: f32,
}

impl Robot {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_velocity: 0.0,
            angular_velocity: 0.0,
            prev_linear_velocity: 0.0,
            prev_angular_velocity: 0.0,
        }
    }

    pub fn update(&mut self, delta_time: f32, cfg: &Config, obstructions: &[Rect]) {
        // bound velocity
        self.linear_velocity = self
            .linear_velocity
            .clamp(-cfg.max_linear_speed, cfg.max_linear_speed);
        self.angular_velocity = self
            .angular_velocity
            .clamp(-cfg.max_angular_speed, cfg.max_angular_speed);

        // apply decay
        self.linear_velocity *= (-cfg.drag_linear * delta_time).exp();
        self.angular_velocity *= (-cfg.drag_angular * delta_time).exp();

        // add noise to velocity; uses separate variable to keep struct's velocities clean
        let noisy_linear_velocity = self.linear_velocity
            + sample_normal(0.0, cfg.real_stdev_linear * self.linear_velocity.abs());
        let noisy_angular_velocity = self.angular_velocity
            + sample_normal(0.0, cfg.real_stdev_angular * self.angular_velocity.abs());

        // update direction
        self.theta += 0.5 * (noisy_angular_velocity + self.prev_angular_velocity) * delta_time;
        self.theta = f32::atan2(self.theta.sin(), self.theta.cos()); // normalize to (-PI, PI]

        // update position
        self.x += (0.5 * (noisy_linear_velocity + self.prev_linear_velocity) * delta_time)
            * self.theta.cos();
        self.y += (0.5 * (noisy_linear_velocity + self.prev_linear_velocity) * delta_time)
            * self.theta.sin();

        // detect obstruction
        for obstruction in obstructions.iter() {
            let closest_x = self.x.clamp(obstruction.x, obstruction.x + obstruction.w);
            let closest_y = self.y.clamp(obstruction.y, obstruction.y + obstruction.h);

            // distance from closest point on obstruction to center of robot
            let distance_x = self.x - closest_x;
            let distance_y = self.y - closest_y;
            let distance_sq = distance_x * distance_x + distance_y * distance_y;

            if distance_sq < cfg.robot_radius * cfg.robot_radius {
                let distance = distance_sq.sqrt();

                if distance > 0.0 {
                    self.x = closest_x + cfg.robot_radius * (distance_x / distance);
                    self.y = closest_y + cfg.robot_radius * (distance_y / distance);
                }
            }
        }

        // needed for calculating x, y, and dir on next frame
        self.prev_linear_velocity = noisy_linear_velocity;
        self.prev_angular_velocity = noisy_angular_velocity;
    }

    pub fn sense(
        &mut self,
        landmarks: &[Landmark],
        obstructions: &[Rect],
        cfg: &Config,
    ) -> Vec<Observation> {
        let mut observations = Vec::new();

        for landmark in landmarks.iter() {
            let distance_x = landmark.x - self.x;
            let distance_y = landmark.y - self.y;

            let gt_range = (distance_x * distance_x + distance_y * distance_y).sqrt();

            if gt_range < cfg.sensor_range {
                let mut blocked = false; // flag for if current landmark is out of line of sight

                for obstruction in obstructions.iter() {
                    if self.liang_barsky(landmark, obstruction) {
                        blocked = true;
                        break;
                    }
                }

                if blocked {
                    continue;
                }

                // absolute angle of landmark from robot
                let absolute_angle = f32::atan2(distance_y, distance_x);
                let relative_angle = absolute_angle - self.theta;

                // normalize ground truth bearing to (-PI, PI]
                let gt_bearing = f32::atan2(relative_angle.sin(), relative_angle.cos());

                let noisy_range = (gt_range + sample_normal(0.0, cfg.real_stdev_range)).max(0.0);
                let mut noisy_bearing = gt_bearing + sample_normal(0.0, cfg.real_stdev_bearing);
                noisy_bearing = f32::atan2(noisy_bearing.sin(), noisy_bearing.cos()); // normalization

                observations.push(Observation {
                    id: landmark.id,
                    range: noisy_range,
                    bearing: noisy_bearing,
                })
            }
        }

        observations
    }

    /*
     * Liang Barsky algorithm to check if segment intersects rectangle
     * https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
     */
    pub(crate) fn liang_barsky(&self, landmark: &Landmark, rect: &Rect) -> bool {
        let x_min = rect.x;
        let y_min = rect.y;
        let x_max = rect.x + rect.w;
        let y_max = rect.y + rect.h;

        let x1 = self.x;
        let y1 = self.y;
        let x2 = landmark.x;
        let y2 = landmark.y;

        let p = [-(x2 - x1), x2 - x1, -(y2 - y1), y2 - y1];
        let q = [x1 - x_min, x_max - x1, y1 - y_min, y_max - y1];

        let mut u1: f32 = 0.0;
        let mut u2: f32 = 1.0;

        for i in 0..4 {
            let p_current = p[i];
            let q_current = q[i];

            if p_current == 0.0 {
                if q_current < 0.0 {
                    return false;
                }
            } else {
                let t = q_current / p_current;

                if p_current < 0.0 {
                    if t > u2 {
                        return false;
                    }
                    if t > u1 {
                        u1 = t;
                    }
                } else {
                    if t < u1 {
                        return false;
                    }
                    if t < u2 {
                        u2 = t;
                    }
                }
            }
        }

        u1 <= u2
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn noiseless_config() -> Config {
        Config {
            real_stdev_linear: 0.0,
            real_stdev_angular: 0.0,
            real_stdev_range: 0.0,
            real_stdev_bearing: 0.0,
            ..Config::default()
        }
    }

    #[test]
    fn test_robot_new_initializes_zero_state() {
        let robot = Robot::new();
        assert_eq!(robot.x, 0.0);
        assert_eq!(robot.y, 0.0);
        assert_eq!(robot.theta, 0.0);
        assert_eq!(robot.linear_velocity, 0.0);
        assert_eq!(robot.angular_velocity, 0.0);
    }

    #[test]
    fn test_robot_update_clamping_max_positive_linear_speed() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.linear_velocity = 500.0;
        robot.update(0.001, &cfg, &[]);
        assert!(robot.linear_velocity <= cfg.max_linear_speed);
    }

    #[test]
    fn test_robot_update_clamping_max_negative_linear_speed() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.linear_velocity = -500.0;
        robot.update(0.001, &cfg, &[]);
        assert!(robot.linear_velocity >= -cfg.max_linear_speed);
    }

    #[test]
    fn test_robot_update_clamping_angular_speed() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.angular_velocity = 50.0;
        robot.update(0.001, &cfg, &[]);
        assert!(robot.angular_velocity <= cfg.max_angular_speed);

        robot.angular_velocity = -50.0;
        robot.update(0.001, &cfg, &[]);
        assert!(robot.angular_velocity >= -cfg.max_angular_speed);
    }

    #[test]
    fn test_robot_update_exponential_drag_decay_linear() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.linear_velocity = 100.0;
        let dt = 0.2;
        let expected = 100.0 * (-cfg.drag_linear * dt).exp();
        robot.update(dt, &cfg, &[]);
        assert!((robot.linear_velocity - expected).abs() < 1e-4);
    }

    #[test]
    fn test_robot_update_exponential_drag_decay_angular() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.angular_velocity = 1.0;
        let dt = 0.2;
        let expected = 1.0 * (-cfg.drag_angular * dt).exp();
        robot.update(dt, &cfg, &[]);
        assert!((robot.angular_velocity - expected).abs() < 1e-4);
    }

    #[test]
    fn test_robot_update_stationary_zero_velocity_drift_free() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.update(1.0, &cfg, &[]);
        assert_eq!(robot.x, 0.0);
        assert_eq!(robot.y, 0.0);
        assert_eq!(robot.theta, 0.0);
    }

    #[test]
    fn test_robot_update_heading_normalization_to_pi_range() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.theta = PI - 0.1;
        robot.angular_velocity = 1.0;
        robot.prev_angular_velocity = 1.0;
        robot.update(1.0, &cfg, &[]);
        assert!(robot.theta > -PI && robot.theta <= PI);
    }

    #[test]
    fn test_robot_update_midpoint_integration_displacement() {
        let mut cfg = noiseless_config();
        cfg.drag_linear = 0.0;
        cfg.drag_angular = 0.0;
        let mut robot = Robot::new();
        robot.linear_velocity = 10.0;
        robot.prev_linear_velocity = 10.0;
        robot.update(1.0, &cfg, &[]);
        assert!((robot.x - 10.0).abs() < 1e-4);
        assert!(robot.y.abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_no_obstructions_retains_position() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        robot.x = 25.0;
        robot.y = 35.0;
        robot.update(0.01, &cfg, &[]);
        assert!((robot.x - 25.0).abs() < 1e-4);
        assert!((robot.y - 35.0).abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_left_edge_penetration_resolves() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 90.0;
        robot.y = 125.0;
        robot.update(0.0, &cfg, &[rect]);
        assert!((robot.x - (100.0 - cfg.robot_radius)).abs() < 1e-4);
        assert!((robot.y - 125.0).abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_right_edge_penetration_resolves() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 160.0;
        robot.y = 125.0;
        robot.update(0.0, &cfg, &[rect]);
        assert!((robot.x - (150.0 + cfg.robot_radius)).abs() < 1e-4);
        assert!((robot.y - 125.0).abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_top_edge_penetration_resolves() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 125.0;
        robot.y = 90.0;
        robot.update(0.0, &cfg, &[rect]);
        assert!((robot.x - 125.0).abs() < 1e-4);
        assert!((robot.y - (100.0 - cfg.robot_radius)).abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_bottom_edge_penetration_resolves() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 125.0;
        robot.y = 160.0;
        robot.update(0.0, &cfg, &[rect]);
        assert!((robot.x - 125.0).abs() < 1e-4);
        assert!((robot.y - (150.0 + cfg.robot_radius)).abs() < 1e-4);
    }

    #[test]
    fn test_robot_collision_corner_penetration_diagonal_pushout() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 90.0;
        robot.y = 90.0;
        robot.update(0.0, &cfg, &[rect]);
        let expected_offset = cfg.robot_radius / 2.0f32.sqrt();
        assert!((robot.x - (100.0 - expected_offset)).abs() < 1e-3);
        assert!((robot.y - (100.0 - expected_offset)).abs() < 1e-3);
    }

    #[test]
    fn test_robot_collision_center_exactly_at_obstruction_center() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let rect = Rect::new(100.0, 100.0, 50.0, 50.0);
        robot.x = 125.0;
        robot.y = 125.0;
        robot.update(0.0, &cfg, &[rect]);
        assert_eq!(robot.x, 125.0);
        assert_eq!(robot.y, 125.0);
    }

    #[test]
    fn test_liang_barsky_segment_completely_inside_rect() {
        let robot = Robot {
            x: 15.0,
            y: 15.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 25.0,
            y: 25.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_completely_outside_left() {
        let robot = Robot {
            x: 0.0,
            y: 15.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 5.0,
            y: 25.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_completely_outside_right() {
        let robot = Robot {
            x: 35.0,
            y: 15.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 40.0,
            y: 25.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_completely_outside_above() {
        let robot = Robot {
            x: 15.0,
            y: 0.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 25.0,
            y: 5.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_completely_outside_below() {
        let robot = Robot {
            x: 15.0,
            y: 35.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 25.0,
            y: 40.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_traversing_horizontal() {
        let robot = Robot {
            x: 0.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 40.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_traversing_vertical() {
        let robot = Robot {
            x: 20.0,
            y: 0.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 20.0,
            y: 40.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_traversing_diagonal() {
        let robot = Robot {
            x: 0.0,
            y: 0.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 40.0,
            y: 40.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_starting_inside_ending_outside() {
        let robot = Robot {
            x: 20.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 50.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_starting_outside_ending_inside() {
        let robot = Robot {
            x: 0.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 20.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_parallel_outside_rect() {
        let robot = Robot {
            x: 0.0,
            y: 5.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 50.0,
            y: 5.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_parallel_inside_rect() {
        let robot = Robot {
            x: 12.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 28.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_segment_collinear_grazing_rect_boundary() {
        let robot = Robot {
            x: 0.0,
            y: 10.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 40.0,
            y: 10.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_degenerate_point_inside_rect() {
        let robot = Robot {
            x: 20.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 20.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_degenerate_point_outside_rect() {
        let robot = Robot {
            x: 5.0,
            y: 5.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 5.0,
            y: 5.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_liang_barsky_infinite_line_intersects_but_finite_segment_misses() {
        let robot = Robot {
            x: 0.0,
            y: 20.0,
            ..Robot::new()
        };
        let landmark = Landmark {
            id: 0,
            x: 5.0,
            y: 20.0,
        };
        let rect = Rect::new(10.0, 10.0, 20.0, 20.0);
        assert!(!robot.liang_barsky(&landmark, &rect));
    }

    #[test]
    fn test_sense_detects_landmark_within_sensor_range() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let landmarks = vec![Landmark {
            id: 1,
            x: 50.0,
            y: 0.0,
        }];
        let obs = robot.sense(&landmarks, &[], &cfg);
        assert_eq!(obs.len(), 1);
        assert_eq!(obs[0].id, 1);
    }

    #[test]
    fn test_sense_ignores_landmark_beyond_sensor_range() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let landmarks = vec![Landmark {
            id: 1,
            x: cfg.sensor_range + 50.0,
            y: 0.0,
        }];
        let obs = robot.sense(&landmarks, &[], &cfg);
        assert!(obs.is_empty());
    }

    #[test]
    fn test_sense_ignores_landmark_occluded_by_obstruction() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let landmarks = vec![Landmark {
            id: 1,
            x: 100.0,
            y: 0.0,
        }];
        let rect = Rect::new(40.0, -10.0, 20.0, 20.0);
        let obs = robot.sense(&landmarks, &[rect], &cfg);
        assert!(obs.is_empty());
    }

    #[test]
    fn test_sense_returns_multiple_unoccluded_landmarks() {
        let cfg = noiseless_config();
        let mut robot = Robot::new();
        let landmarks = vec![
            Landmark {
                id: 1,
                x: 30.0,
                y: 0.0,
            },
            Landmark {
                id: 2,
                x: 0.0,
                y: 40.0,
            },
        ];
        let obs = robot.sense(&landmarks, &[], &cfg);
        assert_eq!(obs.len(), 2);
    }

    #[test]
    fn test_sense_bearing_normalization_within_bounds() {
        let cfg = Config::default();
        let mut robot = Robot {
            theta: 2.5,
            ..Robot::new()
        };
        let landmarks = vec![Landmark {
            id: 1,
            x: -50.0,
            y: -50.0,
        }];
        let obs = robot.sense(&landmarks, &[], &cfg);
        assert_eq!(obs.len(), 1);
        assert!(obs[0].bearing > -PI && obs[0].bearing <= PI);
    }

    #[test]
    fn test_sense_noiseless_observation_matches_ground_truth() {
        let cfg = noiseless_config();
        let mut robot = Robot {
            x: 10.0,
            y: 10.0,
            theta: 0.0,
            ..Robot::new()
        };
        let landmarks = vec![Landmark {
            id: 42,
            x: 40.0,
            y: 50.0,
        }];
        let obs = robot.sense(&landmarks, &[], &cfg);
        assert_eq!(obs.len(), 1);
        let expected_range = (30.0f32.powi(2) + 40.0f32.powi(2)).sqrt();
        let expected_bearing = 40.0f32.atan2(30.0);
        assert!((obs[0].range - expected_range).abs() < 1e-4);
        assert!((obs[0].bearing - expected_bearing).abs() < 1e-4);
    }
}
