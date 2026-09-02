use macroquad::prelude::Rect;
use slam_simulator::config::Config;
use slam_simulator::simulation::{Landmark, Robot};

#[test]
fn test_simulation_full_step_kinematics_and_sensing_pipeline() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let landmarks = vec![
        Landmark {
            id: 1,
            x: 50.0,
            y: 0.0,
        },
        Landmark {
            id: 2,
            x: 300.0,
            y: 0.0,
        },
    ];
    let obstructions = vec![Rect::new(100.0, -25.0, 50.0, 50.0)];

    robot.linear_velocity = 20.0;
    let dt = 0.1;

    for _ in 0..10 {
        robot.update(dt, &cfg, &obstructions);
        let obs = robot.sense(&landmarks, &obstructions, &cfg);
        assert_eq!(obs.len(), 1);
        assert_eq!(obs[0].id, 1);
        assert!(obs[0].range > 0.0);
    }

    assert!(robot.x > 0.0);
    assert!(robot.linear_velocity < 20.0);
}

#[test]
fn test_simulation_robot_navigating_dense_obstacle_field() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let obstructions = vec![
        Rect::new(50.0, -50.0, 40.0, 100.0),
        Rect::new(120.0, -20.0, 40.0, 100.0),
        Rect::new(190.0, -50.0, 40.0, 100.0),
    ];

    robot.linear_velocity = 50.0;
    let dt = 0.05;

    for _ in 0..100 {
        robot.update(dt, &cfg, &obstructions);

        for obs in &obstructions {
            let cx = robot.x.clamp(obs.x, obs.x + obs.w);
            let cy = robot.y.clamp(obs.y, obs.y + obs.h);
            let dist_sq = (robot.x - cx).powi(2) + (robot.y - cy).powi(2);
            if dist_sq > 1e-4 {
                let dist = dist_sq.sqrt();
                assert!(dist >= cfg.robot_radius - 1e-2);
            }
        }
        assert!(!robot.x.is_nan() && !robot.y.is_nan());
    }
}

#[test]
fn test_simulation_dynamic_occlusion_and_rediscovery() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let landmark = Landmark {
        id: 1,
        x: 100.0,
        y: 0.0,
    };
    let obstruction = Rect::new(40.0, -20.0, 20.0, 40.0);

    let obs_initial = robot.sense(
        &[Landmark {
            id: 1,
            x: 100.0,
            y: 0.0,
        }],
        &[obstruction],
        &cfg,
    );
    assert!(obs_initial.is_empty());

    robot.y = 80.0;
    let obs_rediscovered = robot.sense(&[landmark], &[obstruction], &cfg);
    assert_eq!(obs_rediscovered.len(), 1);
    assert_eq!(obs_rediscovered[0].id, 1);
}
