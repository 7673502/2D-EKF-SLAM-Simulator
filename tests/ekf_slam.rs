use slam_simulator::config::Config;
use slam_simulator::simulation::{Landmark, Observation, Robot};
use slam_simulator::slam::{EkfSlam, Slam};
use std::f32::consts::PI;

#[test]
fn test_ekf_slam_straight_trajectory_landmark_stabilization() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        est_stdev_range: 0.5,
        est_stdev_bearing: 0.02,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let landmarks = vec![
        Landmark {
            id: 1,
            x: 20.0,
            y: 5.0,
        },
        Landmark {
            id: 2,
            x: 40.0,
            y: -5.0,
        },
    ];

    robot.linear_velocity = 5.0;
    let dt = 0.1;
    let steps = 60;

    for _ in 0..steps {
        robot.update(dt, &cfg, &[]);
        ekf.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        let obs = robot.sense(&landmarks, &[], &cfg);
        ekf.update(&obs, &cfg);
    }

    let estimated_landmarks = ekf.get_landmarks();
    assert_eq!(estimated_landmarks.len(), 2);

    for (id, ex, ey) in estimated_landmarks {
        let gt = landmarks.iter().find(|l| l.id == id).unwrap();
        assert!((ex - gt.x).abs() < 1.0);
        assert!((ey - gt.y).abs() < 1.0);
    }
}

#[test]
fn test_ekf_slam_circular_trajectory_loop_closure_correction() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        sensor_range: 25.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let landmarks = vec![Landmark {
        id: 0,
        x: 0.0,
        y: 10.0,
    }];

    let initial_obs = robot.sense(&landmarks, &[], &cfg);
    assert_eq!(initial_obs.len(), 1);
    ekf.update(&initial_obs, &cfg);

    let v = 25.0;
    let w = 0.25;
    let dt = 0.1;
    let loop_time = 2.0 * PI / w;
    let total_steps = (loop_time / dt).round() as usize;

    for _ in 0..(total_steps - 15) {
        robot.linear_velocity = v;
        robot.angular_velocity = w;
        robot.update(dt, &cfg, &[]);
        ekf.predict(v, w, dt, &cfg);
    }

    let uncertainty_before_closure = ekf.covariance.fixed_view::<3, 3>(0, 0).trace();

    for _ in 0..30 {
        robot.linear_velocity = v;
        robot.angular_velocity = w;
        robot.update(dt, &cfg, &[]);
        ekf.predict(v, w, dt, &cfg);
        let obs = robot.sense(&landmarks, &[], &cfg);
        if !obs.is_empty() {
            ekf.update(&obs, &cfg);
            break;
        }
    }

    let uncertainty_after_closure = ekf.covariance.fixed_view::<3, 3>(0, 0).trace();
    assert!(uncertainty_after_closure < uncertainty_before_closure);
}

#[test]
fn test_ekf_slam_unobserved_landmarks_retain_cross_covariance() {
    let cfg = Config::default();
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let landmarks = vec![
        Landmark {
            id: 1,
            x: 15.0,
            y: 0.0,
        },
        Landmark {
            id: 2,
            x: 30.0,
            y: 0.0,
        },
    ];

    let obs_both = robot.sense(&landmarks, &[], &cfg);
    ekf.update(&obs_both, &cfg);

    ekf.predict(2.0, 0.0, 1.0, &cfg);
    let obs_one_only = vec![Observation {
        id: obs_both[0].id,
        range: obs_both[0].range,
        bearing: obs_both[0].bearing,
    }];
    ekf.update(&obs_one_only, &cfg);

    let cross_cov = ekf.covariance.view((3, 5), (2, 2));
    let mut has_nonzero = false;
    for r in 0..2 {
        for c in 0..2 {
            if cross_cov[(r, c)].abs() > 1e-6 {
                has_nonzero = true;
            }
        }
    }
    assert!(has_nonzero);
}

#[test]
fn test_ekf_slam_intermittent_sensor_readings_convergence() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let landmarks = vec![Landmark {
        id: 1,
        x: 10.0,
        y: 10.0,
    }];

    robot.linear_velocity = 2.0;
    let dt = 0.1;

    for step in 0..40 {
        robot.update(dt, &cfg, &[]);
        ekf.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        if step % 8 == 0 {
            let obs = robot.sense(&landmarks, &[], &cfg);
            ekf.update(&obs, &cfg);
        }
    }

    let (rx, ry, _) = ekf.get_state();
    assert!((rx - robot.x).abs() < 1.0);
    assert!((ry - robot.y).abs() < 1.0);
}
