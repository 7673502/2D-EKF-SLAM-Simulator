use slam_simulator::config::Config;
use slam_simulator::simulation::{Landmark, Robot};
use slam_simulator::slam::{EkfSlam, FastSlam, Slam};

#[test]
fn test_differential_ekf_and_fast_slam_bounds_under_identical_inputs() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let mut fast = FastSlam::new(50);

    let landmarks = vec![
        Landmark {
            id: 1,
            x: 25.0,
            y: 8.0,
        },
        Landmark {
            id: 2,
            x: 50.0,
            y: -8.0,
        },
    ];

    robot.linear_velocity = 6.0;
    let dt = 0.1;

    for step in 0..50 {
        if step % 10 < 5 {
            robot.angular_velocity = 0.05;
        } else {
            robot.angular_velocity = -0.05;
        }

        robot.update(dt, &cfg, &[]);
        ekf.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        fast.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);

        let obs = robot.sense(&landmarks, &[], &cfg);
        ekf.update(&obs, &cfg);
        fast.update(&obs, &cfg);
    }

    let (ekf_x, ekf_y, _) = ekf.get_state();
    let (fast_x, fast_y, _) = fast.get_state();

    let diff = ((ekf_x - fast_x).powi(2) + (ekf_y - fast_y).powi(2)).sqrt();
    assert!(diff < 5.0);

    let dist_ekf_gt = ((ekf_x - robot.x).powi(2) + (ekf_y - robot.y).powi(2)).sqrt();
    let dist_fast_gt = ((fast_x - robot.x).powi(2) + (fast_y - robot.y).powi(2)).sqrt();
    assert!(dist_ekf_gt < 5.0);
    assert!(dist_fast_gt < 5.0);
}

#[test]
fn test_differential_landmark_convergence_both_algorithms() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut ekf = EkfSlam::new();
    let mut fast = FastSlam::new(50);

    let landmarks = vec![
        Landmark {
            id: 1,
            x: 15.0,
            y: 10.0,
        },
        Landmark {
            id: 2,
            x: 35.0,
            y: -10.0,
        },
        Landmark {
            id: 3,
            x: 55.0,
            y: 5.0,
        },
    ];

    robot.linear_velocity = 5.0;
    let dt = 0.1;

    for _ in 0..60 {
        robot.update(dt, &cfg, &[]);
        ekf.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        fast.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);

        let obs = robot.sense(&landmarks, &[], &cfg);
        ekf.update(&obs, &cfg);
        fast.update(&obs, &cfg);
    }

    let ekf_lms = ekf.get_landmarks();
    let fast_lms = fast.get_landmarks();

    assert_eq!(ekf_lms.len(), 3);
    assert_eq!(fast_lms.len(), 3);

    for (id, ex, ey) in ekf_lms {
        let gt = landmarks.iter().find(|l| l.id == id).unwrap();
        assert!((ex - gt.x).abs() < 2.0);
        assert!((ey - gt.y).abs() < 2.0);

        let (_, fx, fy) = fast_lms.iter().find(|l| l.0 == id).unwrap();
        assert!((*fx - gt.x).abs() < 2.0);
        assert!((*fy - gt.y).abs() < 2.0);

        let dist_between = ((ex - fx).powi(2) + (ey - fy).powi(2)).sqrt();
        assert!(dist_between < 2.0);
    }
}
