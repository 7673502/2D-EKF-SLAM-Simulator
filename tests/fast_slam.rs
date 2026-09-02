use slam_simulator::config::Config;
use slam_simulator::simulation::{Landmark, Robot};
use slam_simulator::slam::{FastSlam, Slam};
use std::f32::consts::PI;

#[test]
fn test_fast_slam_straight_trajectory_particle_cluster_tracking() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut slam = FastSlam::new(30);
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

    robot.linear_velocity = 8.0;
    let dt = 0.1;

    for _ in 0..50 {
        robot.update(dt, &cfg, &[]);
        slam.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        let obs = robot.sense(&landmarks, &[], &cfg);
        slam.update(&obs, &cfg);
    }

    let (sx, sy, _) = slam.get_state();
    assert!((sx - robot.x).abs() < 5.0);
    assert!((sy - robot.y).abs() < 5.0);

    for p in &slam.particles {
        assert!((p.x - sx).abs() < 15.0);
        assert!((p.y - sy).abs() < 15.0);
    }
}

#[test]
fn test_fast_slam_loop_closure_resampling_survival() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        sensor_range: 25.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut slam = FastSlam::new(40);
    let landmarks = vec![Landmark {
        id: 0,
        x: 0.0,
        y: 10.0,
    }];

    let initial_obs = robot.sense(&landmarks, &[], &cfg);
    slam.update(&initial_obs, &cfg);

    let v = 20.0;
    let w = 0.4;
    let dt = 0.1;
    let total_steps = ((2.0 * PI / w) / dt).round() as usize;

    for _ in 0..(total_steps - 10) {
        robot.linear_velocity = v;
        robot.angular_velocity = w;
        robot.update(dt, &cfg, &[]);
        slam.predict(v, w, dt, &cfg);
    }

    for _ in 0..20 {
        robot.linear_velocity = v;
        robot.angular_velocity = w;
        robot.update(dt, &cfg, &[]);
        slam.predict(v, w, dt, &cfg);
        let obs = robot.sense(&landmarks, &[], &cfg);
        if !obs.is_empty() {
            slam.update(&obs, &cfg);
            break;
        }
    }

    assert_eq!(slam.particles.len(), 40);
    for p in &slam.particles {
        assert!(!p.x.is_nan() && !p.y.is_nan() && !p.theta.is_nan());
        assert_eq!(p.weight, 1.0);
    }
}

#[test]
fn test_fast_slam_consistency_over_prolonged_trajectory() {
    let cfg = Config {
        real_stdev_linear: 0.0,
        real_stdev_angular: 0.0,
        real_stdev_range: 0.0,
        real_stdev_bearing: 0.0,
        ..Config::default()
    };
    let mut robot = Robot::new();
    let mut slam = FastSlam::new(30);
    let landmarks = vec![
        Landmark {
            id: 1,
            x: 20.0,
            y: 10.0,
        },
        Landmark {
            id: 2,
            x: 60.0,
            y: -10.0,
        },
        Landmark {
            id: 3,
            x: 100.0,
            y: 15.0,
        },
    ];

    robot.linear_velocity = 5.0;
    let dt = 0.1;

    for step in 0..100 {
        if step % 20 < 10 {
            robot.angular_velocity = 0.1;
        } else {
            robot.angular_velocity = -0.1;
        }
        robot.update(dt, &cfg, &[]);
        slam.predict(robot.linear_velocity, robot.angular_velocity, dt, &cfg);
        let obs = robot.sense(&landmarks, &[], &cfg);
        slam.update(&obs, &cfg);
    }

    let (sx, sy, stheta) = slam.get_state();
    assert!(!sx.is_nan() && !sy.is_nan() && !stheta.is_nan());
    assert!((sx - robot.x).abs() < 10.0);
    assert!((sy - robot.y).abs() < 10.0);

    let lms = slam.get_landmarks();
    assert!(!lms.is_empty());
    for (_, lx, ly) in lms {
        assert!(!lx.is_nan() && !ly.is_nan());
    }
}
