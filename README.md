# 2D EKF SLAM Simulator

This is a work-in-progress Rust implementation of an [EKF-SLAM](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf) simulator.

## To do:
- something explodes when ekf update/correction step runs for a long time (guessing has to do with distance divisions during correction?)
- obstructions blocking landmarks
- use (-π, π] instead of [0, 2π) for angles globally
- good UI
- want to do FastSLAM and GraphSLAM later
