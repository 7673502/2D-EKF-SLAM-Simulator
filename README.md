# 2D SLAM Simulator

## [Try out the simulator online in browser!](https://slam.pramodna.com/)

<div align="center">
 <img src="./slam_sim_example.gif" width="75%" />
</div>

This is a Rust implementation of a simulator for [EKF-SLAM](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf) and [FastSLAM](https://ai.stanford.edu/~koller/Papers/Montemerlo+al:AAAI02.pdf) with planned support for GraphSLAM.

## What is SLAM?

[SLAM](https://www.mathworks.com/discovery/slam.html) stands for "simultaneous localization and mapping." This is a problem in which an agent (in our case, a robot) must map its surroundings and determine its position in that map at the same time. This project visualizes a simulated robot along with the estimated poses given by two famous SLAM algorithms: EKF-SLAM and FastSLAM.


## Controls

- <kbd>&uarr;</kbd> <kbd>&darr;</kbd> <kbd>&larr;</kbd> <kbd>&rarr;</kbd>/<kbd>WASD</kbd> - movement
- click - place obstruction
- <kbd>shift</kbd> + click - place landmark
- <kbd>esc</kbd> - enter/exit visibility settings

Hit the setting button in the top left to choose which algorithms' pose and landmark estimates are visible.

## Project Structure
```
.
├── assets/                  # static resources (fonts, images)
└── src/
    ├── app/                 # application layer (view & controller)
    │   ├── hud.rs           # UI overlays, settings menu, and legends
    │   ├── input.rs         # input handling (mouse clicks, zoom, WASD)
    │   ├── mod.rs           # module exports
    │   ├── renderer.rs      # pure rendering functions (draws the state)
    │   └── user_settings.rs # structs for toggling visualization states
    ├── slam/                # SLAM algorithms
    │   ├── ekf.rs           # EKF implementation
    │   ├── fast.rs          # FastSLAM implementation
    │   ├── mod.rs           # module exports
    │   └── trait_def.rs     # shared trait ensuring algorithms have a common API
    ├── config.rs            # central configuration (noise levels, physics constants)
    ├── main.rs              # entry point (game loop & state management)
    ├── simulation.rs        # the model (physics, ground truth robot, collision)
    └── utils.rs             # math helpers (normal distribution, coordinate transforms)
```

## Sources
1. [Simultaneous localization and mapping with the extended Kalman filter](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf)
2. [FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem](https://ai.stanford.edu/~koller/Papers/Montemerlo+al:AAAI02.pdf)

## License

[MIT License](LICENSE).

## Known Limitations & Design Notes

- **Known Data Association**: Landmark correspondence is currently provided directly via ground-truth IDs (`Observation.id = Landmark.id`). Unknown data association using Mahalanobis distance gating ($\chi^2$ test) with Individual Compatibility Nearest Neighbor (ICNN) or Joint Compatibility Branch and Bound (JCBB) is not yet implemented.
- **Uncertainty Visualization**: The Macroquad renderer displays point estimates for the robot and landmarks, but does not yet draw dynamic 2-sigma/3-sigma covariance confidence ellipses derived from the EKF covariance matrix $\mathbf{P}$.
- **FastSLAM Resampling Frequency**: FastSLAM 1.0 currently resamples on every observation frame. Implementing selective resampling triggered by Effective Sample Size ($N_{\text{eff}} < N / 2$) would reduce particle depletion over prolonged runs.
- **Matrix Inversion Resilience**: Innovation matrix inversions currently use `.unwrap()` calls, which can be made more resilient against ill-conditioned measurement matrices.

## Roadmap

- [ ] Add dynamic covariance error ellipses for robot and landmark state estimates in the renderer
- [ ] Implement unknown data association with Mahalanobis distance gating (ICNN / JCBB)
- [ ] Implement selective resampling for FastSLAM based on effective sample size ($N_{\text{eff}}$)
- [ ] Implement FastSLAM 2.0 (incorporating current observations into the proposal distribution)
- [ ] Implement GraphSLAM (pose graph optimization with factor graphs and non-linear least squares)


