# 2D SLAM Simulator

This is a work-in-progress Rust implementation of a simulator for [EKF-SLAM](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf), [FastSLAM](https://ai.stanford.edu/~koller/Papers/Montemerlo+al:AAAI02.pdf), and GraphSLAM.

## What is SLAM?

[SLAM](https://www.mathworks.com/discovery/slam.html) stands for "simulataneous localization and mapping." This is a problem in which an agent (in our case, a robot) must map its surroundings and determine its position in that map at the same time. This project visualizes a simulated robot along with the estimated poses given by three famous SLAM algorithms: EKF-SLAM, FastSLAM, and GraphSLAM.

## Features

Completed:
- robot in a simulated environment
- EKF-SLAM
- FastSLAM

To do:
- GraphSLAM
- web deployment + Github workflow

## Controls

- <kbd>&uarr;</kbd> <kbd>&darr;</kbd> <kbd>&larr;</kbd> <kbd>&larr;</kbd> movement
- left click: place obstruction
- right click: place landmark
Hit the setting button in the top left to choose which algorithms' position and landmark estimates are visible.

## Project Structure

TODO

## License

[MIT License](LICENSE).

