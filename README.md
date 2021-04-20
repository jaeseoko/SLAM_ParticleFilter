# SLAM_ParticleFilter

## By Jae seok Oh

## Particle Filter process

Robot Ground truth is unknown. We have odometry and landmark measurement data (laser measurements).

- Initialize random particles on the map in a free space (where occupancy probability is zero) for fast convergence.
- Propagate particles forward based on motion model and odometry data.
- Ray cast for each particle and calculate weights by comparing the laser measurements(actual for ground truth) and ray cast measurements (actual for particle).
- Resample particles based on the estimated weights.
- Repeat the process.

### Run Instructions: In PF_python , PF_cpp

Particle filter localization on 500 particles.

https://user-images.githubusercontent.com/50928257/115315012-60c8fc80-a1b1-11eb-8a7c-36d5c73f5fb0.mov




