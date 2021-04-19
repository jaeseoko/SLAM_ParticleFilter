# SLAM_ParticleFilter

## By Jae seok Oh

### Run Instructions
arguments:

('--path_to_map', default='../data/map/wean.dat')

('--path_to_log', default='../data/log/robotdata1.log')

('--output', default='results')

('--num_particles', default=500, type=int)

('--visualize', action='store_true')

('--belief', action='store_true')

Robotdata gives odometry of the robot and laser measurements in 180 degree field of view.

ex) for running robot log 1, 500 particles

'python3 main.py --visualize'

Particle filter localization on 500 particles.


https://user-images.githubusercontent.com/50928257/115315012-60c8fc80-a1b1-11eb-8a7c-36d5c73f5fb0.mov




