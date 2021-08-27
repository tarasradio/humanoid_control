# humanoid_control
Humanoid robot control experiments based on Kajita et al. experience.

This repository contains walking control system algorithms based on __LIPM__ (Linear Inverted Pendulum Model).
## matlab folder
contains MATLAB sources from Kajita (https://github.com/s-kajita/IntroductionToHumanoidRobotics)
## scripts folder
contains python scripts, based on above matlab sources
- __limp.py__ - contains LIPM sources (CoM trajectoriy generation).
- __foot_trajectories.py__ - contains algorithms to legs trajectories generation.
- __run_limp.py__ - can be used for run LIMP with desired steps params and show trajectories.
- __leg_ik.py__ - contains Inverted Kinamatics sources to finding leg joints positions.
## results folder
contains the resulting files of CoM and both legs trajectories in CSV format.
