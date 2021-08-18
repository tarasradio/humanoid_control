import os
from io import TextIOWrapper

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from lipm import lipm

sx = [0.0, 0.3, 0.3, 0.3, 0.0]
sy = [0.2, 0.2, 0.2, 0.2, 0.2]

CoM_z = 0.28 + 0.28 + 0.102 - 0.231

CoM_t, l_foot_t, r_foot_t = lipm(CoM_z - 0.1, 0.8, sx, sy, 0, 0, 0, 0, 0, 0, 10, 1)

def plot_trajectories():
  fig, ax = plt.subplots()
  
  plt.title("Траектории ц.м. (x) и стоп робота (x, z).")

  plt.xlabel("t, sec") # ось абсцисс
  plt.ylabel("x, meters") # ось ординат

  ax.plot(CoM_t['t'], CoM_t['x'], 'r--', label="CoM (x)")
  ax.plot(l_foot_t['t'], l_foot_t['x'], label="Left foot (x)")
  ax.plot(r_foot_t['t'], r_foot_t['x'], label="Right foot (x)")
  ax.plot(l_foot_t['t'], l_foot_t['z'], label="Left foot (z)")
  ax.plot(r_foot_t['t'], r_foot_t['z'], label="Right foot (z)")

  ax.legend()

  ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'))

  plt.show()


def write_trajectory(f:TextIOWrapper, trajectory):
  for i in range(0, len(trajectory['t'])) :
    f.write(str(trajectory['t'][i]) + '\t')
    f.write(str(trajectory['x'][i]) + '\t')
    f.write(str(trajectory['y'][i]) + '\t')
    f.write(str(trajectory['z'][i]) + '\n')
  f.close()

def save_trajectories():
  # Save trajectories in files
  ## Save CoM trajectory
  f = open('./results/CoM_trajectory.txt', 'w')
  f.write('TIME\tCoM_x\tCoM_y\tCoM_z\n')

  write_trajectory(f, CoM_t)

  ## Save left foot trajectory
  f = open('./results/l_foot_trajectory.txt', 'w')
  f.write('TIME\tl_foot_x\tl_foot_y\tl_foot_z\n')

  write_trajectory(f, l_foot_t)

  ## Save right foot trajectory
  f = open('./results/r_foot_trajectory.txt', 'w')
  f.write('TIME\tr_foot_x\tr_foot_y\tr_foot_z\n')

  write_trajectory(f, r_foot_t)

plot_trajectories()
save_trajectories()