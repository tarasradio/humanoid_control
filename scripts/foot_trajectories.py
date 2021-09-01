import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import patches

def foot_swing_trajectory(t0, xi, yi, xd, yd, swing_period, lifting_height = 0.1, raising_period = 0.1, lowering_period = 0.1):
  trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01
  x = xi; y = yi; z = 0

  dx = (xd - xi) / (swing_period / dt)
  dy = (yd - yi) / (swing_period / dt)

  def next_position(t):
    if t < t0 + raising_period:
      phase = (t - t0) / raising_period * math.pi / 2
      z = lifting_height * math.sin(phase)
    elif t < t0 + swing_period - lowering_period:
      z = lifting_height
    else:
      phase = (t - t0 - lowering_period) / lowering_period * math.pi / 2
      z = lifting_height * math.sin(phase - math.pi / 2)

    trajectory['t'].append(t)
    trajectory['x'].append(x)
    trajectory['y'].append(y)
    trajectory['z'].append(z)

  for t in np.arange(0, swing_period, dt):
    next_position(t + t0)
    x += dx; y += dy

  return trajectory

def foot_swing_trajectory_magid(t0, xi, yi, zi, xd, yd, zd, step_period, step_height = 0.05):
  trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01

  step_length = xd - xi
  step_width = yd - yi

  k = math.atan2(zd - zi, step_length)

  def next_position(t):
    phase = (t - t0) / step_period
    
    x = xi + 0.5 * step_length * (1 - math.cos(math.pi * phase))
    y = yi + 0.5 * step_width * (1 - math.cos(math.pi * phase))
    z_h = k * (x - xi) + zi
    z_u = 0.5 * step_height * (1 - math.cos(2 * math.pi * phase))
    z = z_h + z_u

    print('z_h = ', z_h)

    trajectory['t'].append(t)
    trajectory['x'].append(x)
    trajectory['y'].append(y)
    trajectory['z'].append(z)

  for t in np.arange(0, step_period, dt):
    next_position(t + t0)

  return trajectory

def foot_support_trajectory(t0, xi, yi, zi, support_period):
  trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01
  x = xi; y = yi; z = 0

  def next_position(t):
    trajectory['t'].append(t)
    trajectory['x'].append(x)
    trajectory['y'].append(y)
    trajectory['z'].append(z)

  for t in np.arange(0, support_period, dt):
    next_position(t + t0)
  
  return trajectory

def test_foot_swing():
  fig, ax = plt.subplots()
  fig.canvas.set_window_title('Swing trajectory for stairs climbing.')

  plt.title('Swing trajectory for stairs climbing.')
  plt.xlabel('x (m)')
  plt.ylabel('z (m)')

  step_l = 0.3; step_h = 0.1
  stair_h = 0.1

  xi = 0.5; zi = 0.1

  xd = xi + step_l
  zd = zi + stair_h

  swing1 = foot_swing_trajectory_magid(0, xi, 0, zi, xd, 0, zd, 0.8 * 2, step_h)
  swing2 = foot_swing_trajectory_magid(0, xi + step_l, 0, zi + stair_h, xd + step_l, 0, zd + stair_h, 0.8 * 2, step_h)

  first_stairs = patches.Rectangle((xi - step_l / 2, zi - stair_h), step_l, stair_h, linewidth=1, edgecolor='red', facecolor='none', lineStyle='--')
  ax.add_patch(first_stairs)

  second_stairs = patches.Rectangle((xd - step_l / 2, zd - stair_h), step_l, stair_h, linewidth=1, edgecolor='red', facecolor='none', lineStyle='--')
  ax.add_patch(second_stairs)

  fird_stairs = patches.Rectangle((xd + step_l - step_l / 2, zd + stair_h - stair_h), step_l, stair_h, linewidth=1, edgecolor='red', facecolor='none', lineStyle='--')
  ax.add_patch(fird_stairs)

  ax.plot(swing1['x'], swing1['z'])
  ax.plot(swing2['x'], swing2['z'])
  plt.axis("equal")

  plt.show()

test_foot_swing()