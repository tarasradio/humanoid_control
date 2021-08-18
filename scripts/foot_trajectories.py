import numpy as np
import math
import matplotlib.pyplot as plt

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

def foot_swing_trajectory_magid(t0, xi, yi, xd, yd, step_period, step_height = 0.05):
  trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01
  x = xi; y = yi; z = 0

  step_length = xd - xi

  def next_position(t):
    phase =  (t - t0) / (step_period)
    z = 0.5 * step_height * (1 - math.cos(2 * math.pi * phase))
    x = xi + 0.5 * step_length * (1 - math.cos(math.pi * phase))

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