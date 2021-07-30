import math
import matplotlib.pyplot as plt

def calc_foot_swing(foot_lifting_height, swing_period, lift_up_period, lift_down_period, t0, x0, y0, xd, yd):
  foot_path = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01
  x = x0; y = y0; z = 0

  dx = (xd - x0) / (swing_period / dt)
  dy = (yd - y0) / (swing_period / dt)

  lifting_period = (lift_up_period + lift_down_period)

  while t < t0 + lift_up_period:
    phase = (t - t0) / lift_up_period * math.pi / 2
    z = foot_lifting_height * math.sin(phase)
    x += dx; y += dy
    foot_path['t'].append(t)
    foot_path['x'].append(x)
    foot_path['y'].append(y)
    foot_path['z'].append(z)
    t += dt
  while t <= t0 + swing_period - lift_down_period:
    z = foot_lifting_height
    x += dx; y += dy
    foot_path['t'].append(t)
    foot_path['x'].append(x)
    foot_path['y'].append(y)
    foot_path['z'].append(z)
    t += dt
  while t < t0 + swing_period:
    phase = (t - t0 - lift_down_period) / lift_down_period * math.pi / 2
    z = foot_lifting_height * math.sin(phase - math.pi / 2)
    x += dx; y += dy
    foot_path['t'].append(t)
    foot_path['x'].append(x)
    foot_path['y'].append(y)
    foot_path['z'].append(z)
    t += dt

  plt.plot(foot_path['t'], foot_path['x'])
  plt.plot(foot_path['t'], foot_path['y'])
  plt.plot(foot_path['t'], foot_path['z'], '.')
  plt.show()

# leg_swing(0.1, 0.8, 0.2, 0.2, 0, 0, 0, 0.3, 0)