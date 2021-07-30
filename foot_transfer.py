import math
import matplotlib.pyplot as plt

def calc_foot_swing(t0, xi, yi, xd, yd, swing_period, lifting_height = 0.1, raising_period = 0.1, lowering_period = 0.1):
  foot_path = { 't':[], 'x':[], 'y':[], 'z':[] }

  t = t0; dt = 0.01
  x = xi; y = yi; z = 0

  dx = (xd - xi) / (swing_period / dt)
  dy = (yd - yi) / (swing_period / dt)

  while t < t0 + swing_period:
    if t < t0 + raising_period:
      phase = (t - t0) / raising_period * math.pi / 2
      z = lifting_height * math.sin(phase)
    elif t < t0 + swing_period - lowering_period:
      z = lifting_height
    else:
      phase = (t - t0 - lowering_period) / lowering_period * math.pi / 2
      z = lifting_height * math.sin(phase - math.pi / 2)

    foot_path['t'].append(t)
    foot_path['x'].append(x)
    foot_path['y'].append(y)
    foot_path['z'].append(z)
    
    x += dx; y += dy
    t += dt

  return foot_path

# foot_path = calc_foot_swing(0, 0, 0, 0.3, 0, 0.8)

# plt.plot(foot_path['t'], foot_path['x'])
# plt.plot(foot_path['t'], foot_path['y'])
# plt.plot(foot_path['t'], foot_path['z'])
# plt.show()