
import matplotlib.pyplot as plt
import numpy as np
import math

def calc_steps(sx, sy, st, xi, yi, Tsup):
  # the step after the last step should be zero
  sx.append(0)
  sy.append(0)
  st.append(60)

  steps = { 't':[], 'px':[], 'py':[], 'pt':[] }
  n_steps = len(sx)
  n = 0; t = 0.
  position = np.array([xi, yi]).T
  # px = xi; py = yi

  while n < n_steps:
    steps['t'].append(t)
    ## calculate the desired foot place during the n-th step
    if n != 0:
      step_params = np.array([sx[n-1], -(-1)**n * sy[n-1]])
      c = math.cos(math.radians(st[n - 1]))
      s = math.sin(math.radians(st[n - 1]))
      step_rotation = np.array([[c, -s], [s, c]])
      position = position + step_rotation.dot(step_params.T)
    steps['px'].append(position[0])
    steps['py'].append(position[1])
    steps['pt'].append(math.radians(st[n - 1]))
    t += Tsup
    n += 1
  
  # plt.title("Координаты точек шагов (стоп) робота.")
  # plt.plot(steps['px'], steps['py'], 'x')
  # plt.show()
    
  return steps

# sx = [0.0, 0.25, 0.25, 0.25, 0.0]
# sy = [0.2, 0.2, 0.2, 0.2, 0.2]
# st = [0, 20, 40, 60, 60]

# calc_steps(sx, sy, st, 0, 0, 0.8)