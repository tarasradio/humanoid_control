import numpy as np

import math
import matplotlib.pyplot as plt

from foot_trajectories import foot_swing_trajectory, foot_support_trajectory, foot_swing_trajectory_magid

def lipm(zc, Tsup, sx, sy, xi, yi, vxi, vyi, px, py, a, b):
  # zc, Tsup: z-axis intersection; support time
  # sx, sy: walk parameters(vectors)
  # xi, yi: initial CoM coordinates
  # px, py: initial foot placement
  # vxi, vyi: initial CoM velocity
  # a, b: params of cost-func

  ## Initialization
  n = 0
  g = 9.8

  # the step after the last step should be zero
  sx.append(0)
  sy.append(0) 

  def calc_steps(sx, sy, xi, yi, Tsup):
    steps = { 't':[], 'px':[], 'py':[] }
    n_steps = len(sx)
    n = 0; t = 0.
    px = xi; py = yi

    while n < n_steps:
      steps['t'].append(t)
      ## calculate the desired foot place during the n-th step
      if n != 0:
        px = px + sx[n-1]
        py = py - (-1)**n * sy[n-1]
      steps['px'].append(px)
      steps['py'].append(py)
      t += Tsup
      n += 1
      
    return steps
  
   # Calc steps for all legs
  steps = calc_steps(sx, sy, 0, 0, Tsup)

  # plt.plot(steps['px'], steps['py'], 'x')
  # plt.show()

  # desired foot placement
  px0 = px
  py0 = py

  Tc = math.sqrt(zc / g)
  C = math.cosh(Tsup / Tc)
  S = math.sinh(Tsup / Tc)
  D = a * (C - 1)**2 + b * (S / Tc)**2

  CoM_trajectory = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }
  l_foot_trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }
  r_foot_trajectory = { 't':[], 'x':[], 'y':[], 'z':[] }

  def calc_foot_place(px0, py0, px, py):
    px0 = steps['px'][n]
    py0 = steps['py'][n]

    ## calculate the coordinate (xbar, ybar)
    xbar = sx[n] / 2
    ybar = (-1)**n * sy[n] / 2
    vxbar = (C + 1) / (Tc * S) * xbar
    vybar = (C - 1) / (Tc * S) * ybar
    ## target state of CoM, of the n-th step
    xd = px0 + xbar
    yd = py0 + ybar
    vxd = vxbar
    vyd = vybar
    ## update px, py to be the real foot place in step n
    # a, b are parameters for cost func
    px = - a * (C - 1) / D * (xd - C * xi - Tc * S * vxi) \
        - b * S / (Tc * D) * (vxd - S / Tc * xi - C * vxi)
    py = - a * (C - 1) / D * (yd - C * yi - Tc * S * vyi) \
        - b * S / (Tc * D) * (vyd - S / Tc * yi - C * vyi)

    # plt.plot(px0, py0, 'x')
    # plt.plot(px, py, 'o')

    return px0, py0, px, py

  def calc_foots_path(r_foot_xi = 0, r_foot_yi = 0, l_foot_xi = 0, l_foot_yi = 0.2):
    step = 0
    swing_foot = 'r' # first supported foot

    def extend(path, foot_path):
      path['t'].extend(foot_path['t'])
      path['x'].extend(foot_path['x'])
      path['y'].extend(foot_path['y'])
      path['z'].extend(foot_path['z'])

    lsp = foot_support_trajectory(0, l_foot_xi, l_foot_yi, 0, Tsup)
    rsp = foot_support_trajectory(0, r_foot_xi, r_foot_yi, 0, Tsup)

    extend(r_foot_trajectory, rsp)
    extend(l_foot_trajectory, lsp)

    t_start = 0

    while step < len(steps['t']) - 2:
      t_start = steps['t'][step] + Tsup

      px_start = steps['px'][step]
      py_start = steps['py'][step]
      px_end = steps['px'][step + 2]
      py_end = steps['py'][step + 2]
      
      swing_foot_path = foot_swing_trajectory_magid(t_start, px_start, py_start, px_end, py_end, Tsup, 0.05)

      s_xi = steps['px'][step + 1]
      s_yi = steps['py'][step + 1]
      s_zi = 0

      sfp = foot_support_trajectory(t_start, s_xi, s_yi, s_zi, Tsup)
      
      if swing_foot == 'r':
        extend(r_foot_trajectory, swing_foot_path)
        extend(l_foot_trajectory, sfp)
        swing_foot = 'l'

      elif swing_foot == 'l':
        extend(l_foot_trajectory, swing_foot_path)
        extend(r_foot_trajectory, sfp)
        swing_foot = 'r'

      step += 1
    
    t_start = steps['t'][step] + Tsup

    l_xi = steps['px'][step + 1]; r_xi = steps['px'][step]
    l_yi = steps['py'][step + 1]; r_yi = steps['py'][step]
    l_zi = 0; r_zi = 0

    lsp = foot_support_trajectory(t_start, l_xi, l_yi, l_zi, Tsup)
    rsp = foot_support_trajectory(t_start, r_xi, r_yi, r_zi, Tsup)

    extend(r_foot_trajectory, rsp)
    extend(l_foot_trajectory, lsp)

    # plt.plot(l_foot_path['t'], l_foot_path['x'])
    # plt.plot(l_foot_path['t'], l_foot_path['y'])

    # plt.plot(r_foot_path['t'], r_foot_path['x'])
    # plt.plot(r_foot_path['t'], r_foot_path['y'])
    # plt.show()

  def calc_walk_primitive(ti, dt, xi, vxi, yi, vyi):
    walk_t = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }

    def next_CoM_state(t):
      _t = t / Tc

      def calc_position(p, v, fp):
        return (p - fp) * math.cosh(_t) + Tc * v * math.sinh(_t) + fp
      
      def calc_velocity(p, v, fp):
        return (p - fp) / Tc * math.sinh(_t) + v * math.cosh(_t)

      return  calc_position(xi, vxi, px), \
              calc_velocity(xi, vxi, px), \
              calc_position(yi, vyi, py), \
              calc_velocity(yi, vyi, py), \
              zc, \
              t + ti

    def append_CoM_state(x, vx, y, vy, z, t):
      walk_t['x'].append(x)
      walk_t['vx'].append(vx)
      walk_t['y'].append(y)
      walk_t['vy'].append(vy)
      walk_t['z'].append(zc)
      walk_t['t'].append(t)

    for t in np.arange(0, Tsup, dt):
      state = next_CoM_state(t)
      append_CoM_state(*state)
    final_state = next_CoM_state(Tsup)

    return walk_t, final_state

  T = 0

  px0, py0, px, py = calc_foot_place(px0, py0, px, py) # set the initial placement of foot
  ## carry out the steps indicated by walk parameters sx, sy
  while n < len(sx): # sx was expanded by one element previously
    ## plot the n-th step trajectory
    walk_primitive, final_state = calc_walk_primitive(T, 0.01, xi, vxi, yi, vyi)

    CoM_trajectory['t'].extend(walk_primitive['t'])
    CoM_trajectory['x'].extend(walk_primitive['x'])
    CoM_trajectory['vx'].extend(walk_primitive['vx'])
    CoM_trajectory['y'].extend(walk_primitive['y'])
    CoM_trajectory['vy'].extend(walk_primitive['vy'])
    CoM_trajectory['z'].extend(walk_primitive['z'])

    ## update xi, yi, vxi, vyi for the next step
    xi = final_state[0]
    vxi = final_state[1]
    yi = final_state[2]
    vyi = final_state[3]
    
    ## update n, the order number of the step
    n += 1; 

    if n < len(sx):
      px0, py0, px, py = calc_foot_place(px0, py0, px, py); # calculate the actual foot place for step n
    T += Tsup

  calc_foots_path()

  return CoM_trajectory, l_foot_trajectory, r_foot_trajectory