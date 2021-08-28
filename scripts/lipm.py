import numpy as np

import math
import matplotlib.pyplot as plt

from foot_trajectories import foot_support_trajectory, foot_swing_trajectory_magid
from footsteps_generator import calc_steps

def lipm(zc, Tsup, sx, sy, st, xi, yi, vxi, vyi, px, py, a, b):
  # zc, Tsup: z-axis intersection; support time
  # sx, sy, st: walk parameters(vectors)
  # xi, yi: initial CoM coordinates
  # px, py: initial foot placement
  # vxi, vyi: initial CoM velocity
  # a, b: params of cost-func

  ## Initialization
  n = 0
  g = 9.8
  
  # Calc steps for all legs
  steps = calc_steps(sx, sy, st, 0, 0, Tsup)

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

  def calc_foot_place(px0, py0):
    px0 = steps['px'][n]
    py0 = steps['py'][n]

    ## calculate the coordinate (xbar, ybar)
  
    s = math.sin(math.radians(st[n]))
    c = math.cos(math.radians(st[n]))

    bar_rotation = np.array([[c, -s], [s, c]])
    
    xbar = sx[n] / 2
    ybar = -(-1)**(n-1) * sy[n] / 2

    bar_position = np.array([xbar, ybar])
    
    bar = bar_rotation.dot(bar_position)

    vxbar = (C + 1) / (Tc * S) * bar[0]
    vybar = (C - 1) / (Tc * S) * bar[1]

    ## target state of CoM, of the n-th step
    xd = px0 + bar[0]
    yd = py0 + bar[1]
    vxd = vxbar
    vyd = vybar

    ## update px, py to be the real foot place in step n
    # a, b are parameters for cost func
    px = - (a * (C - 1) / D) * (xd - C * xi - Tc * S * vxi) \
        - b * S / (Tc * D) * (vxd - S / Tc * xi - C * vxi)

    py = - (a * (C - 1) / D) * (yd - C * yi - Tc * S * vyi) \
        - b * S / (Tc * D) * (vyd - S / Tc * yi - C * vyi)

    return (px0, py0, px, py)

  def calc_foots_trajectories(r_foot_xi, r_foot_yi, l_foot_xi, l_foot_yi):
    step = 0
    supported_foot = 'r' # first supported foot

    def extend(trajectory, next_part):
      trajectory['t'].extend(next_part['t'])
      trajectory['x'].extend(next_part['x'])
      trajectory['y'].extend(next_part['y'])
      trajectory['z'].extend(next_part['z'])

    lst = foot_support_trajectory(0, l_foot_xi, l_foot_yi, 0, Tsup)
    rst = foot_support_trajectory(0, r_foot_xi, r_foot_yi, 0, Tsup)

    extend(r_foot_trajectory, rst)
    extend(l_foot_trajectory, lst)

    t_start = 0

    while step < len(steps['t']) - 2:
      t_start = steps['t'][step] + Tsup

      px_start = steps['px'][step]
      py_start = steps['py'][step]
      px_end = steps['px'][step + 2]
      py_end = steps['py'][step + 2]
      
      swing_trajectory = foot_swing_trajectory_magid(t_start, px_start, py_start, px_end, py_end, Tsup, 0.05)

      s_xi = steps['px'][step + 1]
      s_yi = steps['py'][step + 1]
      s_zi = 0

      support_trajectory = foot_support_trajectory(t_start, s_xi, s_yi, s_zi, Tsup)
      
      if supported_foot == 'r':
        extend(r_foot_trajectory, swing_trajectory)
        extend(l_foot_trajectory, support_trajectory)
        supported_foot = 'l'
      elif supported_foot == 'l':
        extend(l_foot_trajectory, swing_trajectory)
        extend(r_foot_trajectory, support_trajectory)
        supported_foot = 'r'

      step += 1
    
    t_start = steps['t'][step] + Tsup

    l_xi = steps['px'][step + 1]; r_xi = steps['px'][step]
    l_yi = steps['py'][step + 1]; r_yi = steps['py'][step]
    l_zi = 0; r_zi = 0

    lst = foot_support_trajectory(t_start, l_xi, l_yi, l_zi, Tsup)
    rst = foot_support_trajectory(t_start, r_xi, r_yi, r_zi, Tsup)

    extend(r_foot_trajectory, rst)
    extend(l_foot_trajectory, lst)

  def calc_walk_primitive(ti, dt, xi, vxi, yi, vyi):
    walk_t = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }

    def next_CoM_trajectory(t):
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

    def append_CoM_trajectory(x, vx, y, vy, z, t):
      walk_t['x'].append(x)
      walk_t['vx'].append(vx)
      walk_t['y'].append(y)
      walk_t['vy'].append(vy)
      walk_t['z'].append(zc)
      walk_t['t'].append(t)

    for t in np.arange(0, Tsup, dt):
      trajectory = next_CoM_trajectory(t)
      append_CoM_trajectory(*trajectory)
    
    final_conditions = next_CoM_trajectory(Tsup)

    return walk_t, final_conditions

  T = 0

  px0, py0, px, py = calc_foot_place(px0, py0) # set the initial placement of foot
  ## carry out the steps indicated by walk parameters sx, sy
  while n < len(sx): # sx was expanded by one element previously
    walk_primitive, final_conditions = calc_walk_primitive(T, 0.01, xi, vxi, yi, vyi)

    CoM_trajectory['t'].extend(walk_primitive['t'])
    CoM_trajectory['x'].extend(walk_primitive['x'])
    CoM_trajectory['vx'].extend(walk_primitive['vx'])
    CoM_trajectory['y'].extend(walk_primitive['y'])
    CoM_trajectory['vy'].extend(walk_primitive['vy'])
    CoM_trajectory['z'].extend(walk_primitive['z'])

    xi = final_conditions[0]
    vxi = final_conditions[1]
    yi = final_conditions[2]
    vyi = final_conditions[3]
    
    n += 1; 

    if n < len(sx):
      px0, py0, px, py = calc_foot_place(px0, py0); # calculate the actual foot place for step n
    T += Tsup
  
  calc_foots_trajectories(steps['px'][0], steps['py'][0], steps['px'][1], steps['py'][1])

  return CoM_trajectory, l_foot_trajectory, r_foot_trajectory, steps