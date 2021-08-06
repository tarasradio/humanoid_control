import math
import matplotlib.pyplot as plt

from foot_transfer import calc_foot_swing

def LIP(zc, Tsup, sx, sy, xi, yi, vxi, vyi, px, py, a, b):
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

  CoM_path = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }
  l_foot_path = { 't':[], 'x':[], 'y':[], 'z':[] }
  r_foot_path = { 't':[], 'x':[], 'y':[], 'z':[] }

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

    plt.plot(px0, py0, 'x')
    plt.plot(px, py, 'o')

    return px0, py0, px, py

  def calc_foots_path(r_foot_xi = 0, r_foot_yi = 0, l_foot_xi = 0, l_foot_yi = 0.2):
    step = 0
    swing_foot = 'r' # first supported foot

    def calc_support_foot_path(t0, x0, y0, z0):
      sfp = {'t':[], 'x':[], 'y':[], 'z':[]}
      t = 0
      while(t < Tsup):
        sfp['t'].append(t + t0)
        sfp['x'].append(x0)
        sfp['y'].append(y0)
        sfp['z'].append(z0)
        t += 0.01
      return sfp
    def extend(path, foot_path):
      path['t'].extend(foot_path['t'])
      path['x'].extend(foot_path['x'])
      path['y'].extend(foot_path['y'])
      path['z'].extend(foot_path['z'])

    lsp = calc_support_foot_path(0, l_foot_xi, l_foot_yi, 0)
    rsp = calc_support_foot_path(0, r_foot_xi, r_foot_yi, 0)

    extend(r_foot_path, rsp)
    extend(l_foot_path, lsp)
    t_start = 0
    while step < 4:
      t_start = steps['t'][step] + Tsup
      px_start = steps['px'][step]
      py_start = steps['py'][step]
      px_end = steps['px'][step + 2]
      py_end = steps['py'][step + 2]
      
      swing_foot_path = calc_foot_swing(t0 = t_start, xi = px_start, yi = py_start, xd = px_end, yd = py_end, swing_period = Tsup)
      sfp = calc_support_foot_path(t_start, steps['px'][step + 1], steps['py'][step + 1], 0)
      
      if swing_foot == 'r':
        extend(r_foot_path, swing_foot_path)
        extend(l_foot_path, sfp)
        swing_foot = 'l'

      elif swing_foot == 'l':
        extend(l_foot_path, swing_foot_path)
        extend(r_foot_path, sfp)
        swing_foot = 'r'

      step += 1
    
    t_start = steps['t'][step] + Tsup
    lsp = calc_support_foot_path(t_start, steps['px'][step + 1], steps['py'][step + 1], 0)
    rsp = calc_support_foot_path(t_start, steps['px'][step], steps['py'][step], 0)

    extend(r_foot_path, rsp)
    extend(l_foot_path, lsp)

    # plt.plot(l_foot_path['t'], l_foot_path['x'])
    # plt.plot(l_foot_path['t'], l_foot_path['y'])

    # plt.plot(r_foot_path['t'], r_foot_path['x'])
    # plt.plot(r_foot_path['t'], r_foot_path['y'])
    # plt.show()

  def calc_walk_primitive(ti, dt, xi, vxi, yi, vyi):
    walk_t = { 't':[], 'x':[], 'vx':[], 'y':[], 'vy':[], 'z':[] }

    # walk_t['t'].append(ti)
    # walk_t['x'].append(xi)
    # walk_t['vx'].append(vxi)
    # walk_t['y'].append(yi)
    # walk_t['vy'].append(vyi)
    # walk_t['z'].append(zc)

    def calc_CoM_state():
      _t = t / Tc

      def calc_position(p, v, fp):
        return (p - fp) * math.cosh(_t) + Tc * v * math.sinh(_t) + fp
      
      def calc_velocity(p, v, fp):
        return (p - fp) / Tc * math.sinh(_t) + v * math.cosh(_t)

      walk_t['x'].append(calc_position(xi, vxi, px))
      walk_t['vx'].append(calc_velocity(xi, vxi, px))

      walk_t['y'].append(calc_position(yi, vyi, py))
      walk_t['vy'].append(calc_velocity(yi, vyi, py))

      walk_t['z'].append(zc)

      walk_t['t'].append(ti + t)

    t = 0
    while t < Tsup:
      calc_CoM_state()
      t += dt
    calc_CoM_state() # for last t

    plt.plot(walk_t['x'], walk_t['y'])
    # plt.plot(walk_t['t'], walk_t['y'])
    # plt.plot(walk_t['x'], walk_t['y'])

    return walk_t

  T = 0

  px0, py0, px, py = calc_foot_place(px0, py0, px, py) # set the initial placement of foot
  ## carry out the steps indicated by walk parameters sx, sy
  while n < len(sx): # sx was expanded by one element previously
    ## plot the n-th step trajectory
    walk_primitive = calc_walk_primitive(T, 0.01, xi, vxi, yi, vyi)

    CoM_path['t'].extend(walk_primitive['t'])
    CoM_path['x'].extend(walk_primitive['x'])
    CoM_path['vx'].extend(walk_primitive['vx'])
    CoM_path['y'].extend(walk_primitive['y'])
    CoM_path['vy'].extend(walk_primitive['vy'])
    CoM_path['z'].extend(walk_primitive['z'])

    ## update xi, yi, vxi, vyi for the next step
    xi = walk_primitive['x'][-1]
    vxi = walk_primitive['vx'][-1]
    yi = walk_primitive['y'][-1]
    vyi = walk_primitive['vy'][-1]
    
    ## update n, the order number of the step
    n += 1; 

    if n < len(sx):
      px0, py0, px, py = calc_foot_place(px0, py0, px, py); # calculate the actual foot place for step n
    T += Tsup

  calc_foots_path()

  # plt.plot(CoM_trajectory['x'], CoM_trajectory['y'])
  # plt.plot(l_foot_trajectory['t'], l_foot_trajectory['z'])
  # plt.plot(r_foot_trajectory['t'], r_foot_trajectory['z'])
  plt.show()

  return CoM_path, l_foot_path, r_foot_path

sx = [0.0, 0.3, 0.3, 0.3, 0.0]
sy = [0.2, 0.2, 0.2, 0.2, 0.2]

CoM_z = 0.28 + 0.28 + 0.102 - 0.231

CoM_path, l_foot_path, r_foot_path = LIP(CoM_z - 0.1, 0.8, sx, sy, 0, 0, 0, 0, 0, 0, 10, 1)

def save_paths():
  # Save trajectories in files
  ## Save CoM trajectory
  f = open('results/CoM_trajectory.txt', 'w')
  f.write('TIME\tCoM_x\tCoM_y\tCoM_z\n')

  i = 0
  for i in range(0, len(CoM_path['t'])) :
    f.write(str(CoM_path['t'][i]) + '\t')
    f.write(str(CoM_path['x'][i]) +  '\t')
    f.write(str(CoM_path['y'][i]) + '\t')
    f.write(str(CoM_path['z'][i]) + '\n')
  f.close()

  ## Save left foot trajectory
  f = open('results/l_foot_trajectory.txt', 'w')
  f.write('TIME\tl_foot_x\tl_foot_y\tl_foot_z\n')

  i = 0
  for i in range(0, len(l_foot_path['t'])) :
    f.write(str(l_foot_path['t'][i]) + '\t')
    f.write(str(l_foot_path['x'][i]) +  '\t')
    f.write(str(l_foot_path['y'][i]) + '\t')
    f.write(str(l_foot_path['z'][i]) + '\n')
  f.close()

  ## Save right foot trajectory
  f = open('results/r_foot_trajectory.txt', 'w')
  f.write('TIME\tr_foot_x\tr_foot_y\tr_foot_z\n')

  i = 0
  for i in range(0, len(r_foot_path['t'])) :
    f.write(str(r_foot_path['t'][i]) + '\t')
    f.write(str(r_foot_path['x'][i]) +  '\t')
    f.write(str(r_foot_path['y'][i]) + '\t')
    f.write(str(r_foot_path['z'][i]) + '\n')
  f.close()

save_paths()