from numpy.lib.function_base import angle
import matplotlib.pyplot as plt
from matplotlib import patches

import math
import numpy as np

footLength = 0.25; footWidth = 0.1

def drawFootprint(ax, x, y, theta, leg):
  if leg == 0:
    edgecolor = 'b'
  else:
    edgecolor = 'r'
  
  rotation = math.atan2(footWidth / 2, footLength / 2)
  r = math.hypot((footLength / 2), (footWidth / 2))

  dx = r * math.sin(math.pi / 2 - theta - rotation)
  dy = r * math.cos(math.pi / 2 - theta - rotation)

  pos = (x - dx, y - dy)
  
  footprint = patches.Rectangle(pos, footLength, footWidth, angle=math.degrees(theta), linewidth=1, edgecolor=edgecolor, facecolor='none', lineStyle='--')
  ax.add_patch(footprint)
  ax.plot(x, y, '+', color=edgecolor)

def generate_footsteps(s_l, s_w):
  fig, ax = plt.subplots()

  fig.canvas.set_window_title('Footsteps generation result.')

  plt.title('Footsteps and path line')
  plt.xlabel('x (m)')
  plt.ylabel('y (m)')

  fs_array = { 'x':[], 'y':[] }

  x = 0; y = 0

  fs_array['x'].append(x)
  fs_array['y'].append(y)

  def drawFootsteps(anglePerStep, countSteps, fLeg, initialAngle = 0):
    r = math.hypot(s_l, s_w)
    rotation = math.atan2(s_l, s_w)

    x = fs_array['x'][-1]; y = fs_array['y'][-1]
    
    for i in range(countSteps):
      angle = i * anglePerStep + initialAngle
      if i % 2 == fLeg:
        x += -r * math.cos(-angle + rotation)
        y += r * math.sin(-angle + rotation)
        drawFootprint(ax, x, y, angle + math.pi / 2, 0)
      else:
        x += r * math.cos(angle + rotation)
        y += r * math.sin(angle + rotation)
        drawFootprint(ax, x, y, angle + math.pi / 2, 1)
      fs_array['x'].append(x)
      fs_array['y'].append(y)

    if countSteps % 2 == 1:
      fLeg = 1 - fLeg

    finalAngle = countSteps * anglePerStep + initialAngle
    return (fLeg, finalAngle)

  firstLeg, angle = drawFootsteps(math.radians(10), int(90/10), 0)
  firstLeg, angle = drawFootsteps(math.radians(0), 5, firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(10), int(90/10), firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(0), 5, firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(10), int(90/10), firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(0), 5, firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(10), int(90/10), firstLeg, angle)
  firstLeg, angle = drawFootsteps(math.radians(0), 5, firstLeg, angle)

  ax.plot(fs_array['x'], fs_array['y'], '-g', linewidth=0.8)
  plt.axis("equal")
  plt.show()

generate_footsteps(0.3, 0.18)