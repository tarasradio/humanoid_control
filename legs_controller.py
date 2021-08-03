from leg_ik import calc_leg_joints
import numpy as np
import math
import matplotlib.pyplot as plt

def calc_legs_swing(body_z):

  E = np.eye(3)

  hip_length = 0.28
  shin_length = 0.28
  pelvis_to_hip = 0.176 / 2

  body = { 'p':[0, 0, body_z], 'R':E }
  l_foot = { 'p':[0, 0.2, 0], 'R':E }
  r_foot = { 'p':[0, -0.2, 0], 'R':E }

  t = 0
  T = 10; P = 2

  body_path = {'t':[], 'z':[]}
  l_leg_joint_positions = { 't':[], 'joints':[()]}
  r_leg_joint_positions = { 't':[], 'joints':[()]}

  while t < T:
    phase = (t % P) / P * math.pi * 2
    z = body_z - math.sin(phase) * 0.1

    body_path['t'].append(t)
    body_path['z'].append(z)
    l_leg_joint_positions['t'].append(t)
    r_leg_joint_positions['t'].append(t)
    l_leg_joint_positions['joints'].append(calc_leg_joints(body, pelvis_to_hip, hip_length, shin_length, l_foot))
    r_leg_joint_positions['joints'].append(calc_leg_joints(body, -pelvis_to_hip, hip_length, shin_length, r_foot))
    t += 0.01

  # plt.plot(body_path['t'], body_path['z'])
  # plt.show()
  return l_leg_joint_positions, r_leg_joint_positions