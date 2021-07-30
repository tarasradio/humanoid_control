from leg_ik import calc_leg_joints
import numpy as np
import math
import matplotlib.pyplot as plt

E = np.eye(3)

hip_length = 0.5
shin_length = 0.5
pelvis_to_hip = 0.2

body = { 'p':[0, 0, 0.8], 'R':E }
l_foot = { 'p':[0, 0.2, 0], 'R':E }
r_foot = { 'p':[0, -0.2, 0], 'R':E }

t = 0
T = 10; P = 2

body_path = {'t':[], 'z':[]}

while t < T:
  phase = (t % P) / P * math.pi * 2
  z = 0.8 - math.sin(phase) * 0.1

  body_path['t'].append(t)
  body_path['z'].append(z)

  t += 0.01

plt.plot(body_path['t'], body_path['z'])
plt.show()

l_leg_joint_positions = calc_leg_joints(body, pelvis_to_hip, hip_length, shin_length, l_foot)
r_leg_joint_positions = calc_leg_joints(body, -pelvis_to_hip, hip_length, shin_length, r_foot)

print('l hip yaw:', math.degrees(l_leg_joint_positions[0]))
print('l hip roll:', math.degrees(l_leg_joint_positions[1]))
print('l hip pitch:', math.degrees(l_leg_joint_positions[2]))
print('l knee pitch:', math.degrees(l_leg_joint_positions[3]))
print('l ankle pitch:', math.degrees(l_leg_joint_positions[4]))
print('l ankle roll:', math.degrees(l_leg_joint_positions[5]))

print('r hip yaw:', math.degrees(r_leg_joint_positions[0]))
print('r hip roll:', math.degrees(r_leg_joint_positions[1]))
print('r hip pitch:', math.degrees(r_leg_joint_positions[2]))
print('r knee pitch:', math.degrees(r_leg_joint_positions[3]))
print('r ankle pitch:', math.degrees(r_leg_joint_positions[4]))
print('r ankle roll:', math.degrees(r_leg_joint_positions[5]))

