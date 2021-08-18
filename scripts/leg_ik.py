import math
from math import pi
import numpy as np
import numpy.linalg

from scipy.spatial.transform import Rotation

def calc_leg_joints(body, D, A, B, foot):

  body_p = np.array(body['p'])
  body_R = np.array(body['R'])
  foot_p = np.array(foot['p'])
  foot_R = np.array(foot['R'])

  D_mult = np.array([0, D, 0])
  
  # crotch from ankle
  p2 = body_p + np.dot(body_R, D_mult.T)
  r = foot_R.T.dot(p2 - foot_p)
  C = numpy.linalg.norm(r)

  c5 = (C**2 - A**2 - B**2) / (2.0*A*B)

  # knee pitch
  if c5 >= 1:
    q5 = 0.0
  elif c5 <= -1:
    q5 = pi
  else:
    q5 = math.acos(c5)

  # ankle pitch sub
  q6a = math.asin((A / C) * math.sin(pi - q5))

  # ankle roll
  # -pi/2 < q7 < pi/2
  q7 = math.atan2(r[1], r[2]) 
  if q7 > pi/2:
    q7 -= pi 
  elif q7 < -pi/2:
    q7 += pi
  
  # ankle pitch
  q6 = -math.atan2(r[0], np.sign(r[2]) * math.sqrt(r[1]**2 + r[2]**2)) - q6a

  Rx = Rotation.from_euler('x', -q7).as_matrix()
  Ry = Rotation.from_euler('y', -q6 - q5).as_matrix()

  R = body_R.T.dot(foot_R).dot(Rx).dot(Ry) ## hipZ * hipX * hipY
  
  # hip yaw
  q2 = math.atan2(-R[0, 1], R[1, 1]); 

  cz = math.cos(q2); sz = math.sin(q2)

  # hip roll
  q3 = math.atan2(R[2, 1], -R[0, 1] * sz + R[1, 1] * cz)
  # hip pitch
  q4 = math.atan2(-R[2, 0], R[2, 2]); 

  return q2, q3, q4, q5, q6, q7

def check_leg_joints(joints):
  roll_sum = joints[1] + joints[5]
  pitch_sum = joints[2] + joints[3] + joints[4]
  if math.isclose(roll_sum, 0, abs_tol=0.001):
    if math.isclose(pitch_sum, 0, abs_tol=0.001):
      print("Joints are correct.")
    else:
      print("Pitch joints are incorrect.")
  else:
      print("Roll joints are incorrect.")

