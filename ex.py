from joints import LegJoints
from legs_controller import calc_legs_swing

def main():
  # l_leg = LegJoints()
  # r_leg = LegJoints()

  # l_leg.hip_yaw = 1.0
  # r_leg.hip_yaw = 1.0

  # print('l hip yaw =', l_leg.hip_yaw)
  # print('r hip yaw =', r_leg.hip_yaw)

  calc_legs_swing(0.5)

if __name__ == '__main__':
  main()