import copy

import pybotics as py
import numpy as np

# set numpy print options
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# create robot
model = np.loadtxt('ur10-mdh.csv', delimiter=',')
robot = py.Robot(model)

print('Robot Model:\n{}\n'.format(robot.robot_model))

# demonstrate forward kinematics
joints = [0] * robot.num_dof()
pose = robot.fk(joints)

print('Pose:\n{}\n'.format(pose))

# demonstrate inverse kinematics
new_joints = robot.ik(pose)
print('Solved Joints:\n{}\n'.format(new_joints))
