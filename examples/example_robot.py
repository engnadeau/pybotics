import copy

import pybotics as py
import numpy as np

# set numpy print options
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

# create robot
ideal_robot = py.Robot()
ideal_robot.robot_model = py.robot_model.ur10()

# create pseudo-realistic robot with kinematic errors
real_robot = copy.deepcopy(ideal_robot)
real_robot.impair_robot_model()

print('Ideal Robot Model:\n', ideal_robot.robot_model, '\n')
print('Pseudo-Real Robot Model:\n', real_robot.robot_model, '\n')

# demonstrate forward kinematics
joints = [0, 0, 0, 0, 0, 0]

print('Ideal Pose:\n', ideal_robot.fk(joints), '\n')
print('Pseudo-Real Pose:\n', real_robot.fk(joints), '\n')
