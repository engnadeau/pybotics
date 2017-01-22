import os
import numpy as np
import logging
import timeit

from pybotics import Robot


def load_robot():
    model_path = os.path.abspath(__file__)
    model_path = os.path.dirname(model_path)
    model_path = os.path.dirname(model_path)
    model_path = os.path.join(model_path, 'robot-models')
    model_path = os.path.join(model_path, 'ur10-mdh.csv')
    return Robot(np.loadtxt(model_path, delimiter=','))


def profile(robot: Robot):
    robot.random_joints()
    robot.fk()


if __name__ == '__main__':
    num_calls = int(1e6)
    time = timeit.timeit(stmt='profile(load_robot())',
                         setup="from __main__ import profile, load_robot",
                         number=num_calls)
    print('Total time: {} s'.format(time))
    print('Mean time: {} s'.format(time / num_calls))
