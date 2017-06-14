import os
import numpy as np
import timeit
import logging

from pybotics import Robot


def load_robot():
    model_path = os.path.abspath(__file__)
    model_path = os.path.dirname(model_path)
    model_path = os.path.dirname(model_path)
    model_path = os.path.join(model_path, 'robot-models')
    model_path = os.path.join(model_path, 'ur10-mdh.csv')
    return Robot(np.loadtxt(model_path, delimiter=','))


robot = load_robot()


def profile():
    robot.random_joints()
    robot.fk()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    logging.info('Profiling pybotics forward kinematics')
    profile()   # use method to avoid vulture linting error

    num_calls = int(1e6)
    logging.info('Number of calls:\t{}'.format(num_calls))

    time = timeit.timeit(stmt='profile()',
                         setup="from __main__ import profile",
                         number=num_calls)
    mean_time = time / num_calls

    logging.info('Total time:\t{} s'.format(time))
    logging.info('Mean time:\t{} s'.format(mean_time))
    logging.info('Mean time:\t{} ms'.format(mean_time * 1e3))
