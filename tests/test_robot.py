import os

import numpy as np
from pytest import fixture

from pybotics.robot import Robot

np.set_printoptions(suppress=True)

ROBOT_MDH_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    os.pardir,
    os.pardir,
    'robot-models',
    'ur10-mdh.csv'
))


@fixture(name='robot')
def robot_fixture():
    robot = Robot.from_revolute_mdh(np.loadtxt(ROBOT_MDH_PATH, delimiter=','))
    return robot


def test_position(robot):
    assert robot.position is robot._position

    new_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    robot.position = new_value

    assert robot._position is new_value
    assert robot.position is robot._position


def test_position_limits(robot):
    assert robot.position_limits is robot._position_limits

    new_value = np.repeat((-1, 1), robot.num_dof()).reshape((2, -1))
    robot.position_limits = new_value

    assert robot._position_limits is new_value
    assert robot.position_limits is new_value


def test_num_dof(robot):
    mdh_array = np.loadtxt(ROBOT_MDH_PATH, delimiter=',')
    assert robot.num_dof() == mdh_array.shape[0]


def test_fk(robot):
    # validate using current/default robot position (zeros)
    actual_pose = robot.fk()
    desired_pose = np.array([
        1, 0, 0, -1184.3,
        -0, 0, -1, -256.1,
        -0, 1, 0, 2.3,
        0, 0, 0, 1
    ]).reshape((4, 4))
    np.testing.assert_allclose(desired=desired_pose, actual=actual_pose, atol=1e-7)

    # validate using new robot position
    robot.position = np.deg2rad([10, 20, 30, 40, 50, 60])
    actual_pose = robot.fk()
    desired_pose = np.array([
        -0.786357, -0.607604, 0.111619, -776.143784,
        -0.527587, 0.566511, -0.633022, -363.462788,
        0.321394, -0.556670, -0.766044, -600.056043,
        0, 0, 0, 1
    ]).reshape((4, 4))
    np.testing.assert_allclose(desired=desired_pose, actual=actual_pose, atol=1e-6)

    # validate using position argument
    actual_pose = robot.fk(position=-np.deg2rad([10, 20, 30, 40, 50, 60]))
    desired_pose = np.array([
        -0.786357, 0.607604, -0.111619, -1081.532695,
        0.527587, 0.566511, -0.633022, -35.904303,
        -0.321394, -0.556670, -0.766044, 694.797448,
        0.000000, 0.000000, 0.000000, 1.000000
    ]).reshape((4, 4))
    np.testing.assert_allclose(desired=desired_pose, actual=actual_pose, atol=1e-6)
