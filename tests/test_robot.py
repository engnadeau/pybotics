"""Test robot."""
import json
from typing import Tuple

import hypothesis
import numpy as np
from hypothesis import given
from hypothesis.extra.numpy import arrays
from hypothesis.strategies import floats

from pybotics.constants import TRANSFORM_MATRIX_SHAPE
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot


def test_fk(serial_robot):
    """
    Test robot.

    :param serial_robot:
    :return:
    """
    joints = np.deg2rad([10, 20, 30, 40, 50, 60])
    desired_pose = np.array(
        [-0.786357, -0.607604, 0.111619, -776.143784,
         -0.527587, 0.566511, -0.633022, -363.462788,
         0.321394, -0.556670, -0.766044, -600.056043,
         0, 0, 0, 1]
    ).reshape(TRANSFORM_MATRIX_SHAPE)

    # test with position argument
    actual_pose = serial_robot.fk(q=joints)
    np.testing.assert_allclose(actual_pose, desired_pose, atol=1e-6)

    # test with internal position attribute
    serial_robot.joints = joints
    actual_pose = serial_robot.fk()
    np.testing.assert_allclose(actual_pose, desired_pose, atol=1e-6)


def test_init():
    """
    Test robot.

    :return:
    """
    Robot(KinematicChain.from_array(np.ones(4)))


def test_repr(serial_robot):
    s = repr(serial_robot)

    # check for important attributes
    assert '_position' in s
    assert '_position_limits' in s
    assert 'world_frame' in s
    assert 'kinematic_chain' in s
    assert 'tool' in s

    # check if valid json
    json.loads(s)

    # check str()
    assert str(serial_robot) == s


def test_calculate_joint_torques(planar_robot: Robot,
                                 planar_robot_link_lengths: Tuple):
    """
    From EXAMPLE 5.7 of
    Craig, John J. Introduction to robotics: mechanics and control.
    Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
    :return:
    """

    # set test force and angles
    force = [-100, -200, 0]
    moment = [0] * 3
    wrench = force + moment
    joint_angles = np.deg2rad([30, 60, 0])

    # calculate expected torques
    expected_torques = [
        planar_robot_link_lengths[0] * np.sin(joint_angles[1]) * force[0] +
        (planar_robot_link_lengths[1] + planar_robot_link_lengths[0] *
         np.cos(joint_angles[1])) * force[1],
        planar_robot_link_lengths[1] * force[1],
        0
    ]

    # test
    actual_torques = planar_robot.calculate_joint_torques(joint_angles, wrench)
    np.testing.assert_allclose(actual_torques, expected_torques)


@given(q=arrays(shape=(3,), dtype=float, elements=floats(max_value=1e9,
                                                         min_value=-1e9,
                                                         allow_nan=False,
                                                         allow_infinity=False)))
def test_jacobian_world(q: np.ndarray, planar_robot: Robot,
                        planar_robot_link_lengths: Tuple):
    # example from Craig has last joint set to 0
    q[-1] = 0

    s0 = np.sin(q[0])
    c0 = np.cos(q[0])

    s01 = np.sin(q[0] + q[1])
    c01 = np.cos(q[0] + q[1])

    expected = np.zeros((6, 3))
    expected[0, 0] = -planar_robot_link_lengths[0] * s0 \
                     - planar_robot_link_lengths[1] * s01
    expected[0, 1] = - planar_robot_link_lengths[1] * s01
    expected[1, 0] = planar_robot_link_lengths[0] * c0 \
                     + planar_robot_link_lengths[1] * c01
    expected[1, 1] = planar_robot_link_lengths[1] * c01
    expected[-1, :] = 1

    actual = planar_robot.jacobian_world(q)
    np.testing.assert_allclose(actual, expected, atol=1e-3)


@given(q=arrays(shape=(3,), dtype=float,
                elements=floats(allow_nan=False, allow_infinity=False)))
def test_jacobian_flange(q: np.ndarray, planar_robot: Robot,
                         planar_robot_link_lengths: Tuple):
    # example from Craig has last joint set to 0
    q[-1] = 0

    s1 = np.sin(q[1])
    c1 = np.cos(q[1])

    expected = np.zeros((6, 3))
    expected[0, 0] = planar_robot_link_lengths[0] * s1
    expected[1, 0] = planar_robot_link_lengths[0] * c1 + \
                     planar_robot_link_lengths[1]
    expected[1, 1] = planar_robot_link_lengths[1]
    expected[-1, :] = 1

    actual = planar_robot.jacobian_flange(q)
    np.testing.assert_allclose(actual, expected, atol=1e-6)


@given(q=arrays(shape=(3,), dtype=float,
                elements=floats(allow_nan=False, allow_infinity=False)))
def test_ik_jacobian(q: np.ndarray, planar_robot: Robot):
    pose = planar_robot.fk(q)

    q_actual = planar_robot.ik_jacobian(pose, max_iter=1e9)
    actual_pose = planar_robot.fk(q_actual)

    # test the matrix with lower accuracy
    # rotation components are hard to achieve when x0 isn't good
    np.testing.assert_allclose(actual_pose, pose, atol=1e-1)

    # test the position with high accuracy
    desired_position = pose[:-1, -1]
    actual_position = actual_pose[:-1, -1]
    np.testing.assert_allclose(actual_position, desired_position)


@given(q=arrays(shape=(6,), dtype=float,
                elements=floats(allow_nan=False, allow_infinity=False)))
@hypothesis.settings(use_coverage=False)
def test_ik(q: np.ndarray, serial_robot: Robot):
    pose = serial_robot.fk(q)

    q_actual = serial_robot.ik(pose)
    actual_pose = serial_robot.fk(q_actual)

    # test the matrix with lower accuracy
    # rotation components are hard to achieve when x0 isn't good
    np.testing.assert_allclose(actual_pose, pose, atol=1)

    # test the position with high accuracy
    desired_position = pose[:-1, -1]
    actual_position = actual_pose[:-1, -1]
    np.testing.assert_allclose(actual_position, desired_position, atol=1e-2)
