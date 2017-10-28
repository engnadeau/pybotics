"""Test robot."""
from itertools import chain
from typing import Sequence

import numpy as np
from pytest import raises

from pybotics.constants import TRANSFORM_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE
from pybotics.errors import SequenceLengthError, PyboticsError
from pybotics.geometry import euler_zyx_2_matrix
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot
from pybotics.robot_optimization_mask import RobotOptimizationMask


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
    actual_pose = serial_robot.fk(position=joints)
    np.testing.assert_allclose(actual_pose, desired_pose, atol=1e-6)

    # test with internal position attribute
    serial_robot.position = joints
    actual_pose = serial_robot.fk()
    np.testing.assert_allclose(actual_pose, desired_pose, atol=1e-6)

    # test validation
    with raises(SequenceLengthError):
        serial_robot.fk(position=np.ones(len(serial_robot) * 2))


def test_optimization(serial_robot):
    """
    Test robot.

    :param serial_robot:
    :return:
    """
    masked_index = 1
    serial_robot.world_frame.matrix = euler_zyx_2_matrix([1, 2, 3,
                                                          np.deg2rad(10),
                                                          np.deg2rad(20),
                                                          np.deg2rad(30)])
    serial_robot.tool.matrix = euler_zyx_2_matrix([1, 2, 3,
                                                   np.deg2rad(10),
                                                   np.deg2rad(20),
                                                   np.deg2rad(30)])

    # test mask
    mask = RobotOptimizationMask(world_frame=True,
                                 kinematic_chain=True,
                                 tool=True)
    serial_robot.optimization_mask = mask

    assert isinstance(serial_robot.optimization_mask.world_frame, Sequence)
    assert all(serial_robot.optimization_mask.world_frame)
    assert len(serial_robot.
               optimization_mask.
               world_frame) == TRANSFORM_VECTOR_LENGTH

    assert isinstance(serial_robot.optimization_mask.kinematic_chain, Sequence)
    assert all(serial_robot.optimization_mask.kinematic_chain)
    assert len(serial_robot.
               optimization_mask.
               kinematic_chain) == serial_robot.kinematic_chain.num_parameters

    assert isinstance(serial_robot.optimization_mask.tool, Sequence)
    assert all(serial_robot.optimization_mask.tool)
    assert len(serial_robot.
               optimization_mask.
               tool) == TRANSFORM_VECTOR_LENGTH

    # test optimization vector
    # apply mask with single False
    # check to make sure everything is properly set
    frame_mask = [True] * TRANSFORM_VECTOR_LENGTH
    frame_mask[masked_index] = False

    kc_mask = [True] * serial_robot.kinematic_chain.num_parameters
    kc_mask[masked_index] = False

    mask = RobotOptimizationMask(world_frame=frame_mask,
                                 kinematic_chain=kc_mask,
                                 tool=frame_mask)
    serial_robot.optimization_mask = mask

    # these masked elements should not change after the optimization applied
    masked_world_element = serial_robot.world_frame.vector()[masked_index]
    masked_kc_element = serial_robot.kinematic_chain.vector[masked_index]
    masked_tool_element = serial_robot.tool.vector()[masked_index]

    new_optimization_vector = serial_robot.optimization_vector * 2
    serial_robot.apply_optimization_vector(new_optimization_vector)

    np.testing. \
        assert_almost_equal(serial_robot.world_frame.vector()[masked_index],
                            masked_world_element)
    np.testing. \
        assert_almost_equal(serial_robot.
                            kinematic_chain.vector[masked_index],
                            masked_kc_element)
    np.testing. \
        assert_almost_equal(serial_robot.tool.vector()[masked_index],
                            masked_tool_element)

    leftover_world_vector = [e for i, e in
                             enumerate(serial_robot.world_frame.vector()) if
                             i != masked_index]
    leftover_kc_vector = [e for i, e in
                          enumerate(serial_robot.kinematic_chain.vector) if
                          i != masked_index]
    leftover_tool_vector = [e for i, e in
                            enumerate(serial_robot.tool.vector()) if
                            i != masked_index]

    leftover_vector = list(chain(leftover_world_vector,
                                 leftover_kc_vector,
                                 leftover_tool_vector))
    np.testing.assert_allclose(leftover_vector,
                               new_optimization_vector,
                               atol=1e-6)


def test_position(serial_robot):
    """
    Test robot.

    :param serial_robot:
    :return:
    """
    with raises(SequenceLengthError):
        serial_robot.position = np.ones(len(serial_robot) * 2)


def test_position_limits(serial_robot):
    """
    Test robot.

    :param serial_robot:
    :return:
    """
    # test normal usage
    limits = serial_robot.position_limits
    serial_robot.position_limits = limits * 2

    with raises(PyboticsError):
        serial_robot.position_limits = np.ones((2, len(serial_robot) * 2))
    with raises(PyboticsError):
        serial_robot.position_limits = np.ones((1, len(serial_robot)))


def test_init():
    """
    Test robot.

    :return:
    """
    Robot(KinematicChain.from_array(np.ones(4)))
