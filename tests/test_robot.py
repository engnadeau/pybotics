from pybotics.robot import Robot
from pybotics.robot import Tool
from pybotics.constants import Constant
from pybotics import geometry
from pybotics.exceptions import PybotException
import numpy as np
import itertools
import pytest
import os


@pytest.fixture
def robot():
    model_path = os.path.abspath(__file__)
    model_path = os.path.dirname(model_path)
    model_path = os.path.join(model_path, 'ur10-mdh.csv')
    return Robot(np.loadtxt(model_path, delimiter=','))


def test_num_dof(robot):
    assert robot.num_dof() == robot.robot_model.shape[0]


def test_fk(robot):
    expected_transform = np.array([
        [0, 0, -1, -663.8],
        [-1, 0, -0, -163.9],
        [-0, 1, 0, 615],
        [0, 0, 0, 1]
    ])

    robot.joint_angles = np.deg2rad([0, -90, 90, 0, 90, 0])
    actual_transform = robot.fk()

    assert actual_transform.shape[0] == expected_transform.shape[0]
    assert actual_transform.shape[1] == expected_transform.shape[1]
    assert actual_transform.size == expected_transform.size
    np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)


def test_ik(robot):
    test_joints_list = np.deg2rad([
        [0, -90, 90, 0, 90, 0],
        [10, 90, 80, 20, 90, 123],
        [20, -40, 90, 10, 20, 0],
        [-10, 20, -30, 40, -50, 60]
    ])

    for test_joints in test_joints_list:
        robot.joint_angles = test_joints
        expected_transform = robot.fk()

        ik_joints = robot.ik(expected_transform)
        robot.joint_angles = ik_joints
        result_transform = robot.fk()

        assert len(ik_joints) == len(test_joints)
        np.testing.assert_allclose(actual=result_transform, desired=expected_transform, rtol=1e-1, atol=1e-1)


def test_calculate_joint_torques(robot):
    """
    From EXAMPLE 5.7 of
    Craig, John J. Introduction to robotics: mechanics and control.
    Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.

    :return:
    """

    # set robot
    link_length = [10, 20]
    robot_model = np.array([
        [0, 0, 0, 0],
        [0, link_length[0], 0, 0],
        [0, link_length[1], 0, 0]
    ], dtype=np.float)
    robot = Robot(robot_model)

    # set test force and angles
    force = [-100, -200, 0]
    moment = [0] * 3
    wrench = force + moment
    joint_angles = np.deg2rad([30, 60, 0])

    # calculate expected torques
    expected_torques = [
        link_length[0] * np.sin(joint_angles[1]) * force[0] +
        (link_length[1] + link_length[0] * np.cos(joint_angles[1])) * force[1],
        link_length[1] * force[1],
        0
    ]

    # test
    robot.joint_angles = joint_angles
    torques = robot.calculate_external_wrench_joint_torques(wrench)
    np.testing.assert_allclose(torques, expected_torques)


def test_validate_joint_angles(robot):
    test_joints_list = np.deg2rad([
        [10, 90, 80, 20, 90, 123],
        [-10, 20, -30, 40, -50, 191]
    ])

    assert robot.validate_joint_angles(test_joints_list[0])
    assert not robot.validate_joint_angles(test_joints_list[1])


def test_set_tool_xyz(robot):
    test_tool = [1.1, 2.2, 3.3]
    robot.set_tool_xyz(test_tool)
    np.testing.assert_allclose(test_tool, robot.tool.tcp[:-1, -1])


def test_generate_optimization_vector(robot):
    mask = robot.generate_optimization_mask()
    vector = robot.generate_optimization_vector(mask)
    assert len(vector) == 0

    mask = robot.generate_optimization_mask(world_mask=True)
    vector = robot.generate_optimization_vector(mask)
    assert len(vector) == 6


def test_apply_optimization_vector(robot):
    mask = robot.generate_optimization_mask(world_mask=True,
                                            robot_model_mask=True,
                                            tool_mask=True,
                                            joint_compliance_mask=True)
    vector = robot.generate_optimization_vector(mask)
    vector = [x + 1 for x in vector]
    robot.apply_optimization_vector(vector, mask)

    parameters = list(itertools.chain(
        geometry.pose_2_xyzrpw(robot.world_frame),
        robot.robot_model.ravel(),
        geometry.pose_2_xyzrpw(robot.tool.tcp),
        robot.joint_compliance
    ))

    np.testing.assert_allclose(parameters, vector)


def test_generate_optimization_mask(robot):
    mask = robot.generate_optimization_mask()
    assert sum(mask) == 0

    mask = robot.generate_optimization_mask(world_mask=True)
    assert sum(mask) == 6

    assert len(robot.generate_optimization_mask()) == len(robot.generate_optimization_mask(world_mask=True,
                                                                                           robot_model_mask=True,
                                                                                           tool_mask=True,
                                                                                           joint_compliance_mask=True))

    with pytest.raises(PybotException):
        robot.generate_optimization_mask(world_mask=[True])
    with pytest.raises(PybotException):
        robot.generate_optimization_mask(robot_model_mask=[True])
    with pytest.raises(PybotException):
        robot.generate_optimization_mask(tool_mask=[True])
    with pytest.raises(PybotException):
        robot.generate_optimization_mask(joint_compliance_mask=[True])


def test_generate_parameter_bounds(robot):
    mask = robot.generate_optimization_mask()
    bounds = robot.generate_parameter_bounds(mask)
    assert len(bounds) == 0

    mask = robot.generate_optimization_mask(world_mask=True,
                                            robot_model_mask=True,
                                            tool_mask=True,
                                            joint_compliance_mask=True)
    bounds = robot.generate_parameter_bounds(mask)
    assert len(bounds) > 0
    for bound in bounds:
        for b in bound:
            assert b is None


def test_random_joints(robot):
    robot.random_joints()
    for limit, joint in zip(robot.joint_angle_limits, robot.joint_angles):
        assert min(limit) < joint
        assert max(limit) > joint


def test_calculate_tool_wrench(robot):
    # test mass directly on flange
    robot.tool = Tool(mass=10)
    wrench = robot.calculate_tool_wrench()
    wrench[1] -= robot.tool.mass * Constant.GRAVITY.value  # only y-force should see load
    np.testing.assert_allclose(wrench, [0] * 6, atol=1e-7)

    # test mass further from flange
    robot.tool.cg = [0, 0, 100]
    wrench = robot.calculate_tool_wrench()
    wrench[1] -= robot.tool.mass * Constant.GRAVITY.value  # only y-force should see load
    wrench[3] += robot.tool.mass * Constant.GRAVITY.value * robot.tool.cg[2]  # only x-moment should see load
    np.testing.assert_allclose(wrench, [0] * 6, atol=1e-7)

    # test new directions
    robot.joint_angles = np.deg2rad([0, 0, 0, 0, 0, 90])
    wrench = robot.calculate_tool_wrench()
    wrench[0] -= robot.tool.mass * Constant.GRAVITY.value  # only x-force should see load
    wrench[4] -= robot.tool.mass * Constant.GRAVITY.value * robot.tool.cg[2]  # only y-moment should see load
    np.testing.assert_allclose(wrench, [0] * 6, atol=1e-7)


def test_joint_torques(robot):
    torques = [1, 2, 3, -4, -5, -6]
    robot.joint_torques = torques
    np.testing.assert_allclose(robot.joint_torques, torques)

    with pytest.raises(PybotException):
        robot.joint_torques = torques + torques
