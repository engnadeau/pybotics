import numpy as np
import pytest
import os

from pybotics import Robot
from pybotics import calibration
from pybotics.exceptions import PybotException


@pytest.fixture(name='robot')
def robot_fixture():
    model_path = os.path.abspath(__file__)
    model_path = os.path.dirname(model_path)
    model_path = os.path.dirname(model_path)
    model_path = os.path.join(model_path, 'robot-models')
    model_path = os.path.join(model_path, 'ur10-mdh.csv')
    return Robot(np.loadtxt(model_path, delimiter=','))


def test_compute_absolute_errors(robot):
    joints = []
    torques = []
    positions = []
    for _ in range(100):
        robot.generate_random_joints()
        joints.append(robot.joint_angles)
        torques.append([0] * robot.num_dof())
        positions.append(robot.fk()[:-1, -1])

    errors = calibration.compute_absolute_errors(robot=robot,
                                                 joint_angles=joints,
                                                 joint_torques=torques,
                                                 tcp_positions=positions)
    assert len(errors) == len(joints)
    np.testing.assert_allclose(errors, [0] * len(errors))

    with pytest.raises(PybotException):
        calibration.compute_absolute_errors(robot=robot,
                                            joint_angles=joints + joints,
                                            joint_torques=torques,
                                            tcp_positions=positions)
    with pytest.raises(PybotException):
        calibration.compute_absolute_errors(robot=robot,
                                            joint_angles=joints,
                                            joint_torques=torques + torques,
                                            tcp_positions=positions)
    with pytest.raises(PybotException):
        calibration.compute_absolute_errors(robot=robot,
                                            joint_angles=joints,
                                            joint_torques=torques,
                                            tcp_positions=positions + positions)
