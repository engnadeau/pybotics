from pybotics import Robot
from pybotics import calibration
import numpy as np
import pytest
import os


@pytest.fixture
def robot():
    model_path = os.path.abspath(__file__)
    model_path = os.path.dirname(model_path)
    model_path = os.path.join(model_path, 'ur10-mdh.csv')
    return Robot(np.loadtxt(model_path, delimiter=','))


def test_compute_absolute_errors(robot):
    joints = []
    torques = []
    positions = []
    for i in range(100):
        robot.random_joints()
        joints.append(robot.joint_angles)
        torques.append([0] * robot.num_dof())
        positions.append(robot.fk()[:-1, -1])

    errors = calibration.compute_absolute_errors(robot=robot,
                                                 joints=joints,
                                                 torques=torques,
                                                 positions=positions)
    assert len(errors) == len(joints)
    assert sum(errors) == 0
    assert min(errors) >= 0
