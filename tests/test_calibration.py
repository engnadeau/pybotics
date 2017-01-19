from pybotics.robot import Robot
from pybotics import robot_utilities
from pybotics import calibration
import numpy as np
import pytest
import os

np.set_printoptions(suppress=True)


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
        joints.append(robot_utilities.random_joints(robot.joint_angle_limits))
        torques.append([0] * robot.num_dof())
        positions.append(robot.fk(joints[-1])[:-1, -1])

    errors = calibration.compute_absolute_errors(robot=robot,
                                                 joints=joints,
                                                 torques=torques,
                                                 positions=positions,
                                                 reference_frame=robot.world_frame
                                                 )
    assert len(errors) == len(joints)
    assert sum(errors) == 0
    assert min(errors) >= 0


def test_calibration_fitness_func(robot):
    joints = []
    torques = []
    positions = []
    for i in range(100):
        joints.append(robot_utilities.random_joints(robot.joint_angle_limits))
        torques.append([0] * robot.num_dof())
        positions.append(robot.fk(joints[-1])[:-1, -1])

    optimization_mask = robot.generate_optimization_mask(tool_mask=True)
    optimization_vector = np.array(robot.generate_optimization_vector(optimization_mask))

    # test abs list
    errors = calibration.calibration_fitness_func(optimization_vector,
                                                  optimization_mask,
                                                  robot,
                                                  joints,
                                                  torques,
                                                  positions,
                                                  robot.world_frame,
                                                  'abs',
                                                  'list'
                                                  )

    assert len(errors) == len(joints)
    assert sum(errors) == 0
    assert min(errors) >= 0

    # test abs sumsq
    errors = calibration.calibration_fitness_func(optimization_vector,
                                                  optimization_mask,
                                                  robot,
                                                  joints,
                                                  torques,
                                                  positions,
                                                  robot.world_frame,
                                                  'abs',
                                                  'sumsq'
                                                  )

    assert errors == 0

    # test exceptions
    with pytest.raises(ValueError):
        calibration.calibration_fitness_func(optimization_vector,
                                             optimization_mask,
                                             robot,
                                             joints,
                                             torques,
                                             positions,
                                             robot.world_frame,
                                             'absxxx',
                                             'sumsq'
                                             )
    with pytest.raises(ValueError):
        calibration.calibration_fitness_func(optimization_vector,
                                             optimization_mask,
                                             robot,
                                             joints,
                                             torques,
                                             positions,
                                             robot.world_frame,
                                             'abs',
                                             'sumsqxxx'
                                             )
