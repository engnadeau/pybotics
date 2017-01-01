from unittest import TestCase
from pybotics.robot import Robot
from pybotics import robot_utilities
from pybotics import calibration
import numpy as np


class TestGeometry(TestCase):
    def setUp(self):
        np.set_printoptions(suppress=True)

        # load UR10 model for testing
        model = np.array([
            0, 0, 0, 118,
            1.5707963267949, 0, 3.14159265358979, 0,
            0, 612.7, 0, 0,
            0, 571.6, 0, 163.9,
            - 1.5707963267949, 0, 0, 115.7,
            1.5707963267949, 0, 3.14159265358979, 92.2
        ]).reshape((-1, 4))
        self.robot = Robot(model)

    def test_compute_absolute_errors(self):
        joints = []
        torques = []
        positions = []
        for i in range(100):
            joints.append(robot_utilities.random_joints(self.robot.joint_angle_limits))
            torques.append([0] * self.robot.num_dof())
            positions.append(self.robot.fk(joints[-1])[:-1, -1])

        errors = calibration.compute_absolute_errors(robot=self.robot,
                                                     joints=joints,
                                                     torques=torques,
                                                     positions=positions,
                                                     reference_frame=self.robot.world_frame
                                                     )

        self.assertEqual(len(errors), len(joints))
        self.assertEqual(sum(errors), 0)
        self.assertTrue(min(errors) >= 0)

    def test_calibration_fitness_func(self):
        joints = []
        torques = []
        positions = []
        for i in range(100):
            joints.append(robot_utilities.random_joints(self.robot.joint_angle_limits))
            torques.append([0] * self.robot.num_dof())
            positions.append(self.robot.fk(joints[-1])[:-1, -1])

        optimization_mask = self.robot.generate_optimization_mask(tool_mask=True)
        optimization_vector = np.array(self.robot.generate_optimization_vector(optimization_mask))

        # test abs list
        errors = calibration.calibration_fitness_func(optimization_vector,
                                                      optimization_mask,
                                                      self.robot,
                                                      joints,
                                                      torques,
                                                      positions,
                                                      self.robot.world_frame,
                                                      'abs',
                                                      'list'
                                                      )

        self.assertEqual(len(errors), len(joints))
        self.assertEqual(sum(errors), 0)
        self.assertTrue(min(errors) >= 0)

        # test abs sumsq
        errors = calibration.calibration_fitness_func(optimization_vector,
                                                      optimization_mask,
                                                      self.robot,
                                                      joints,
                                                      torques,
                                                      positions,
                                                      self.robot.world_frame,
                                                      'abs',
                                                      'sumsq'
                                                      )

        self.assertEqual(errors, 0)

        # test exceptions
        with self.assertRaises(ValueError):
            calibration.calibration_fitness_func(optimization_vector,
                                                 optimization_mask,
                                                 self.robot,
                                                 joints,
                                                 torques,
                                                 positions,
                                                 self.robot.world_frame,
                                                 'absxxx',
                                                 'sumsq'
                                                 )
        with self.assertRaises(ValueError):
            calibration.calibration_fitness_func(optimization_vector,
                                                 optimization_mask,
                                                 self.robot,
                                                 joints,
                                                 torques,
                                                 positions,
                                                 self.robot.world_frame,
                                                 'abs',
                                                 'sumsqxxx'
                                                 )
