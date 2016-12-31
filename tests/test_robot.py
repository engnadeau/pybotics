from unittest import TestCase
from pybotics.robot import Robot
from pybotics import geometry
import numpy as np
import itertools


class TestRobot(TestCase):
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

    def test_num_dof(self):
        # ur10 has 6 DOF
        assert self.robot.num_dof() == 6

    def test_fk(self):
        # define test values
        joints = np.deg2rad([0, -90, 90, 0, 90, 0])

        expected_transform = np.array([
            [0, 0, -1, -663.8],
            [-1, 0, -0, -163.9],
            [-0, 1, 0, 615],
            [0, 0, 0, 1]
        ])

        # test fk
        reference_frames = [None, np.eye(4)]
        torques = [None, [0] * self.robot.num_dof()]
        for reference_frame in reference_frames:
            for torque in torques:
                actual_transform = self.robot.fk(joints, reference_frame=reference_frame, torques=torque)

                assert actual_transform.shape[0] == expected_transform.shape[0]
                assert actual_transform.shape[1] == expected_transform.shape[1]
                assert actual_transform.size == expected_transform.size
                np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)

        # test exception raising
        with self.assertRaises(AssertionError):
            self.robot.fk(joints, torques=[0] * (self.robot.num_dof() + 1))

    def test_ik(self):
        test_joints_list = np.deg2rad([
            [0, -90, 90, 0, 90, 0],
            [10, 90, 80, 20, 90, 123],
            [20, -40, 90, 10, 20, 0],
            [-10, 20, -30, 40, -50, 60]
        ])

        for test_joints in test_joints_list:
            # define test values
            expected_transform = self.robot.fk(test_joints)

            # test single transform
            ik_joints = self.robot.ik(expected_transform)
            result_transform = self.robot.fk(ik_joints)

            assert len(ik_joints) == len(test_joints)
            np.testing.assert_allclose(actual=result_transform, desired=expected_transform, rtol=1e-1, atol=1e-1)

    def test_validate_joint_angles(self):
        test_joints_list = np.deg2rad([
            [10, 90, 80, 20, 90, 123],
            [-10, 20, -30, 40, -50, 191]
        ])

        self.assertTrue(self.robot.validate_joint_angles(test_joints_list[0]))
        self.assertFalse(self.robot.validate_joint_angles(test_joints_list[1]))

    def test_set_tool_xyz(self):
        test_tool = [1.1, 2.2, 3.3]
        self.robot.set_tool_xyz(test_tool)

        for i in range(3):
            self.assertAlmostEqual(test_tool[i], self.robot.tool[i, -1])

    def test_generate_optimization_vector(self):
        mask = self.robot.generate_optimization_mask()
        vector = self.robot.generate_optimization_vector(mask)
        self.assertEqual(len(vector), 0)

        mask = self.robot.generate_optimization_mask(world_mask=True)
        vector = self.robot.generate_optimization_vector(mask)
        self.assertEqual(len(vector), 6)

    def test_apply_optimization_vector(self):
        # test full mask
        mask = self.robot.generate_optimization_mask(world_mask=True,
                                                     robot_model_mask=True,
                                                     tool_mask=True,
                                                     joint_stiffness_mask=True)
        vector = self.robot.generate_optimization_vector(mask)
        vector = [x + 1 for x in vector]
        self.robot.apply_optimization_vector(vector, mask)

        parameters = list(itertools.chain(
            geometry.pose_2_xyzrpw(self.robot.world_frame),
            self.robot.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.robot.tool),
            self.robot.joint_stiffness
        ))

        self.assertIsNone(np.testing.assert_allclose(parameters, vector))

    def test_generate_optimization_mask(self):
        mask = self.robot.generate_optimization_mask()
        self.assertEqual(sum(mask), 0)

        mask = self.robot.generate_optimization_mask(world_mask=True)
        self.assertEqual(sum(mask), 6)

        self.assertEqual(len(self.robot.generate_optimization_mask()),
                         len(self.robot.generate_optimization_mask(world_mask=True,
                                                                   robot_model_mask=True,
                                                                   tool_mask=True,
                                                                   joint_stiffness_mask=True)))

        with self.assertRaises(AssertionError):
            self.robot.generate_optimization_mask(world_mask=[True])
        with self.assertRaises(AssertionError):
            self.robot.generate_optimization_mask(robot_model_mask=[True])
        with self.assertRaises(AssertionError):
            self.robot.generate_optimization_mask(tool_mask=[True])
        with self.assertRaises(AssertionError):
            self.robot.generate_optimization_mask(joint_stiffness_mask=[True])

    def test_generate_parameter_bounds(self):
        mask = self.robot.generate_optimization_mask()
        bounds = self.robot.generate_parameter_bounds(mask)
        self.assertEqual(len(bounds), 0)

        mask = self.robot.generate_optimization_mask(world_mask=True,
                                                     robot_model_mask=True,
                                                     tool_mask=True,
                                                     joint_stiffness_mask=True)
        bounds = self.robot.generate_parameter_bounds(mask)
        self.assertTrue(len(bounds) > 0)
        for bound in bounds:
            for b in bound:
                self.assertIsNone(b)

    def test_jacobian_world(self):
        # TODO: test
        pass

    def test_jacobian_flange(self):
        # TODO: test
        pass

    def test_ik_fit_func(self):
        reference_frame = np.eye(4)
        test_angles = [0] * self.robot.num_dof()
        test_pose = self.robot.fk(test_angles, reference_frame=reference_frame)

        result_errors = self.robot.ik_fit_func(joint_angles=test_angles, pose=test_pose,
                                               reference_frame=reference_frame)
        self.assertAlmostEqual(sum(result_errors), 0, 1e-9)
