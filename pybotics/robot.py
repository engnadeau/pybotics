from copy import copy
import itertools
import numpy as np
import scipy.optimize

from pybotics import kinematics, geometry, robot_utilities


class Robot:
    def __init__(self, robot_model, name='Pybot'):
        self.robot_model = robot_model
        self.tool = np.eye(4)
        self.world_frame = np.eye(4)
        self.joint_stiffness = [0] * self.num_dof()
        self.name = name
        self.joint_angle_limits = [(-np.pi, np.pi)] * self.num_dof()

    def validate_joint_angles(self, joint_angles):
        is_success = True
        for limit, joint_angle in zip(self.joint_angle_limits, joint_angles):
            if joint_angle > max(limit) or joint_angle < min(limit):
                is_success = False
                break

        return is_success

    def num_dof(self):
        return len(self.robot_model)

    def fk(self, joint_angles, torques=None, reference_frame=None):

        # validate input
        assert len(joint_angles) == self.num_dof()
        if torques is not None:
            assert len(torques) == self.num_dof()

        # define transform to carry matrix multiplications through joints
        if reference_frame is None:
            transform = np.eye(4)
        else:
            transform = reference_frame

        # multiply through the forward transforms of the joints
        for i, joint in enumerate(joint_angles):
            # add the current joint pose to the forward transform
            current_link = self.robot_model[i].copy()
            current_link[2] += joint

            if torques is not None:
                current_link[2] += torques[i] * self.joint_stiffness[i]

            # get the transform step
            current_link_transform = kinematics.forward_transform(current_link)
            transform = np.dot(transform, current_link_transform)

        # add tool transform
        transform = np.dot(transform, self.tool)

        return transform

    def set_tool_xyz(self, xyz):
        for i, parameter in enumerate(xyz):
            self.tool[i, -1] = parameter

    def generate_optimization_vector(self, optimization_mask):
        parameters = itertools.chain(
            geometry.pose_2_xyzrpw(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.tool),
            self.joint_stiffness
        )
        parameters = list(itertools.compress(parameters, optimization_mask))

        return parameters

    def apply_optimization_vector(self, optimization_vector, optimization_mask):
        optimization_vector = copy(optimization_vector)

        # create parameter vector
        parameters = list(itertools.chain(
            geometry.pose_2_xyzrpw(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.tool),
            self.joint_stiffness
        ))

        # update vector wrt optimizations and mask
        for i, truth in enumerate(optimization_mask):
            if truth:
                parameters[i] = optimization_vector.pop(0)

        # update self wrt new vector
        self.world_frame = geometry.xyzrpw_2_pose(parameters[:6])
        del parameters[:6]

        self.robot_model = np.array(parameters[:self.robot_model.size]).reshape((-1, 4))
        del parameters[:self.robot_model.size]

        self.tool = geometry.xyzrpw_2_pose(parameters[:6])
        del parameters[:6]

        self.joint_stiffness = parameters[:self.num_dof()]
        del parameters[:self.num_dof()]

    def generate_optimization_mask(self, world_mask=False, robot_model_mask=False, tool_mask=False,
                                   joint_stiffness_mask=False):

        if not isinstance(world_mask, list):
            world_mask = [world_mask] * 6
        else:
            assert len(world_mask) == 6

        if not isinstance(robot_model_mask, list):
            robot_model_mask = [robot_model_mask] * self.robot_model.size
        else:
            assert len(robot_model_mask) == self.robot_model.size

        if not isinstance(tool_mask, list):
            tool_mask = [tool_mask] * 6
        else:
            assert len(tool_mask) == 6

        if not isinstance(joint_stiffness_mask, list):
            joint_stiffness_mask = [joint_stiffness_mask] * self.num_dof()
        else:
            assert len(joint_stiffness_mask) == self.num_dof()

        mask = list(itertools.chain(
            world_mask,
            robot_model_mask,
            tool_mask,
            joint_stiffness_mask
        ))

        return mask

    def generate_parameter_bounds(self, optimization_mask, world_bounds=None, robot_model_bounds=None, tool_bounds=None,
                                  joint_stiffness_bounds=None):
        if world_bounds is None:
            world_bounds = [(None, None)] * 6

        if robot_model_bounds is None:
            robot_model_bounds = [(None, None)] * 4 * self.num_dof()

        if tool_bounds is None:
            tool_bounds = [(None, None)] * 6

        if joint_stiffness_bounds is None:
            joint_stiffness_bounds = [(None, None)] * self.num_dof()

        bounds = itertools.chain(
            world_bounds,
            robot_model_bounds,
            tool_bounds,
            joint_stiffness_bounds
        )

        bounds = list(itertools.compress(bounds, optimization_mask))

        return bounds

    def ik(self, pose, joint_angles=None, reference_frame=None):
        # set initial joints
        if joint_angles is not None:
            assert len(joint_angles) == self.num_dof()
        else:
            joint_angles = [0] * self.num_dof()

        if not isinstance(joint_angles, np.ndarray):
            joint_angles = np.array(joint_angles)

        bounds = np.array(self.joint_angle_limits)
        bounds = tuple(map(tuple, bounds.transpose()))

        is_success = False
        max_iterations = 5
        current_iteration = 0
        result = None
        while not is_success and current_iteration < max_iterations:
            current_iteration += 1
            optimize_result = scipy.optimize.least_squares(fun=self.ik_fit_func,
                                                           x0=joint_angles,
                                                           args=(pose, reference_frame),
                                                           bounds=bounds,
                                                           )

            result = optimize_result.x
            if optimize_result.fun.max() < 1e-1 and self.validate_joint_angles(result):
                is_success = True
            else:
                joint_angles = robot_utilities.random_joints(self.joint_angle_limits)

        if not is_success:
            result = None

        return result

    def jacobian_world(self, joint_angles=None):
        # set initial joints
        if joint_angles is not None:
            assert len(joint_angles) == self.num_dof()
        else:
            joint_angles = self.current_joints

        jacobian_flange = self.jacobian_flange(joint_angles)
        pose = self.fk(joint_angles)
        rotation = pose[0:3, 0:3]
        jacobian_transform = np.zeros((6, 6))
        jacobian_transform[:3, :3] = rotation
        jacobian_transform[3:, 3:] = rotation
        jacobian_world = np.dot(jacobian_transform, jacobian_flange)

        return jacobian_world

    def jacobian_flange(self, joint_angles=None):
        # set initial joints
        if joint_angles is not None:
            assert len(joint_angles) == self.num_dof()
        else:
            joint_angles = self.current_joints

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.num_dof()))
        current_transform = copy(self.tool)

        for i in reversed(range(self.num_dof())):
            d = np.array([
                -current_transform[0, 0] * current_transform[1, 3] + current_transform[1, 0] * current_transform[0, 3],
                - current_transform[0, 1] * current_transform[1, 3] + current_transform[1, 1] * current_transform[0, 3],
                - current_transform[0, 2] * current_transform[1, 3] + current_transform[1, 2] * current_transform[0, 3],
            ])
            delta = current_transform[2, 0:3]
            jacobian_flange[:, i] = np.hstack((d, delta))
            current_link = self.robot_model[i]
            current_link_transform = kinematics.forward_transform(current_link, joint_angle=joint_angles[i])
            current_transform = np.dot(current_link_transform, current_transform)

        return jacobian_flange

    def ik_fit_func(self, joint_angles, pose, reference_frame):
        geometry.wrap_2_pi(joint_angles)
        actual_pose = self.fk(joint_angles, reference_frame=reference_frame)

        error = actual_pose - pose
        error = error.flatten()
        return error
