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
        else:
            torques = [0] * self.num_dof()

        # define transform to carry matrix multiplications through joints
        if reference_frame is None:
            transform = np.eye(4)
        else:
            transform = reference_frame

        # multiply through the forward transforms of the joints
        for i, (joint, torque) in enumerate(zip(joint_angles, torques)):
            # add the current joint pose to the forward transform
            transform = np.dot(transform, self.calculate_link_transform(i, joint, torque))

        # add tool transform
        transform = np.dot(transform, self.tool)

        return transform

    def calculate_link_transform(self, link_index, joint_angle, torque=0):
        link = self.robot_model[link_index].copy()
        link[2] += joint_angle
        link[2] += torque * self.joint_stiffness[link_index]
        link_transform = kinematics.forward_transform(link)
        return link_transform

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
            robot_model_bounds = [(None, None)] * self.robot_model.size

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

        # transpose joint angle limits to least_squares format
        bounds = np.array(self.joint_angle_limits)
        bounds = tuple(map(tuple, bounds.transpose()))

        result = None
        for _ in range(5):
            optimize_result = scipy.optimize.least_squares(fun=self.ik_fit_func,
                                                           x0=joint_angles,
                                                           args=(pose, reference_frame),
                                                           bounds=bounds,
                                                           )

            joint_angles = optimize_result.x
            if optimize_result.fun.max() < 1e-1 and self.validate_joint_angles(joint_angles):
                result = joint_angles
                break
            else:
                joint_angles = robot_utilities.random_joints(self.joint_angle_limits)

        return result

    def ik_fit_func(self, joint_angles, pose, reference_frame):
        joint_angles = geometry.wrap_2_pi(joint_angles)
        actual_pose = self.fk(joint_angles, reference_frame=reference_frame)

        error = actual_pose - pose
        error = error.flatten()
        return error

    def calculate_joint_torques(self, joint_angles, wrench):
        """
        Calculate the joint torques due to external force applied to the flange frame.

        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.

        :param joint_angles:
        :param force:
        :return:
        """

        # split wrench into components
        force = wrench[:3].copy()
        moment = wrench[-3:].copy()

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base, each iteration calculates for link i-1
        for i, joint_angle in reversed(list(enumerate(joint_angles))):
            if i == 0:
                break

            # get current link transform
            transform = self.calculate_link_transform(i, joint_angle)

            # calculate force applied to current link
            rotation = transform[:3, :3]
            force = np.dot(rotation, force)

            # calculate moment applied to current link
            position = transform[:3, -1]
            moment = np.dot(rotation, moment) + np.cross(position, force)

            # append z-component as joint torque
            joint_torques.append(moment[-1])

        # reverse torques into correct order
        return list(reversed(joint_torques))
