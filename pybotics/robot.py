from copy import copy

import numpy as np
from pybotics import kinematics, robot_model, geometry


class Robot:
    def __init__(self, robot_model, name='Pybot'):
        self.robot_model = robot_model
        self.tool = np.eye(4)
        self.world_frame = np.eye(4)
        self.current_joints = [0] * self.num_dof()
        self.joint_stiffness = [0] * self.num_dof()
        self.name = name

    def num_dof(self):
        return len(self.robot_model)

    def fk(self, joint_list=None, joint_limit=None, is_radians=False, torques=None, reference_frame=None):

        # validate input
        if torques is not None:
            assert len(torques) == self.num_dof()

        # define output
        transforms = []

        # load current robot joints if none given
        if joint_list is None:

            # if current joints are empty, assign zero
            if not self.current_joints:
                self.current_joints = [0] * self.num_dof()

            joint_list = [self.current_joints]

        # make sure joints are contained in a list
        elif not isinstance(joint_list[0], list):
            joint_list = [joint_list]

        # convert joints to radians
        if not is_radians:
            joint_list = np.deg2rad(joint_list)

        # define joint limit, transform up to n-th joint
        tool_transform = np.eye(4)
        if joint_limit is None:
            joint_limit = self.num_dof()
            tool_transform = self.tool

        # define reference frame
        if reference_frame is None:
            reference_frame = np.eye(4)

        # iterate through input
        for joints in joint_list:

            # define transform identity matrix to carry multiplications
            transform = np.eye(4)

            # multiply wrt reference frame
            transform = np.dot(reference_frame, transform)

            # multiply through the forward transforms of the joints
            for i in range(joint_limit):
                # add the current joint pose to the forward transform
                current_link = self.robot_model[i].copy()
                current_link[2] += joints[i]

                if torques is not None:
                    current_link[2] += torques[i] * self.joint_stiffness[i]

                # get the transform step
                current_link_transform = kinematics.forward_transform(current_link)
                transform = np.dot(transform, current_link_transform)

            # add tool transform
            transform = np.dot(transform, tool_transform)
            transforms.append(transform)

        # return only transform if only one joint config is given
        if len(transforms) == 1:
            transforms = transforms[0]

        return transforms

    def impair_robot_model(self, relative_error=0.05):
        # random error multiplier between [-1,1]
        error_attenuation = 2 * np.random.rand(self.robot_model.shape[0], self.robot_model.shape[1])
        error_attenuation -= 1

        # attenuate the multiplier wrt user input (e.g., limit to 5% of nominal)
        error_attenuation *= relative_error

        # create error delta
        error_delta = np.multiply(self.robot_model, error_attenuation)

        bit_mask = np.isclose(error_delta, np.zeros(error_delta.shape))

        if np.any(bit_mask):
            error_adjustment = 2 * np.random.rand(self.robot_model.shape[0], self.robot_model.shape[1])
            error_adjustment -= 1

            error_adjustment[:, 0] *= abs(error_delta[:, 0]).max()
            error_adjustment[:, 1] *= abs(error_delta[:, 1]).max()
            error_adjustment[:, 2] *= abs(error_delta[:, 2]).max()
            error_adjustment[:, 3] *= abs(error_delta[:, 3]).max()

            error_adjustment = np.multiply(error_adjustment, bit_mask.astype(int))

            error_delta += error_adjustment

        self.robot_model += error_delta

    def set_tool_xyz(self, xyz):
        for i, parameter in enumerate(xyz):
            self.tool[i, -1] = parameter

    def generate_optimization_vector(self, optimization_mask):
        optimization_mask = copy(optimization_mask)
        vector = []

        # get world frame
        world_vector = geometry.pose_2_xyzrpw(self.world_frame)
        for parameter in world_vector:
            truth = optimization_mask.pop(0)
            if truth:
                vector.append(parameter)

        # get MDH parameters
        parameters = np.reshape(self.robot_model, self.robot_model.size)
        for parameter in parameters:
            truth = optimization_mask.pop(0)
            if truth:
                vector.append(parameter)

        # get tool frame
        tool_vector = geometry.pose_2_xyzrpw(self.tool)
        for parameter in tool_vector:
            truth = optimization_mask.pop(0)
            if truth:
                vector.append(parameter)

        # get joint stiffness parameters
        for parameter in self.joint_stiffness:
            truth = optimization_mask.pop(0)
            if truth:
                vector.append(parameter)

        return vector

    def apply_optimization_vector(self, optimization_vector, optimization_mask):
        optimization_mask = copy(optimization_mask)

        # set world frame
        world_vector = geometry.pose_2_xyzrpw(self.world_frame)
        for i in range(len(world_vector)):
            truth = optimization_mask.pop(0)
            if truth:
                parameter = optimization_vector.pop(0)
                world_vector[i] = parameter
        self.world_frame = geometry.xyzrpw_2_pose(world_vector)

        # set MDH parameters
        for i in range(self.robot_model.size):
            truth = optimization_mask.pop(0)
            if truth:
                parameter = optimization_vector.pop(0)
                row = int(i / 4)
                col = int(i % 4)
                self.robot_model[row, col] = parameter

        # set tool frame
        tool_vector = geometry.pose_2_xyzrpw(self.tool)
        for i in range(len(tool_vector)):
            truth = optimization_mask.pop(0)
            if truth:
                parameter = optimization_vector.pop(0)
                tool_vector[i] = parameter
        self.tool = geometry.xyzrpw_2_pose(tool_vector)

        # get joint stiffness parameters
        for i in range(len(self.joint_stiffness)):
            truth = optimization_mask.pop(0)
            if truth:
                parameter = optimization_vector.pop(0)
                self.joint_stiffness[i] = parameter

    def generate_optimization_mask(self, world_mask=False, robot_model_mask=False, tool_mask=False,
                                   joint_stiffness_mask=False):

        if not isinstance(world_mask, list):
            world_mask = [world_mask] * 6

        if not isinstance(robot_model_mask, list):
            robot_model_mask = [robot_model_mask] * 4 * self.num_dof()

        if not isinstance(tool_mask, list):
            tool_mask = [tool_mask] * 6

        if not isinstance(joint_stiffness_mask, list):
            joint_stiffness_mask = [joint_stiffness_mask] * self.num_dof()

        mask = []
        mask.extend(world_mask)
        mask.extend(robot_model_mask)
        mask.extend(tool_mask)
        mask.extend(joint_stiffness_mask)

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

        glob_bounds = []
        glob_bounds.extend(world_bounds)
        glob_bounds.extend(robot_model_bounds)
        glob_bounds.extend(tool_bounds)
        glob_bounds.extend(joint_stiffness_bounds)

        bounds = []
        for i, truth in enumerate(optimization_mask):
            if truth:
                bounds.append(glob_bounds[i])

        return bounds
