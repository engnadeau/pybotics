from copy import copy

import numpy as np
from pybotics import kinematics, robot_model, geometry


class Robot:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tool = np.eye(4)
        self.world_frame = np.eye(4)
        self.current_joints = [0] * self.num_dof()
        self.joint_stiffness = [0] * self.num_dof()

    def num_dof(self):
        return len(self.robot_model)

    def fk(self, joint_list=None, joint_limit=None, is_radians=False, torques=None):

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

        # iterate through input
        for joints in joint_list:

            # define transform identity matrix to carry multiplications
            transform = np.eye(4)

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
