import numpy as np

from pybotics import kinematics, robot_model


class Robot:
    def __init__(self):
        self.robot_model = None
        self.tool = np.eye(4)
        self.world_frame = np.eye(4)
        self.current_joints = []

    def num_dof(self):
        return len(self.robot_model)

    def fk(self, joints=None):

        # load current robot joints if none given
        if joints is None:
            joints = self.current_joints

        # define transform identity matrix to carry multiplications
        transform = np.eye(4)

        # multiply through the forward transforms of the joints
        for i in range(self.num_dof()):
            # add the current joint pose to the forward transform
            current_link = self.robot_model[i].copy()
            current_link[3] += np.deg2rad(joints[i])

            # get the transform step
            current_link_transform = kinematics.forward_transform(current_link)
            transform = np.dot(transform, current_link_transform)

        # add tool transform
        transform = np.dot(transform, self.tool)

        return transform

    def impair_robot_model(self, relative_error=0.05):
        # random error multiplier between [-1,1]
        error_attenuation = 2 * np.random.rand(self.robot_model.shape[0], self.robot_model.shape[1])
        error_attenuation -= 1

        # attenuate the multiplier wrt user input (e.g., limit to 5% of nominal)
        error_attenuation *= relative_error

        # create error delta
        # TODO: what about zero values? Take the average of the column?
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
