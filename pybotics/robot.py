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
