"""Robot module."""
from typing import Optional, Sequence, Sized

import numpy as np  # type: ignore

from pybotics.errors import SequenceError, PyboticsError
from pybotics.frame import Frame
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot_json_encoder import RobotJSONEncoder
from pybotics.robot_optimization_mask import RobotOptimizationMask
from pybotics.tool import Tool
from pybotics.validation import is_1d_ndarray, is_1d_sequence


class Robot(Sized):
    """Robot manipulator class."""

    def __init__(self, kinematic_chain: KinematicChain,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        """
        Construct a robot instance.

        :param kinematic_chain: kinematic chain of the manipulator
        :param tool: attached tool
        :param world_frame: transform of robot base with respect to the world
        """
        self._position = np.zeros(kinematic_chain.num_dof)
        self._position_limits = np.repeat((-np.inf, np.inf),
                                          kinematic_chain.num_dof
                                          ).reshape((2, -1))

        self.world_frame = Frame() if world_frame is None else world_frame
        self.kinematic_chain = kinematic_chain
        self.tool = Tool() if tool is None else tool

    def __len__(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return self.kinematic_chain.num_dof

    def __repr__(self) -> str:
        """
        Get the debug representation of the robot model.

        :return:
        """
        encoder = RobotJSONEncoder(sort_keys=True, indent=4)
        return encoder.encode(self)

    def __str__(self) -> str:
        """
        Get the string representation of the robot model.

        :return:
        """
        return self.__repr__()

    def apply_optimization_vector(self, vector: np.ndarray) -> None:
        """
        Update the current instance with new optimization parameters.

        :param vector: new parameters to apply
        """
        split_vector = np.split(
            vector,
            np.cumsum(
                [len(self.world_frame.optimization_vector),
                 len(self.kinematic_chain.optimization_vector)]
            ))
        self.world_frame.apply_optimization_vector(split_vector[0])
        self.kinematic_chain.apply_optimization_vector(split_vector[1])
        self.tool.apply_optimization_vector(split_vector[2])

    def fk(self, position: Optional[Sequence[float]] = None) -> np.ndarray:
        """
        Compute the forward kinematics of a given position.

        Uses the current position if None is given.
        :param position:
        :return: 4x4 transform matrix of the FK pose
        """
        # validate
        if position is not None:
            if not is_1d_sequence(position, self.num_dof):
                raise SequenceError('position', self.num_dof)
        else:
            position = self.position

        # gather transforms
        transforms = [self.world_frame.matrix]
        transforms.extend(self.kinematic_chain.transforms(position))
        transforms.append(self.tool.matrix)

        # matrix multiply through transforms
        pose = np.eye(4)
        for t in transforms:
            pose = np.dot(pose, t)

        return pose

    @property
    def num_dof(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return self.kinematic_chain.num_dof

    @property
    def optimization_mask(self) -> RobotOptimizationMask:
        """
        Return the mask used to select the optimization parameters.

        :return: mask
        """
        mask = RobotOptimizationMask(
            world_frame=self.world_frame.optimization_mask,
            kinematic_chain=self.kinematic_chain.optimization_mask,
            tool=self.tool.optimization_mask)
        return mask

    @optimization_mask.setter
    def optimization_mask(self, value: RobotOptimizationMask) -> None:
        # FIXME: remove `# type: ignore`
        # FIXME: remove kc; it's there to shorten line length for flake8
        # https://github.com/python/mypy/issues/4167
        self.world_frame.optimization_mask = value.world_frame  # type: ignore
        kc = value.kinematic_chain
        self.kinematic_chain.optimization_mask = kc  # type: ignore
        self.tool.optimization_mask = value.tool  # type: ignore

    @property
    def optimization_vector(self) -> np.ndarray:
        """
        Return the values of parameters being optimized.

        :return: optimization parameter values
        """
        world = self.world_frame.optimization_vector
        kinematic_chain = self.kinematic_chain.optimization_vector
        tool = self.tool.optimization_vector

        vector = np.hstack((world, kinematic_chain, tool))
        return vector

    @property
    def position(self) -> np.ndarray:
        """
        Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._position

    @position.setter
    def position(self, value: np.ndarray) -> None:
        # TODO: check if position is in limits
        if is_1d_ndarray(value, self.num_dof):
            self._position = value
        else:
            raise SequenceError('value', self.num_dof)

    @property
    def position_limits(self) -> np.ndarray:
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._position_limits

    @position_limits.setter
    def position_limits(self, value: np.ndarray) -> None:
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise PyboticsError(
                'position_limits must have shape=(2,{})'.format(len(self)))
        self._position_limits = value

    def jacobian_world(self, position: Optional[Sequence[float]] = None):
        if position is not None:
            if not is_1d_sequence(position, self.num_dof):
                raise SequenceError('position', self.num_dof)
        else:
            position = self.position

        jacobian_flange = self.jacobian_flange(position)
        pose = self.fk(position)
        rotation = pose[0:3, 0:3]
        jacobian_transform = np.zeros((6, 6))
        jacobian_transform[:3, :3] = rotation
        jacobian_transform[3:, 3:] = rotation
        jacobian_world = np.dot(jacobian_transform, jacobian_flange)

        return jacobian_world

    def jacobian_flange(self, position: Optional[Sequence[float]] = None):
        if position is not None:
            if not is_1d_sequence(position, self.num_dof):
                raise SequenceError('position', self.num_dof)
        else:
            position = self.position

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.num_dof))
        current_transform = self.tool.matrix.copy()

        for i in reversed(range(self.num_dof)):
            d = np.array([
                -current_transform[0, 0] * current_transform[1, 3] +
                current_transform[1, 0] * current_transform[0, 3],
                - current_transform[0, 1] * current_transform[1, 3] +
                current_transform[1, 1] * current_transform[0, 3],
                - current_transform[0, 2] * current_transform[1, 3] +
                current_transform[1, 2] * current_transform[0, 3],
            ])
            delta = current_transform[2, 0:3]
            jacobian_flange[:, i] = np.hstack((d, delta))
            current_link = self.kinematic_chain.links[i]
            current_link_transform = current_link.transform(position[i])
            current_transform = np.dot(current_link_transform,
                                       current_transform)

        return jacobian_flange

    def calculate_joint_torques(self, position, wrench):
        """
        Calculate the joint torques due to external force applied to the flange frame.
        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
        :param position:
        :param force:
        :return:
        """

        # split wrench into components
        force = wrench[:3].copy()
        moment = wrench[-3:].copy()

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base
        # each iteration calculates for link i-1
        for i, p in reversed(list(enumerate(position))):
            if i == 0:
                break

            # get current link transform
            transform = self.kinematic_chain.links[i].transform(p)

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
