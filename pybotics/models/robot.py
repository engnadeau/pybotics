"""Robot module."""
import itertools
from collections import namedtuple
from copy import copy, deepcopy
from typing import List, Optional, Union, Iterable

import numpy as np  # type: ignore
import scipy.optimize  # type: ignore

from pybotics import geometry
from pybotics import kinematics
from pybotics.constants import Constant
from pybotics.data_validation import validate_4x4_matrix, validate_1d_vector

from pybotics.calibration.optimization_mask import OptimizationMask
from pybotics.models.tool import Tool


class Robot:
    """Robot class."""

    def __init__(self,
                 robot_model: np.ndarray,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None,
                 random_state: Optional[np.random.RandomState] = None) -> None:
        """Construct a Robot object.

        :param robot_model: Modified Denavit-Hartenberg (MDH) parameters, shape=[number of joints, 4]
        :param tool: tool object
        :param world_frame: 4x4 transformation locating robot base with respect to a reference frame
        :param name: robot name
        """
        # validate input
        if robot_model.shape[1] != 4:
            raise ValueError('Required: robot_model.shape[1] == 4')

        validate_4x4_matrix(world_frame)

        # public members
        self.robot_model = robot_model
        self.tool = Tool() if tool is None else tool
        self.world_frame = np.eye(4) if world_frame is None else world_frame
        self.random_state = np.random.RandomState() if random_state is None else random_state

        # private members
        self._joint_angles = np.zeros(self.num_dof())
        self._joint_torques = np.zeros(self.num_dof())
        self._joint_compliances = np.zeros(self.num_dof())
        self._joint_angle_limits = np.repeat((-np.pi, np.pi), self.num_dof()).reshape((2, -1))

        # calibration members
        self._optimization_mask = OptimizationMask()

    @property
    def optimization_mask(self):
        return self._optimization_mask

    @optimization_mask.setter
    def optimization_mask(self, mask: OptimizationMask):
        self._optimization_mask = mask

    def generate_optimization_vector(self):
        # noinspection PyProtectedMember
        for field in self._optimization_mask._fields:
            attr = getattr(self, field)



    @property
    def joint_angles(self) -> np.ndarray:
        """Get current joint angles.

        :return: joint angles [rad]
        """
        return self._joint_angles

    @joint_angles.setter
    def joint_angles(self, value: np.ndarray) -> None:
        """Set current joint angles.

        :param value: vector of joint angles [rad]
        :return:
        """

        validate_1d_vector(value, length=self.num_dof())

        if not self.check_joint_angle_limits(value):
            raise ValueError('Joint angles outside limits.')

        self._joint_angles = value

    @property
    def joint_angle_limits(self) -> np.ndarray:
        """Get current joint angle limits.

        :return: sequence of joint angle limits [rad]
        """
        return self._joint_angle_limits

    @joint_angle_limits.setter
    def joint_angle_limits(self, value: np.ndarray) -> None:
        """Set joint angle limits.

        :param value: sequence of joint angle limits [rad]
        :return:
        """
        # validate input
        if not np.array_equal(value.shape, [self.num_dof(), 2]):
            raise ValueError('Required: value.shape == [{}, 2].'.format(self.num_dof()))

        if np.any(np.greater_equal(value[0, :], value[1, :])):
            raise ValueError('Required: first row (lower limits) must be less than second row (upper limits).')

        self._joint_angle_limits = value

    @property
    def joint_compliances(self) -> np.ndarray:
        """Get joint stiffnesses.

        :return: sequence of joint stiffnesses [rad/N-mm]
        """
        return self._joint_compliances

    @joint_compliances.setter
    def joint_compliances(self, value: np.ndarray) -> None:
        """Set joint stiffnesses.

        :param value: sequence of joint stiffnesses [rad/N-mm]
        :return:
        """
        validate_1d_vector(value, length=self.num_dof())
        self._joint_compliances = value

    @property
    def joint_torques(self) -> np.ndarray:
        """Get current joint torques.

        :return: sequence of joint torques [N-mm]
        """
        return self._joint_torques

    @joint_torques.setter
    def joint_torques(self, value: np.ndarray) -> None:
        """Set current joint torques.

        :param value: sequence of joint torques [N-mm]
        :return:
        """
        validate_1d_vector(value, length=self.num_dof())
        self._joint_torques = value

    def check_joint_angle_limits(self, joint_angles: np.ndarray) -> bool:
        """Validate a sequence of joint angles with respect to current limits.

        :param joint_angles: sequence of joint angles [rad]
        :return:
        """
        lower_limit_truth = np.greater_equal(joint_angles, self._joint_angle_limits[0, :])
        upper_limit_truth = np.less_equal(joint_angles, self._joint_angle_limits[1, :])

        is_success = np.all(lower_limit_truth) and np.all(upper_limit_truth)

        return is_success

    def num_dof(self) -> int:
        """Get number of degrees of freedom (i.e., number of joints).

        :return: number of degrees of freedom
        """
        return self.robot_model.shape[0]

    def fk(self,
           joint_angles: Optional[np.ndarray] = None,
           joint_torques: Optional[np.ndarray] = None) -> np.ndarray:
        """Calculate the forward kinematic 4x4 pose of the robot wrt the world frame.

         Uses the current joint angles and torques.

        :return: 4x4 transform
        """
        # parse input
        if joint_angles is None:
            joint_angles = self._joint_angles.copy()  # type: np.ndarray
        else:
            self._validate_joint_angles(joint_angles)

        if joint_torques is None:
            joint_torques = self._joint_torques.copy()  # type: np.ndarray
        else:
            self._validate_joint_torques(joint_torques)

        # collect the needed transforms
        transforms = [self.world_frame.copy()]
        # TODO: investigate why pycharm warns here.
        # noinspection PyArgumentList
        transforms.extend(list(map(self.calculate_link_transform,
                                   range(self.num_dof()),
                                   joint_angles,
                                   joint_torques)))
        transforms.append(self.tool.tcp)

        # matrix multiply
        fk_pose = np.linalg.multi_dot(transforms)

        return fk_pose

    def calculate_link_transform(self,
                                 link_index: int,
                                 joint_angle: float,
                                 torque: float = 0) -> np.ndarray:
        """Calculate the forward transform of a given link.

        :param link_index: index of link (row of robot model)
        :param joint_angle: joint angle [rad]
        :param torque: joint torque [N-mm]
        :return:
        """
        link = self.robot_model[link_index].copy()
        link[2] += joint_angle
        link[2] += torque * self.joint_compliances[link_index]
        transform = kinematics.forward_transform(link)
        return transform

    def apply_optimization_vector(self,
                                  optimization_vector: List[float],
                                  optimization_mask: List[bool]) -> None:
        """Apply an optimization vector to the Robot.

        :param optimization_vector: sequence of parameters
        :param optimization_mask: mask sequence of booleans
        :return:
        """
        # TODO: review and clean code
        optimization_vector = copy(optimization_vector)

        # create parameter vector
        parameters = list(itertools.chain(
            geometry.pose_2_euler_zyx(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_euler_zyx(self.tool.tcp),
            self.joint_compliances
        ))

        # update vector wrt optimizations and mask
        for i, truth in enumerate(optimization_mask):
            if truth:
                parameters[i] = optimization_vector.pop(0)

        # update self wrt new vector
        self.world_frame = geometry.euler_zyx_2_pose(parameters[:6])
        del parameters[:6]

        self.robot_model = np.array(parameters[:self.robot_model.size]).reshape((-1, 4))
        del parameters[:self.robot_model.size]

        self.tool.tcp = geometry.euler_zyx_2_pose(parameters[:6])
        del parameters[:6]

        self.joint_compliances = parameters[:self.num_dof()]
        del parameters[:self.num_dof()]

    def ik(self,
           pose: np.ndarray,
           joint_angles: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Calculate iteratively the inverse kinematics for a given pose.

        :param pose: 4x4 transform of the current tool in the world frame
        :return: sequence of joint angles [rad]
        """
        # TODO: fix SOLVER
        # TODO: use given joint angles
        # copy self for iterative solver
        robot = deepcopy(self)

        # transpose joint angle limits to least_squares format
        bounds = np.array(robot.joint_angle_limits)
        bounds = tuple(map(tuple, bounds.transpose()))

        # solver variables
        result = None
        for _ in range(5):
            optimize_result = scipy.optimize.least_squares(fun=_ik_fit_func,
                                                           x0=robot.joint_angles,
                                                           args=(robot, pose),
                                                           bounds=bounds)

            joint_angles = optimize_result.x
            if optimize_result.fun.max() < 1e-1 and robot.check_joint_angle_limits(joint_angles):
                result = joint_angles
                break
            else:
                robot.generate_random_joints()

        return result

    def calculate_tool_wrench(self,
                              joint_angles: Optional[np.ndarray] = None,
                              joint_torques: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Calculate the wrench (force and moment) generated by the tool in the flange frame.

        :return: tool wrench vector (force [N] + torque [N-mm])
        """
        # TODO: review and clean code
        # find FK, get flange position and weight vector of tool
        tool_pose = self.fk(joint_angles=joint_angles, joint_torques=joint_torques)
        flange_pose = np.dot(tool_pose, np.linalg.inv(self.tool.tcp))
        weight_vector = flange_pose[2, :-1]  # reference frame z-direction wrt flange
        weight_vector /= np.linalg.norm(weight_vector)  # make sure it's a unit vector

        force = (self.tool.mass * Constant.GRAVITY.value) * weight_vector

        # calculate moment generate from CG position and force
        moment = np.cross(self.tool.cg, force)

        # combine and return
        wrench = np.hstack((force, moment))
        return wrench

    def calculate_external_wrench_joint_torques(self, wrench: np.ndarray) -> List[float]:
        """
        Calculate the joint torques due to external force applied to the flange frame.

        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.

        :param wrench: external wrench (force [N] + torque [N-mm])
        :return: sequence of joint torques [N-mm]
        """
        # TODO: review and clean code
        # split wrench into components
        force = wrench[:3]
        moment = wrench[-3:]

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base
        # each iteration calculates for link i-1
        for i, joint_angle in reversed(list(enumerate(self._joint_angles))):
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

    def generate_random_joints(self) -> np.ndarray:
        """
        Generate random joint angle values within the limits.

        :return:
        """
        return self.random_state.uniform(low=self._joint_angle_limits[0, :],
                                         high=self._joint_angle_limits[1, :])


def _ik_fit_func(joint_angles: np.ndarray,
                 robot: Robot,
                 pose: np.ndarray) -> np.ndarray:
    """Fitness function used internally for the inverse kinematics.

    :param joint_angles: sequence of joint angles [rad]
    :param robot: Robot object
    :param pose: 4x4 transform
    :return:
    """
    # TODO: review and clean code
    joint_angles = geometry.wrap_2_pi(joint_angles)
    robot.joint_angles = joint_angles
    actual_pose = robot.fk()

    error = actual_pose - pose
    error = error.flatten()
    return error
