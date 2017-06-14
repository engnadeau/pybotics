"""Robot module."""
from copy import copy, deepcopy
import itertools
import numpy as np  # type: ignore
import scipy.optimize  # type: ignore
from typing import Tuple, Union, List, Optional

from pybotics.constants import Constant
from pybotics.pybot_types import RobotBound
from pybotics.tool import Tool
from pybotics import exceptions
from pybotics import geometry
from pybotics import kinematics


class Robot:
    """Robot class."""

    def __init__(self,
                 robot_model: np.ndarray,
                 tool: Tool = Tool(),
                 world_frame: np.ndarray = None,
                 name: str = 'Pybot') -> None:
        """Construct a Robot object.

        :param robot_model: Modified Denavit-Hartenberg (MDH) parameters, size=[number of joints, 4]
        :param tool: tool object
        :param world_frame: 4x4 transformation locating robot base with respect to a reference frame
        :param name: robot name
        """
        # public members
        self.name = name
        self.robot_model = robot_model.reshape((-1, 4))
        self.tool = tool
        self.world_frame = np.eye(4) if world_frame is None else world_frame

        # private members
        self._joint_angles = [0] * self.num_dof()
        self._joint_torques = [0] * self.num_dof()
        self._joint_stiffness = [0] * self.num_dof()
        self._joint_angle_limits = [(-np.pi, np.pi)] * self.num_dof()

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
        if len(value) != self.num_dof():
            raise exceptions.PybotException
        elif not self.validate_joint_angles(value):
            raise exceptions.PybotException
        self._joint_angles = value

    @property
    def joint_angle_limits(self) -> List[Tuple[float, float]]:
        """Get current joint angle limits.

        :return: sequence of joint angle limits [rad]
        """
        return self._joint_angle_limits

    @joint_angle_limits.setter
    def joint_angle_limits(self, value: List[Tuple[float, float]]) -> None:
        """Set joint angle limits.

        :param value: sequence of joint angle limits [rad]
        :return:
        """
        if len(value) != self.num_dof():
            raise exceptions.PybotException
        self._joint_angle_limits = value

    @property
    def joint_compliance(self) -> np.ndarray:
        """Get joint stiffnesses.

        :return: sequence of joint stiffnesses [rad/N-mm]
        """
        return self._joint_stiffness

    @joint_compliance.setter
    def joint_compliance(self, value: np.ndarray) -> None:
        """Set joint stiffnesses.

        :param value: sequence of joint stiffnesses [rad/N-mm]
        :return:
        """
        if len(value) != self.num_dof():
            raise exceptions.PybotException
        self._joint_stiffness = value

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
        if len(value) != self.num_dof():
            raise exceptions.PybotException
        self._joint_torques = value

    def validate_joint_angles(self, joint_angles: np.ndarray) -> bool:
        """Validate a sequence of joint angles with respect to current limits.

        :param joint_angles: sequence of joint angles [rad]
        :return:
        """
        is_success = True
        for limit, joint_angle in zip(self.joint_angle_limits, joint_angles):
            if joint_angle > max(limit) or joint_angle < min(limit):
                is_success = False
                break
        return is_success

    def num_dof(self) -> int:
        """Get number of degrees of freedom (i.e., number of joints).

        :return: number of degrees of freedom
        """
        return len(self.robot_model)

    def fk(self) -> np.ndarray:
        """Calculate the forward kinematic 4x4 pose of the robot wrt the world frame.

         Uses the current joint angles and torques.

        :return: 4x4 transform
        """
        # define transform to carry matrix multiplications through joints
        transform = self.world_frame.copy()

        # multiply through the forward transforms of the joints
        for i, (joint, torque) in enumerate(zip(self._joint_angles, self._joint_torques)):
            # add the current joint pose to the forward transform
            transform = np.dot(transform, self.calculate_link_transform(i, joint, torque))

        # add tool transform
        transform = np.dot(transform, self.tool.tcp)

        return transform

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
        link[2] += torque * self.joint_compliance[link_index]
        transform = kinematics.forward_transform(link)
        return transform

    def generate_optimization_vector(self, optimization_mask: List[bool]) -> List[float]:
        """Generate an optimization vector from a given mask.

        :param optimization_mask: sequence of booleans describing desired parameters
        :return: sequence of parameters
        """
        parameters = list(itertools.chain(
            geometry.pose_2_xyzrpw(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.tool.tcp),
            self.joint_compliance
        ))
        parameters = list(itertools.compress(parameters, optimization_mask))

        return parameters

    def apply_optimization_vector(self,
                                  optimization_vector: List[float],
                                  optimization_mask: List[bool]) -> None:
        """Apply an optimization vector to the Robot.

        :param optimization_vector: sequence of parameters
        :param optimization_mask: mask sequence of booleans
        :return:
        """
        optimization_vector = copy(optimization_vector)

        # create parameter vector
        parameters = list(itertools.chain(
            geometry.pose_2_xyzrpw(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.tool.tcp),
            self.joint_compliance
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

        self.tool.tcp = geometry.xyzrpw_2_pose(parameters[:6])
        del parameters[:6]

        self.joint_compliance = parameters[:self.num_dof()]
        del parameters[:self.num_dof()]

    def generate_optimization_mask(self,
                                   world_mask: Union[bool, List[bool]] = False,
                                   robot_model_mask: Union[bool, List[bool]] = False,
                                   tool_mask: Union[bool, List[bool]] = False,
                                   joint_compliance_mask: Union[bool, List[bool]] = False) -> List[bool]:
        """Generate a mask used for optimization.

        :param world_mask: sequence of booleans representing the world frame parameters
        :param robot_model_mask: sequence of booleans representing the robot model parameters
        :param tool_mask: sequence of booleans representing the tool frame parameters
        :param joint_compliance_mask: sequence of booleans representing the joint compliance parameters
        :return: optimization mask
        """
        # TODO: use namedtuple for masks?

        if isinstance(world_mask, bool):
            world_mask = [world_mask] * 6
        elif len(world_mask) != 6:
            raise exceptions.PybotException

        if isinstance(robot_model_mask, bool):
            robot_model_mask = [robot_model_mask] * self.robot_model.size
        elif len(robot_model_mask) != self.robot_model.size:
            raise exceptions.PybotException

        if isinstance(tool_mask, bool):
            tool_mask = [tool_mask] * 6
        elif len(tool_mask) != 6:
            raise exceptions.PybotException

        if isinstance(joint_compliance_mask, bool):
            joint_compliance_mask = [joint_compliance_mask] * self.num_dof()
        elif len(joint_compliance_mask) != self.num_dof():
            raise exceptions.PybotException

        mask = list(itertools.chain(
            world_mask,
            robot_model_mask,
            tool_mask,
            joint_compliance_mask
        ))

        return mask

    def generate_parameter_bounds(self,
                                  optimization_mask: List[bool],
                                  world_bounds: RobotBound = None,
                                  robot_model_bounds: RobotBound = None,
                                  tool_bounds: RobotBound = None,
                                  joint_compliance_bounds: RobotBound = None
                                  ) -> RobotBound:
        """
        Generate optimization bounds.

        :param optimization_mask: optimization parameters boolean mask
        :param world_bounds: world transform bounds
        :param robot_model_bounds: MDH parameter bounds
        :param tool_bounds: tool transform bounds
        :param joint_compliance_bounds: joint compliance bounds
        :return: optimization bounds
        """
        world_bounds = [(None, None)] * 6 if world_bounds is None else world_bounds
        robot_model_bounds = [(
            None, None)] * self.robot_model.size if robot_model_bounds is None else robot_model_bounds
        tool_bounds = [(None, None)] * 6 if tool_bounds is None else tool_bounds
        joint_compliance_bounds = [(
            None, None)] * self.num_dof() if joint_compliance_bounds is None else joint_compliance_bounds

        bounds = list(itertools.chain(
            world_bounds,
            robot_model_bounds,
            tool_bounds,
            joint_compliance_bounds
        ))

        bounds = list(itertools.compress(bounds, optimization_mask))

        return bounds

    def ik(self, pose: np.ndarray) -> Optional[np.ndarray]:
        """
        Calculate iteratively the inverse kinematics for a given pose.

        Uses the current joint angles as a seed value.

        :param pose: 4x4 transform of the current tool in the world frame
        :return: sequence of joint angles [rad]
        """
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
            if optimize_result.fun.max() < 1e-1 and robot.validate_joint_angles(joint_angles):
                result = joint_angles
                break
            else:
                robot.random_joints()

        return result

    def calculate_tool_wrench(self) -> np.ndarray:
        """
        Calculate the wrench (force and moment/torque) generated by the tool in the flange frame.

        :return: tool wrench vector (force [N] + torque [N-mm])
        """
        # find FK, get flange position and weight vector of tool
        tool_pose = self.fk()
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

    def random_joints(self) -> None:
        """
        Set random joint angle values within the limits.

        :return:
        """
        joint_angles = []
        for limits in self.joint_angle_limits:
            joint_angles.append(np.random.uniform(min(limits), max(limits)))
        self.joint_angles = joint_angles


def _ik_fit_func(joint_angles: np.ndarray,
                 robot: Robot,
                 pose: np.ndarray) -> np.ndarray:
    """Fitness function used internally for the inverse kinematics.

    :param joint_angles: sequence of joint angles [rad]
    :param robot: Robot object
    :param pose: 4x4 transform
    :return:
    """
    joint_angles = geometry.wrap_2_pi(joint_angles)
    robot.joint_angles = joint_angles
    actual_pose = robot.fk()

    error = actual_pose - pose
    error = error.flatten()
    return error
