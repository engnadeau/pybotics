from copy import copy, deepcopy
import itertools
import numpy as np
import scipy.optimize
from typing import Iterable, Tuple
from pybotics.tool import Tool
from pybotics.exceptions import *
from pybotics import kinematics
from pybotics import geometry


class Robot:
    def __init__(self,
                 robot_model: np.ndarray,
                 tool: Tool = Tool(),
                 world_frame: np.ndarray = None,
                 joint_stiffness: Iterable[float] = None,
                 joint_angle_limits: Iterable[Tuple[float]] = None,
                 name: str = 'Pybot'):

        # public members
        self.name = name
        self.robot_model = robot_model.reshape((-1, 4))
        self.tool = tool
        self.world_frame = np.eye(4) if world_frame is None else world_frame
        self.joint_stiffness = [0] * self.num_dof() if joint_stiffness is None else joint_stiffness
        self.joint_angle_limits = [(-np.pi,
                                    np.pi)] * self.num_dof() if joint_angle_limits is None else joint_angle_limits

        # private members
        self._joint_angles = [0] * self.num_dof()
        self._joint_torques = [0] * self.num_dof()

    @property
    def joint_angles(self):
        return self._joint_angles

    @joint_angles.setter
    def joint_angles(self, value: Iterable[float]):
        if len(value) != self.num_dof():
            raise PybotException
        elif not self.validate_joint_angles(value):
            raise PybotException
        self._joint_angles = value

    @property
    def joint_torques(self):
        return self._joint_torques

    @joint_torques.setter
    def joint_torques(self, value: Iterable[float]):

        if len(value) != self.num_dof():
            raise PybotException
        self._joint_torques = value

    def joint_torques_from_external_wrench(self, wrench: Iterable[float]):
        """
         Calculate the joint torques due to external force applied to the flange frame.

         Method from:
         5.9 STATIC FORCES IN MANIPULATORS
         Craig, John J. Introduction to robotics: mechanics and control.
         Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.

         :param torques:
         :param external_wrench:
         :return:
         """

        # split wrench into components
        force = wrench[:3].copy()
        moment = wrench[-3:].copy()

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base, each iteration calculates for link i-1
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
        self._joint_torques = list(reversed(joint_torques))

    def validate_joint_angles(self, joint_angles):
        is_success = True
        for limit, joint_angle in zip(self.joint_angle_limits, joint_angles):
            if joint_angle > max(limit) or joint_angle < min(limit):
                is_success = False
                break

        return is_success

    def num_dof(self):
        return len(self.robot_model)

    def fk(self):
        # define transform to carry matrix multiplications through joints
        transform = self.world_frame.copy()

        # multiply through the forward transforms of the joints
        for i, (joint, torque) in enumerate(zip(self._joint_angles, self._joint_torques)):
            # add the current joint pose to the forward transform
            transform = np.dot(transform, self.calculate_link_transform(i, joint, torque))

        # add tool transform
        transform = np.dot(transform, self.tool.tcp)

        return transform

    def calculate_link_transform(self, link_index, joint_angle, torque=0):
        link = self.robot_model[link_index].copy()
        link[2] += joint_angle
        link[2] += torque * self.joint_stiffness[link_index]
        transform = kinematics.forward_transform(link)
        return transform

    def set_tool_xyz(self, xyz: Iterable[float]):
        for i, parameter in enumerate(xyz):
            self.tool.tcp[i, -1] = parameter

    def generate_optimization_vector(self, optimization_mask):
        parameters = itertools.chain(
            geometry.pose_2_xyzrpw(self.world_frame),
            self.robot_model.ravel(),
            geometry.pose_2_xyzrpw(self.tool.tcp),
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
            geometry.pose_2_xyzrpw(self.tool.tcp),
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

        self.tool.tcp = geometry.xyzrpw_2_pose(parameters[:6])
        del parameters[:6]

        self.joint_stiffness = parameters[:self.num_dof()]
        del parameters[:self.num_dof()]

    def generate_optimization_mask(self,
                                   world_mask=False,
                                   robot_model_mask=False,
                                   tool_mask=False,
                                   joint_stiffness_mask=False):

        # TODO: use namedtuple for masks?

        if isinstance(world_mask, bool):
            world_mask = [world_mask] * 6
        elif len(world_mask) != 6:
            raise PybotException

        if isinstance(robot_model_mask, bool):
            robot_model_mask = [robot_model_mask] * self.robot_model.size
        elif len(robot_model_mask) != self.robot_model.size:
            raise PybotException

        if isinstance(tool_mask, bool):
            tool_mask = [tool_mask] * 6
        elif len(tool_mask) != 6:
            raise PybotException

        if isinstance(joint_stiffness_mask, bool):
            joint_stiffness_mask = [joint_stiffness_mask] * self.num_dof()
        elif len(joint_stiffness_mask) != self.num_dof():
            raise PybotException

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

    def ik(self, pose):
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

    def tool_wrench(self):
        """Calculate the wrench (force and moment/torque) generated by the tool."""
        pose = self.fk()

    def random_joints(self):
        joint_angles = []
        for limits in self.joint_angle_limits:
            joint_angles.append(np.random.uniform(min(limits), max(limits)))
        self.joint_angles = joint_angles


def _ik_fit_func(joint_angles, robot, pose):
    joint_angles = geometry.wrap_2_pi(joint_angles)
    robot.joint_angles = joint_angles
    actual_pose = robot.fk()

    error = actual_pose - pose
    error = error.flatten()
    return error
