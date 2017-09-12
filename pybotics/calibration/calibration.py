"""Calibration functions and utilities."""
from typing import List, Optional, Union

import numpy as np  # type: ignore

from pybotics.models.robot import Robot


def compute_absolute_errors(robot: Robot,
                            joint_angles: np.ndarray,
                            tcp_positions: np.ndarray,
                            joint_torques: Optional[np.ndarray] = None) -> np.ndarray:
    """Compute absolute forward kinematic position errors.

    All positions must be in the same world frame as the robot.
    Must have the same number of joints, torques, and positions.

    :param robot: Robot object with N joints
    :param joint_angles: MxN joint angles [rads]
    :param joint_torques: MxN joint torques [N-mm]
    :param tcp_positions: Mx3 expected tcp xyz positions [mm]
    :return: M absolute distance errors [mm]
    """
    # TODO: review and simplify code
    # validate arguments
    if joint_angles.shape[1] != robot.num_dof():
        raise ValueError('Required: joint_angles.shape[1] == {}'.format(robot.num_dof()))

    if joint_torques is not None:
        if joint_torques.shape[0] != joint_angles.shape[0]:
            raise ValueError('Required: joint_torques.shape[0] == {}'.format(joint_angles.shape[0]))

        if joint_torques.shape[1] != robot.num_dof():
            raise ValueError('Required: joint_torques.shape[1] == {}'.format(robot.num_dof()))

    if tcp_positions.shape[0] != joint_angles.shape[0]:
        raise ValueError('Required: tcp_positions.shape[0] == {}'.format(joint_angles.shape[0]))

    if tcp_positions.shape[1] != 3:
        raise ValueError('Required: tcp_positions.shape[1] == 3')

    errors = []
    for i, _ in enumerate(joint_angles):
        robot.joint_angles = joint_angles[i]
        robot.joint_torques = joint_torques[i]

        pose = robot.fk()
        pose_xyz = pose[0:-1, -1]

        position_error = pose_xyz - tcp_positions[i]
        error = np.linalg.norm(position_error)
        errors.append(error)

    errors = np.abs(errors)
    return errors


def generate_optimization_vector(robot: Robot,
                                 optimization_mask: List[bool]) -> np.ndarray:
    """Generate an optimization vector from a given mask.

    :param optimization_mask: sequence of booleans describing desired parameters
    :return: sequence of parameters
    """
    # TODO: review and simplify code
    parameters = list(itertools.chain(
        geometry.pose_2_xyzrpw(self.world_frame),
        self.robot_model.ravel(),
        geometry.pose_2_xyzrpw(self.tool.tcp),
        self.joint_compliances
    ))
    parameters = np.array(list(itertools.compress(parameters, optimization_mask)))

    return parameters


def generate_optimization_mask(robot: Robot,
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
        pass

    if isinstance(robot_model_mask, bool):
        robot_model_mask = [robot_model_mask] * self.robot_model.size
    elif len(robot_model_mask) != self.robot_model.size:
        pass

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


# def generate_parameter_bounds(robot: Robot,
#                               optimization_mask: List[bool],
#                               world_bounds: Union[RobotBound, None] = None,
#                               robot_model_bounds: Union[RobotBound, None] = None,
#                               tool_bounds: Union[RobotBound, None] = None,
#                               joint_compliance_bounds: Union[RobotBound, None] = None
#                               ) -> RobotBound:
#     """
#     Generate optimization bounds.
#
#     :param optimization_mask: optimization parameters boolean mask
#     :param world_bounds: world transform bounds
#     :param robot_model_bounds: MDH parameter bounds
#     :param tool_bounds: tool transform bounds
#     :param joint_compliance_bounds: joint compliance bounds
#     :return: optimization bounds
#     """
#     world_bounds = [(None, None)] * 6 if world_bounds is None else world_bounds
#     robot_model_bounds = [(
#         None, None)] * self.robot_model.size if robot_model_bounds is None else robot_model_bounds
#     tool_bounds = [(None, None)] * 6 if tool_bounds is None else tool_bounds
#     joint_compliance_bounds = [(
#         None, None)] * self.num_dof() if joint_compliance_bounds is None else joint_compliance_bounds
#
#     bounds = list(itertools.chain(
#         world_bounds,
#         robot_model_bounds,
#         tool_bounds,
#         joint_compliance_bounds
#     ))
#
#     bounds = list(itertools.compress(bounds, optimization_mask))
#
#     return bounds
