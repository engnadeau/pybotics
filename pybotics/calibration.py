import numpy as np
from pybotics.exceptions import PybotException
from pybotics.robot import Robot
from typing import Iterable, Sequence


def compute_absolute_errors(robot: Robot,
                            joints: Sequence[Iterable[float]],
                            torques: Sequence[Iterable[float]],
                            positions: Sequence[Iterable[float]]) -> np.ndarray:
    """Compute absolute forward kinematic position errors.

    All positions must be in the same world frame as the robot.
    Must have the same number of joints, torques, and positions.

    :param robot: Robot object
    :param joints: iterable of joint angles [rads]
    :param torques: iterable of joint torques [N-mm]
    :param positions: iterable of the expected xyz positions [mm]
    :return: absolute distance errors [mm]
    """
    if len(joints) != len(torques) or len(joints) != len(positions):
        raise PybotException

    errors = []
    for i, _ in enumerate(joints):
        robot.joint_angles = joints[i]
        robot.joint_torques = torques[i]

        pose = robot.fk()
        pose_xyz = pose[0:-1, -1]

        position_error = pose_xyz - positions[i]
        error = np.linalg.norm(position_error)
        errors.append(error)

    errors = np.abs(errors)
    return errors
