"""Calibration module."""
from typing import Sequence
import numpy as np  # type: ignore

from pybotics.constants import POSITION_VECTOR_LENGTH
from pybotics.robot import Robot


def compute_absolute_errors(robot: Robot,
                            link_positions: Sequence[Sequence[float]],
                            measured_positions: Sequence[Sequence[float]]
                            ) -> np.ndarray:
    """
    Compute the absolute errors of a given set of positions.

    :param robot: robot model
    :param link_positions: sequence of positions of each link (e.g., joints)
    :param measured_positions: sequence of XYZ positions measured
    :return:
    """
    # ensure ndarray
    link_positions = np.array(link_positions)
    measured_positions = np.array(measured_positions)

    actual_poses = np.array(list(map(robot.fk, link_positions)))
    actual_positions = actual_poses[:, :-1, -1]
    position_errors = measured_positions - actual_positions
    distance_errors = np.linalg.norm(position_errors, axis=1)

    return distance_errors
