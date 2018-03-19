"""Optimization module."""
from typing import Sequence, NamedTuple, Union, Iterable

import numpy as np  # type: ignore

from pybotics import Robot
from pybotics.constants import TRANSFORM_VECTOR_LENGTH
from pybotics.errors import PyboticsError


class Optimizer:
    def __init__(self,
                 robot: Robot,
                 robot_mask: Union[bool, Sequence[bool]] = True,
                 tool_mask: Union[bool, Sequence[bool]] = True,
                 world_mask: Union[bool, Sequence[bool]] = True
                 ):
        self.world_mask = self._validate_transform_mask(
            world_mask, 'world_mask', TRANSFORM_VECTOR_LENGTH)
        self.tool_mask = self._validate_transform_mask(
            tool_mask, 'tool_mask', TRANSFORM_VECTOR_LENGTH)

        self.robot = robot
        self.robot_mask = self._validate_transform_mask(
            robot_mask, 'robot_mask', robot.num_parameters)

    def _validate_transform_mask(
            self,
            mask: Union[bool, Sequence[bool]],
            name: str,
            required_length: int) -> Sequence[bool]:

        # validate input
        if isinstance(mask, bool):
            return [mask] * TRANSFORM_VECTOR_LENGTH
        elif len(mask) != TRANSFORM_VECTOR_LENGTH:
            raise PyboticsError(
                '{} must be of length {}'.format(
                    name, required_length))
        else:
            return mask

    def update_robot(self):
        pass


def compute_absolute_errors(qs: Sequence[Sequence[float]],
                            positions: Sequence[Sequence[float]],
                            robot: Robot
                            ) -> np.ndarray:
    """
    Compute the absolute errors of a given set of positions.

    :param robot: robot model
    :param qs: sequence of link positions (e.g., joints)
    :param positions: sequence of actual XYZ positions
    :return:
    """
    # ensure array of arrays
    qs = np.array(qs)
    positions = np.array(positions)

    if qs.ndim == 1:
        qs = np.expand_dims(qs, axis=0)
    if positions.ndim == 1:
        positions = np.expand_dims(positions, axis=0)

    # compute fk positions
    actual_poses = np.array(list(map(robot.fk, qs)))
    actual_positions = actual_poses[:, :-1, -1]

    # compute error
    position_errors = positions - actual_positions
    distance_errors = np.linalg.norm(position_errors, axis=1)

    return distance_errors
