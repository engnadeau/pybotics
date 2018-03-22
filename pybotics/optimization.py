"""Optimization module."""
from copy import deepcopy
from typing import Sequence, Union

import numpy as np  # type: ignore

from pybotics import Robot
from pybotics.constants import TRANSFORM_VECTOR_LENGTH
from pybotics.errors import PyboticsError
from pybotics.geometry import matrix_2_vector, vector_2_matrix


class OptimizationHandler:
    """Handler for optimization tasks."""

    def __init__(self,
                 robot: Robot,
                 kinematic_chain_mask: Union[bool, Sequence[bool]] = False,
                 tool_mask: Union[bool, Sequence[bool]] = False,
                 world_mask: Union[bool, Sequence[bool]] = False,
                 ) -> None:
        """Init handler."""
        self.world_mask = self._validate_transform_mask(
            world_mask, 'world_mask', TRANSFORM_VECTOR_LENGTH)
        self.tool_mask = self._validate_transform_mask(
            tool_mask, 'tool_mask', TRANSFORM_VECTOR_LENGTH)

        self.robot = deepcopy(robot)
        self.kinematic_chain_mask = self._validate_transform_mask(
            kinematic_chain_mask, 'kinematic_chain_mask',
            robot.kinematic_chain.num_parameters)

    @staticmethod
    def _validate_transform_mask(mask: Union[bool, Sequence[bool]],
                                 name: str,
                                 required_length: int
                                 ) -> Sequence[bool]:
        """Validate mask arguments."""
        # validate input
        if isinstance(mask, bool):
            return [mask] * required_length
        elif len(mask) != required_length:
            raise PyboticsError('{} must be of length {}'.format(
                name, required_length))
        else:
            return mask

    def apply_optimization_vector(self, vector: np.ndarray) -> None:
        """Apply vector."""
        # get number of parameters
        num_kc_parameters = np.sum(self.kinematic_chain_mask)
        num_tool_parameters = np.sum(self.tool_mask)

        # extract vector segments
        segments = np.split(vector,
                            [num_kc_parameters,
                             num_kc_parameters + num_tool_parameters])
        kc_segment = segments[0]
        tool_segment = segments[1]
        world_segment = segments[2]

        # update vectors
        kc_vector = self.robot.kinematic_chain.vector
        kc_vector[self.kinematic_chain_mask] = kc_segment
        self.robot.kinematic_chain.vector = kc_vector

        tool_vector = self.robot.tool.vector
        tool_vector[self.tool_mask] = tool_segment
        self.robot.tool.vector = tool_vector

        world_vector = matrix_2_vector(self.robot.world_frame)
        world_vector[self.world_mask] = world_segment
        self.robot.world_frame = vector_2_matrix(world_vector)

    def generate_optimization_vector(self) -> np.ndarray:
        """Generate vector."""
        kc_vector = np.compress(self.kinematic_chain_mask,
                                self.robot.kinematic_chain.vector)
        tool_vector = np.compress(self.tool_mask,
                                  self.robot.tool.vector)
        world_vector = np.compress(self.world_mask,
                                   matrix_2_vector(self.robot.world_frame))
        return np.hstack((kc_vector, tool_vector, world_vector))


def optimize_accuracy(optimization_vector: np.ndarray,
                      handler: OptimizationHandler,
                      qs: Sequence[Sequence[float]],
                      positions: Sequence[Sequence[float]]
                      ) -> np.ndarray:
    """Fitness function for accuracy optimization."""
    handler.apply_optimization_vector(optimization_vector)
    errors = compute_absolute_errors(qs=qs,
                                     positions=positions,
                                     robot=handler.robot)
    return errors


def compute_absolute_errors(qs: Sequence[Sequence[float]],
                            positions: Sequence[Sequence[float]],
                            robot: Robot
                            ) -> Union[Sequence[float], np.ndarray]:
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

    if np.ndim(qs) == 1:
        qs = np.expand_dims(qs, axis=0)
    if np.ndim(positions) == 1:
        positions = np.expand_dims(positions, axis=0)

    # compute fk positions
    actual_poses = np.array(list(map(robot.fk, qs)))
    actual_positions = actual_poses[:, :-1, -1]

    # compute error
    position_errors = positions - actual_positions
    distance_errors = np.linalg.norm(position_errors, axis=1)

    return distance_errors
