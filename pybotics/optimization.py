"""Optimization module."""
from copy import deepcopy
from itertools import repeat
from typing import MutableSequence, Sequence

import attr
import numpy as np
import numpy.typing as npt

from pybotics.errors import PyboticsError
from pybotics.geometry import matrix_2_vector, position_from_matrix, vector_2_matrix
from pybotics.robot import Robot


def _validate_transform_mask(
    mask: MutableSequence[bool], name: str, size: int
) -> MutableSequence[bool]:
    """Validate mask arguments."""
    # validate input
    if isinstance(mask, bool):
        return [mask] * size
    elif len(mask) != size:
        raise PyboticsError(f"{name} must be of length {size}")
    else:
        return mask


@attr.s
class OptimizationHandler:
    """Handler for optimization tasks."""

    robot = attr.ib(type=Robot)
    kinematic_chain_mask = attr.ib(False, type=MutableSequence[bool])
    tool_mask = attr.ib(False, type=MutableSequence[bool])
    world_mask = attr.ib(False, type=MutableSequence[bool])

    def __attrs_post_init__(self) -> None:
        """Post-init handler."""
        self.world_mask = _validate_transform_mask(
            mask=self.world_mask, name="world_mask", size=6
        )
        self.tool_mask = _validate_transform_mask(
            mask=self.tool_mask, name="tool_mask", size=6
        )
        self.robot = deepcopy(self.robot)
        self.kinematic_chain_mask = _validate_transform_mask(
            mask=self.kinematic_chain_mask,
            name="kinematic_chain_mask",
            size=self.robot.kinematic_chain.num_parameters,
        )

    def apply_optimization_vector(self, vector: npt.NDArray[np.float64]) -> None:
        """Apply vector."""
        # get number of parameters
        num_kc_parameters = np.sum(self.kinematic_chain_mask)
        num_tool_parameters = np.sum(self.tool_mask)

        # extract vector segments
        segments = np.split(  # type: ignore
            vector, [num_kc_parameters, num_kc_parameters + num_tool_parameters]
        )
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

    def generate_optimization_vector(self) -> npt.NDArray[np.float64]:
        """Generate vector."""
        kc_vector = np.compress(
            self.kinematic_chain_mask, self.robot.kinematic_chain.vector
        )
        tool_vector = np.compress(self.tool_mask, self.robot.tool.vector)
        world_vector = np.compress(
            self.world_mask, matrix_2_vector(self.robot.world_frame)
        )
        return np.hstack((kc_vector, tool_vector, world_vector))


def optimize_accuracy(
    optimization_vector: npt.NDArray[np.float64],
    handler: OptimizationHandler,
    qs: Sequence[npt.NDArray[np.float64]],
    positions: Sequence[npt.NDArray[np.float64]],
) -> npt.NDArray[np.float64]:
    """Fitness function for accuracy optimization."""
    handler.apply_optimization_vector(optimization_vector)
    errors = compute_absolute_errors(qs=qs, positions=positions, robot=handler.robot)
    return errors


def compute_absolute_error(
    q: npt.NDArray[np.float64], position: npt.NDArray[np.float64], robot: Robot
) -> float:
    """Compute the absolute error of a given position."""
    pose = robot.fk(q)
    actual_position = position_from_matrix(pose)
    error = position - actual_position  # type: npt.NDArray[np.float64]
    result = float(np.linalg.norm(error))  # type: ignore
    return result


def compute_absolute_errors(
    qs: Sequence[npt.NDArray[np.float64]],
    positions: Sequence[npt.NDArray[np.float64]],
    robot: Robot,
) -> npt.NDArray[np.float64]:
    """
    Compute the absolute errors of a given set of positions.

    :param qs: Array of joints, shape=(n-poses, n-dof) [rad]
    :param positions: Array of Cartesian positions, shape=(n-poses, 3)
    :param robot: Robot model
    """
    return np.fromiter(  # type: ignore
        map(compute_absolute_error, qs, positions, repeat(robot)), dtype=np.float64
    )


def compute_relative_error(
    q_a: npt.NDArray[np.float64],
    q_b: npt.NDArray[np.float64],
    distance: float,
    robot: Robot,
) -> float:
    """Compute the relative error of a given position combination."""
    pose_a = robot.fk(q_a)
    pose_b = robot.fk(q_b)

    actual_position_a = position_from_matrix(pose_a)
    actual_position_b = position_from_matrix(pose_b)

    actual_distance = actual_position_a - actual_position_b  # type: float
    actual_distance = np.linalg.norm(actual_distance)  # type: ignore

    error = float(np.linalg.norm(distance - actual_distance))  # type: ignore

    return error


def compute_relative_errors(
    qs_a: npt.NDArray[np.float64],
    qs_b: npt.NDArray[np.float64],
    distances: npt.NDArray[np.float64],
    robot: Robot,
) -> npt.NDArray[np.float64]:
    """Compute the relative errors of a given set of position combinations."""
    return np.fromiter(  # type: ignore
        map(compute_relative_error, qs_a, qs_b, distances, repeat(robot)),
        dtype=np.float64,
    )
