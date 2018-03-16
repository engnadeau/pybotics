"""Calibration module."""
from typing import Sequence, NamedTuple, Union

import numpy as np  # type: ignore

from pybotics.robot import Robot





def apply_optimization_vector(self, vector: np.ndarray) -> None:
    """
    Update the current instance with new optimization parameters.

    :param vector: new parameters to apply
    """
    # we are going to iterate through the given vector;
    # an iterator allows us to next()
    # (aka `pop`) the values only when desired;
    # we only update the current vector where the mask is True
    vector_iterator = iter(vector)
    updated_vector = [v if not m else next(vector_iterator)
                      for v, m in zip(self.vector,
                                      self.optimization_mask)]
    updated_links = self.array_2_links(np.array(updated_vector),
                                       self.convention)
    self.links = updated_links


OptimizationMask = NamedTuple(
    'RobotOptimizationMask',
    [
        ('world_frame', Union[bool, Sequence[bool]]),
        ('kinematic_chain', Union[bool, Sequence[bool]]),
        ('tool', Union[bool, Sequence[bool]])
    ]
)


@property
def optimization_mask(self) -> OptimizationMask:
    """
    Return the mask used to select the optimization parameters.

    :return: mask
    """
    mask = OptimizationMask(
        world_frame=self.world_frame.optimization_mask,
        kinematic_chain=self.kinematic_chain.optimization_mask,
        tool=self.tool.optimization_mask)
    return mask


@optimization_mask.setter
def optimization_mask(self, value: OptimizationMask) -> None:
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


def apply_optimization_vector(self, vector: np.ndarray) -> None:
    # we are going to iterate through the given vector;
    # an iterator allows us to next()
    # (aka `pop`) the values only when desired;
    # we only update the current vector where the mask is True
    """
    Update the current instance with new optimization parameters.

    :param vector: new parameters to apply
    """
    vector_iterator = iter(vector)
    updated_vector = [v if not m else next(vector_iterator)
                      for v, m in zip(self.vector(),
                                      self.optimization_mask)]

    updated_matrix = euler_zyx_2_matrix(np.array(updated_vector))
    self.matrix = updated_matrix
