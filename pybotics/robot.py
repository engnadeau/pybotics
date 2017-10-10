"""Robot module."""
from itertools import compress
from typing import Optional, Iterable

import numpy as np

from pybotics.frame import Frame
from pybotics.kinematic_chain import KinematicChain
from pybotics.link import Link
from pybotics.optimizable import Optimizable
from pybotics.tool import Tool


class Robot(KinematicChain, Optimizable):
    @property
    def optimization_vector(self) -> np.ndarray:
        world_vector = self.world_frame.optimization_vector
        tool_vector = self.tool.optimization_vector
        robot_vector = np.array(list(compress(self.vector, self.optimization_mask)))

        return np.hstack((world_vector, robot_vector, tool_vector))

    @optimization_vector.setter
    def optimization_vector(self, value: np.ndarray):
        pass

    @property
    def optimization_mask(self):
        return

    @optimization_mask.setter
    def optimization_mask(self, value):
        pass

    def __init__(self, links: Iterable[Link], tool: Optional[Tool] = None, world_frame: Optional[Frame] = None) -> None:
        super().__init__(links)
        # public members
        self.tool = Tool() if tool is None else tool
        self.world_frame = Frame() if world_frame is None else world_frame

        # private members
        self._position = np.zeros(self.num_dof())
        self._position_limits = np.repeat((-np.inf, np.inf), self.num_dof()).reshape((2, -1))

    @property
    def position(self) -> np.ndarray:
        return self._position

    @position.setter
    def position(self, value: np.ndarray) -> None:
        self._position = value

    @property
    def position_limits(self) -> np.ndarray:
        return self._position_limits

    @position_limits.setter
    def position_limits(self, value: np.ndarray) -> None:
        self._position_limits = value

    def fk(self, position: np.ndarray = None) -> np.ndarray:
        position = self.position if position is None else position

        transforms = [self.world_frame.matrix]
        transforms.extend(self.transforms(position))
        transforms.append(self.tool.matrix)

        pose = np.eye(4)
        for t in transforms:
            pose = np.dot(pose, t)

        return pose
