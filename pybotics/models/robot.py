"""Robot module."""
from typing import Optional, List

import numpy as np
from pybotics import Tool
from pybotics.calibration.optimization_mask import OptimizationMask
from pybotics.kinematics.kinematic_chain import KinematicChain
from pybotics.models.frame import Frame
from pybotics.utilities.geometry import frame_2_euler_zyx
from pybotics.utilities.validation import is_4x4_ndarray


class Robot:
    def __init__(self, kinematic_chain: KinematicChain, tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        # public members
        self.kinematic_chain = kinematic_chain
        self.tool = Tool() if tool is None else tool
        self.world_frame = Frame() if world_frame is None else world_frame

        # private members
        self._position = np.zeros(self.num_dof())
        self._position_limits = np.repeat((-np.inf, np.inf), self.num_dof()).reshape((2, -1))

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = position

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = position

    def num_dof(self):
        return self.kinematic_chain.num_dof()

    def fk(self, position=None):
        position = self.position if position is None else position
        transforms = []
        transforms.append(self.world_frame.matrix)
        transforms.extend(self.kinematic_chain.transforms(position))
        transforms.append(self.tool.tcp)
