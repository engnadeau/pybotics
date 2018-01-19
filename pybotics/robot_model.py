from typing import Optional

import numpy as np  # type: ignore

from pybotics.frame import Frame
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot
from pybotics.tool import Tool


class KukaLbrIiwa7(Robot):
    # TODO: add joint limits
    def __init__(self, tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        kinematic_chain = KinematicChain.from_array(
            np.array([
                0, 0, 0, 340,
                -np.pi / 2, 0, 0, 0,
                np.pi / 2, 0, 0, 400,
                np.pi / 2, 0, 0, 0,
                -np.pi / 2, 0, 0, 400,
                -np.pi / 2, 0, 0, 0,
                np.pi / 2, 0, 0, 126
            ]).reshape((-1, 4)))
        super().__init__(kinematic_chain, tool, world_frame)


class MecademicMeca500(Robot):
    def __init__(self, tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        kinematic_chain = KinematicChain.from_array(
            np.array([
                0, 0, 0, 135,
                -np.pi / 2, 0, -np.pi / 2, 0,
                0, 135, 0, 0,
                -np.pi / 2, 38, 0, 120,
                np.pi / 2, 0, 0, 0,
                -np.pi / 2, 0, np.pi, 72
            ]).reshape((-1, 4)))
        super().__init__(kinematic_chain, tool, world_frame)


class Puma560(Robot):
    def __init__(self, tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        kinematic_chain = KinematicChain.from_array(
            np.array([
                0, 0, 0, 0,
                -np.pi / 2, 0, 0, 0,
                0, 612.7, 0, 0,
                0, 571.6, 0, 163.9,
                -np.pi / 2, 0, 0, 115.7,
                np.pi / 2, 0, np.pi, 92.2
            ]).reshape((-1, 4)))
        super().__init__(kinematic_chain, tool, world_frame)


class UR10(Robot):
    def __init__(self, tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        kinematic_chain = KinematicChain.from_array(
            np.array([
                0, 0, 0, 118,
                np.pi / 2, 0, np.pi, 0,
                0, 612.7, 0, 0,
                0, 571.6, 0, 163.9,
                -np.pi / 2, 0, 0, 115.7,
                np.pi / 2, 0, np.pi, 92.2
            ]).reshape((-1, 4)))
        super().__init__(kinematic_chain, tool, world_frame)
