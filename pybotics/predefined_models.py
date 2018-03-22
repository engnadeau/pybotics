"""Predefined robot models."""
from typing import Optional

import numpy as np  # type: ignore

from pybotics import Robot, Tool
from pybotics.kinematic_chain import MDHKinematicChain


class KukaLBRiiwa7(Robot):
    """KUKA LBR iiwa 7 R800 collaborative robot."""

    # TODO: add manufacturer's joint limits
    kinematic_chain = MDHKinematicChain(
        np.array([
            0, 0, 0, 340,
            -np.pi / 2, 0, 0, 0,
            np.pi / 2, 0, 0, 400,
            np.pi / 2, 0, 0, 0,
            -np.pi / 2, 0, 0, 400,
            -np.pi / 2, 0, 0, 0,
            np.pi / 2, 0, 0, 126
        ])
    )

    def __init__(self,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None
                 ) -> None:
        """Init robot."""
        super().__init__(self.kinematic_chain, tool, world_frame)


class MecademicMeca500(Robot):
    """Mecademic Meca500 small robot."""

    # TODO: add manufacturer's joint limits
    kinematic_chain = MDHKinematicChain(
        np.array([
            0, 0, 0, 135,
            -np.pi / 2, 0, -np.pi / 2, 0,
            0, 135, 0, 0,
            -np.pi / 2, 38, 0, 120,
            np.pi / 2, 0, 0, 0,
            -np.pi / 2, 0, np.pi, 72
        ])
    )

    def __init__(self,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None
                 ) -> None:
        """Init robot."""
        super().__init__(self.kinematic_chain, tool, world_frame)


class PUMA560(Robot):
    """PUMA 560 robot."""

    # TODO: add manufacturer's joint limits
    kinematic_chain = MDHKinematicChain(
        np.array([
            0, 0, 0, 0,
            -np.pi / 2, 0, 0, 0,
            0, 612.7, 0, 0,
            0, 571.6, 0, 163.9,
            -np.pi / 2, 0, 0, 115.7,
            np.pi / 2, 0, np.pi, 92.2
        ])
    )

    def __init__(self,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None
                 ) -> None:
        """Init robot."""
        super().__init__(self.kinematic_chain, tool, world_frame)


class UR10(Robot):
    """Universal Robots UR10 collaborative robot."""

    # TODO: add manufacturer's joint limits
    kinematic_chain = MDHKinematicChain(
        np.array([
            0, 0, 0, 118,
            np.pi / 2, 0, np.pi, 0,
            0, 612.7, 0, 0,
            0, 571.6, 0, 163.9,
            -np.pi / 2, 0, 0, 115.7,
            np.pi / 2, 0, np.pi, 92.2
        ])
    )

    def __init__(self,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None
                 ) -> None:
        """Init robot."""
        super().__init__(self.kinematic_chain, tool, world_frame)
