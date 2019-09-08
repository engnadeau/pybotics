"""Predefined robot models."""
from copy import deepcopy
from typing import Any

import attr
import numpy as np  # type: ignore

from pybotics import Robot
from pybotics.kinematic_chain import MDHKinematicChain

KukaLBRiiwa7 = np.array(
    [
        [0, 0, 0, 340],
        [-np.pi / 2, 0, 0, 0],
        [np.pi / 2, 0, 0, 400],
        [np.pi / 2, 0, 0, 0],
        [-np.pi / 2, 0, 0, 400],
        [-np.pi / 2, 0, 0, 0],
        [np.pi / 2, 0, 0, 126],
    ]
)

MecademicMeca500 = np.array(
    [
        [0, 0, 0, 135],
        [-np.pi / 2, 0, -np.pi / 2, 0],
        [0, 135, 0, 0],
        [-np.pi / 2, 38, 0, 120],
        [np.pi / 2, 0, 0, 0],
        [-np.pi / 2, 0, np.pi, 72],
    ]
)

PUMA560 = np.array(
    [
        [0, 0, 0, 0],
        [-np.pi / 2, 0, 0, 0],
        [0, 612.7, 0, 0],
        [0, 571.6, 0, 163.9],
        [-np.pi / 2, 0, 0, 115.7],
        [np.pi / 2, 0, np.pi, 92.2],
    ]
)


UR10 = np.array(
    [
        [0, 0, 0, 118],
        [np.pi / 2, 0, np.pi, 0],
        [0, 612.7, 0, 0],
        [0, 571.6, 0, 163.9],
        [-np.pi / 2, 0, 0, 115.7],
        [np.pi / 2, 0, np.pi, 92.2],
    ]
)
