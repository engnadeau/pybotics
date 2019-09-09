"""Pytest config."""
from pathlib import Path

import numpy as np
from pytest import fixture

from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.robot import Robot


@fixture()
def planar_robot():
    """Generate planar robot."""
    return Robot(
        MDHKinematicChain(np.array([[0, 0, 0, 0], [0, 10, 0, 0], [0, 20, 0, 0]]))
    )


@fixture()
def resources_path():
    """Get resources path."""
    return (Path(__file__).parent / "resources").resolve()


@fixture()
def vector_transforms(resources_path: Path):
    """Get resource data."""
    data = np.genfromtxt(
        fname=resources_path / "vector-transforms.csv", delimiter=",", dtype=str
    )
    return [
        {
            "vector": d[:6].astype(float),
            "transform": d[6:-1].astype(float),
            "order": d[-1],
        }
        for d in data
    ]
