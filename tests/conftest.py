"""Pytest config."""
from pathlib import Path
from typing import Dict, List, Union

import numpy as np
import numpy.typing as npt
from pytest import fixture

from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.robot import Robot


@fixture()
def planar_robot() -> Robot:
    """Generate planar robot."""
    return Robot(
        MDHKinematicChain.from_parameters(
            np.array([[0, 0, 0, 0], [0, 10, 0, 0], [0, 20, 0, 0]])
        )
    )


@fixture()
def resources_path() -> Path:
    """Get resources path."""
    return (Path(__file__).parent / "resources").resolve()


@fixture()
def vector_transforms(
    resources_path: Path,
) -> List[Dict[str, Union[npt.NDArray[np.float64], str]]]:
    """Get resource data."""
    data = np.genfromtxt(
        fname=resources_path / "vector-transforms.csv", delimiter=",", dtype=str
    )  # type: ignore

    result = [
        {
            "vector": d[:6].astype(float),
            "transform": d[6:-1].astype(float),
            "order": d[-1],
        }
        for d in data
    ]
    return result
