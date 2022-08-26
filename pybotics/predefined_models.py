"""Predefined robot models.

These models correspond to the Modified Denavit–Hartenberg parameters:
https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
"""
import numpy as np
import numpy.typing as npt


def kuka_lbr_iiwa_7() -> npt.NDArray[np.float64]:  # pragma: no cover
    """Get KUKA LBR iiwa 7 MDH model."""
    return np.array(
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


def mecademic_meca500() -> npt.NDArray[np.float64]:  # pragma: no cover
    """Get Meca500 MDH model."""
    return np.array(
        [
            [0, 0, 0, 135],
            [-np.pi / 2, 0, -np.pi / 2, 0],
            [0, 135, 0, 0],
            [-np.pi / 2, 38, 0, 120],
            [np.pi / 2, 0, 0, 0],
            [-np.pi / 2, 0, np.pi, 72],
        ]
    )


def puma560() -> npt.NDArray[np.float64]:  # pragma: no cover
    """Get PUMA560 MDH model."""
    return np.array(
        [
            [0, 0, 0, 0],
            [-np.pi / 2, 0, 0, 0],
            [0, 612.7, 0, 0],
            [0, 571.6, 0, 163.9],
            [-np.pi / 2, 0, 0, 115.7],
            [np.pi / 2, 0, np.pi, 92.2],
        ]
    )


def ur10() -> npt.NDArray[np.float64]:  # pragma: no cover
    """Get UR10 MDH model."""
    return np.array(
        [
            [0, 0, 0, 118],
            [np.pi / 2, 0, np.pi, 0],
            [0, 612.7, 0, 0],
            [0, 571.6, 0, 163.9],
            [-np.pi / 2, 0, 0, 115.7],
            [np.pi / 2, 0, np.pi, 92.2],
        ]
    )


def abb_irb120() -> npt.NDArray[np.float64]:  # pragma: no cover
    """Get ABB irb120 MDH model."""
    return np.array(
        [
            [0, 0, 0, 290],
            [-np.pi / 2, 0, -np.pi / 2, 0],
            [0, 270, 0, 0],
            [-np.pi / 2, 70, 0, 302],
            [np.pi / 2, 0, 0, 0],
            [-np.pi / 2, 0, np.pi, 72],
        ]
    )
