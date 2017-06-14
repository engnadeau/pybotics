"""Kinematic functions and utilities."""
import math
import numpy as np  # type: ignore


def forward_transform(mdh_parameters: np.ndarray) -> np.ndarray:
    """Return the Modified Denavit-Hartenberg (MDH) 4x4 matrix for a robot link (Craig 1986).

    Angular arguments are in radians.
    Calling forward_transform(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)

    :param mdh_parameters: list of MDH paramters: alpha [rad], a [mm], theta [rad], d [mm]
    :return: 4x4 transform
    """
    rx, tx, rz, tz = mdh_parameters

    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)

    transform = [
        [crz, -srz, 0, tx],
        [crx * srz, crx * crz, -srx, -tz * srx],
        [srx * srz, crz * srx, crx, tz * crx],
        [0, 0, 0, 1]
    ]

    return np.array(transform, dtype=np.float)
