"""Geometry functions and utilities."""
import math
from typing import Union

import numpy as np  # type: ignore


def euler_zyx_2_frame(vector: np.ndarray) -> np.ndarray:
    """
    Calculate the pose from the position and euler angles ([x,y,z,rx,ry,rz] vector).

    Equivalent to transl(x,y,z)*rotz(rz)*roty(ry)*rotx(rx)

    :param vector:
    :return:
    """
    # get individual variables
    [x, y, z, rx, ry, rz] = vector

    # get trig values
    cx = np.cos(rx)
    sx = np.sin(rx)

    cy = np.cos(ry)
    sy = np.sin(ry)

    cz = np.cos(rz)
    sz = np.sin(rz)

    # get resulting transform
    transform = [
        [cy * cz, -cy * sz, sy, x],
        [cx * sz + cz * sx * sy, cx * cz - sx * sy * sz, -cy * sx, y],
        [sx * sz - cx * cz * sy, cz * sx + cx * sy * sz, cx * cy, z],
        [0, 0, 0, 1]
    ]

    return np.array(transform, dtype=np.float)


def frame_2_euler_zyx(pose: np.ndarray) -> np.ndarray:
    """
    Calculate the equivalent position and euler angles ([x,y,z,rx,ry,rz] vector) of the given pose.

    Decomposes transl(x,y,z)*rotz(rz)*roty(ry)*rotx(rx).

    From: Craig, John J. Introduction to robotics: mechanics and control, 2005
    """
    validate_4x4_matrix(pose)

    x = pose[0, 3]
    y = pose[1, 3]
    z = pose[2, 3]

    # solution degenerates near ry = +/- 90deg
    cos_ry = np.sqrt(pose[1, 2] ** 2 + pose[2, 2] ** 2)
    sin_ry = pose[0, 2]

    if np.isclose([cos_ry], [0]):
        rx = 0.0
        if sin_ry > 0:
            ry = np.pi / 2
            rz = np.arctan2(pose[1, 0], -pose[2, 0])
        else:
            ry = -math.pi / 2
            rz = math.atan2(pose[1, 0], pose[2, 0])
    else:
        sin_rx = -pose[1, 2] / cos_ry
        cos_rx = pose[2, 2] / cos_ry
        rx = math.atan2(sin_rx, cos_rx)

        ry = math.atan2(sin_ry, cos_ry)

        sin_rz = -pose[0, 1] / cos_ry
        cos_rz = pose[0, 0] / cos_ry
        rz = math.atan2(sin_rz, cos_rz)

    return np.array([x, y, z, rx, ry, rz])


def wrap_2_pi(angles: Union[np.ndarray, float]) -> Union[np.ndarray, float]:
    """
    Recursively wrap given angles to +/- PI.

    :param angles:
    :return:
    """
    return (angles + np.pi) % (2 * np.pi) - np.pi
