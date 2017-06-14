"""Geometry functions and utilities."""
import math
from typing import Union

import numpy as np  # type: ignore

from pybotics import exceptions


def xyzrpw_2_pose(xyzrpw: np.ndarray) -> np.ndarray:
    """
    Calculate the pose from the position and euler angles ([x,y,z,r,p,w] vector).

    Equivalent to transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)

    :param xyzrpw:
    :return:
    """
    # validate input
    if len(xyzrpw) != 6:
        raise exceptions.PybotException

    # get individual variables
    [x, y, z, r, p, w] = xyzrpw

    # get trig values
    cr = math.cos(r)
    sr = math.sin(r)
    cp = math.cos(p)
    sp = math.sin(p)
    cw = math.cos(w)
    sw = math.sin(w)

    # get resulting transform
    transform = [
        [cp * cw, -cp * sw, sp, x],
        [cr * sw + cw * sr * sp, cr * cw - sr * sp * sw, -cp * sr, y],
        [sr * sw - cr * cw * sp, cw * sr + cr * sp * sw, cr * cp, z],
        [0, 0, 0, 1]
    ]

    return np.array(transform, dtype=np.float)


def pose_2_xyzrpw(pose: np.ndarray) -> np.ndarray:
    """
    Calculate the equivalent position and euler angles ([x,y,z,r,p,w] vector) of the given pose.

    Extract rotation from Rx * Ry * Rz order.
    Decomposes transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180).

    From: Craig, John J. Introduction to robotics: mechanics and control, 2005
    """
    x = pose[0, 3]
    y = pose[1, 3]
    z = pose[2, 3]

    # solution degenerates near p = +/- 90deg
    cos_p = math.sqrt(pose[1, 2] ** 2 + pose[2, 2] ** 2)
    sin_p = pose[0, 2]

    if np.isclose([cos_p], [0]):
        r = 0.0
        if sin_p > 0:
            p = math.pi / 2
            w = math.atan2(pose[1, 0], -pose[2, 0])
        else:
            p = -math.pi / 2
            w = math.atan2(pose[1, 0], pose[2, 0])
    else:
        sin_r = -pose[1, 2] / cos_p
        cos_r = pose[2, 2] / cos_p
        r = math.atan2(sin_r, cos_r)

        p = math.atan2(sin_p, cos_p)

        sin_w = -pose[0, 1] / cos_p
        cos_w = pose[0, 0] / cos_p
        w = math.atan2(sin_w, cos_w)

    return np.array([x, y, z, r, p, w])


def wrap_2_pi(angles: Union[np.ndarray, float]) -> Union[np.ndarray, float]:
    """
    Recursively wrap given angles to +/- PI.

    :param angles:
    :return:
    """
    if isinstance(angles, float):
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
    else:
        angles = np.array(list(map(wrap_2_pi, angles)))

    return angles  # type: ignore
