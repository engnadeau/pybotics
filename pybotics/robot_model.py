from enum import Enum

import numpy as np


class RobotModel:
    '''
    Modified DH parameter matrices of various robots (rx, tx, tz, rz)
    '''

    UR10 = np.array([
        [0.0, 0.0, 118.0, 0],
        [np.pi / 2.0, 0.0, 0.0, np.pi],
        [0.0, 612.7, 0.0, 0.0],
        [0.0, 571.6, 163.9, 0.0],
        [-np.pi / 2.0, 0.0, 115.7, 0.0],
        [np.pi / 2.0, 0.0, 92.2, np.pi]
    ])

    PUMA_560 = np.array([
        [0.0, 0.0, 0.0, 0],
        [-np.pi / 2.0, 0.0, 0.0, 0.0],
        [0.0, 612.7, 0.0, 0.0],
        [0.0, 571.6, 163.9, 0.0],
        [-np.pi / 2.0, 0.0, 115.7, 0.0],
        [np.pi / 2.0, 0.0, 92.2, np.pi]
    ])

    KUKA_LBR_IIWA_7 = np.array([
        [np.pi / 2.0, 0.0, 340.0, 0],
        [-np.pi / 2.0, 0.0, 0.0, 0],
        [-np.pi / 2.0, 400.0, 0.0, 0.0],
        [np.pi / 2.0, 0, 0, 0.0],
        [np.pi / 2.0, 0.0, 400.0, 0.0],
        [-np.pi / 2.0, 0.0, 0.0, 0.0],
        [0, 0.0, 126.0, 0.0]
    ])
