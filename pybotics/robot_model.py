import numpy as np

'''
Modified DH parameter matrices of various robots (rx, tx, rz, tz) aka (alpha, a, theta, d)
'''


def ur10():
    model = np.array([
        [0, 0, 0, 118.0],
        [np.pi / 2.0, 0, np.pi, 0],
        [0, 612.7, 0, 0],
        [0, 571.6, 0, 163.9],
        [-np.pi / 2.0, 0, 0, 115.7],
        [np.pi / 2.0, 0, np.pi, 92.2]
    ])
    return model


def puma_560():
    model = np.array([
        [0, 0, 0, 0],
        [-np.pi / 2.0, 0, 0, 0],
        [0, 612.7, 0, 0],
        [0, 571.6, 0, 163.9],
        [-np.pi / 2.0, 0, 0, 115.7],
        [np.pi / 2.0, 0, np.pi, 92.2]
    ])
    return model


def kuka_lbr_iiwa_7():
    model = np.array([
        [0, 0, 0, 340],
        [-np.pi / 2.0, 0, 0, 0],
        [np.pi / 2.0, 0, 0, 400],
        [np.pi / 2.0, 0, 0, 0],
        [-np.pi / 2.0, 0, 0, 400],
        [-np.pi / 2.0, 0, 0, 0],
        [np.pi / 2.0, 0, 0, 126.0]
    ])
    return model
