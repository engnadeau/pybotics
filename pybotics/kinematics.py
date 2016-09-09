import math
import numpy as np


def forward_transform(rx, tx=None, rz=None, tz=None, is_radians=True):
    """
    Returns the Denavit-Hartenberg Modified 4x4 matrix for a robot link (Craig 1986).
    Angular arguments are in radians.
    Calling forward_transform(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)

    :param rx: DHM alpha
    :param tx: DHM a
    :param rz: DHM theta
    :param tz: DHM d
    :return:
    """

    # if argument is a vector
    if tx is None:
        [rx, tx, rz, tz] = rx

    # get angular elements
    if not is_radians:
        rx = np.deg2rad(rx)
        rz = np.deg2rad(rz)

    crx = math.cos(rx)
    srx = math.sin(rx)
    crz = math.cos(rz)
    srz = math.sin(rz)

    # generate output matrix
    transform = [
        [crz, -srz, 0, tx],
        [crx * srz, crx * crz, -srx, -tz * srx],
        [srx * srz, crz * srx, crx, tz * crx],
        [0, 0, 0, 1]]

    return np.array(transform)
