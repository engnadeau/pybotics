import math
import numpy as np


def forward_transform(rx, tx=None, tz=None, rz=None):
    """
    Returns the Denavit-Hartenberg Modified 4x4 matrix for a robot link (Craig 1986).
    Angular arguments are in radians.
    Calling forward_transform(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)

    :param rx:
    :param tx:
    :param tz:
    :param rz:
    :return:
    """

    # if argument is a vector
    if tx is None:
        [rx, tx, tz, rz] = rx

    # get angular elements
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
