import math
import numpy as np


def xyzrpw_2_pose(xyzrpw):
    """
    Calculates the pose from the position and euler angles ([x,y,z,r,p,w] vector)
    The result is the same as calling: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    :param xyzrpw:
    :return:
    """

    # get individual variables
    [x, y, z, r, p, w] = xyzrpw

    # convert to rads
    a = np.deg2rad(r)
    b = np.deg2rad(p)
    c = np.deg2rad(w)

    # get trig values
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)

    # get resulting transform
    transform = [
        [cb * cc, -cb * sc, sb, x],
        [ca * sc + cc * sa * sb, ca * cc - sa * sb * sc, -cb * sa, y],
        [sa * sc - ca * cc * sb, cc * sa + ca * sb * sc, ca * cb, z],
        [0, 0, 0, 1]]

    return np.array(transform)
