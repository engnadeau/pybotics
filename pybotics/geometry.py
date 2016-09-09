import math
import numpy as np


def xyzrpw_2_pose(xyzrpw, is_radians=False):
    """
    Calculates the pose from the position and euler angles ([x,y,z,r,p,w] vector)
    The result is the same as calling: H = transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)
    :param xyzrpw:
    :return:
    """

    # get individual variables
    [x, y, z, r, p, w] = xyzrpw

    # convert to rads
    if not is_radians:
        r = np.deg2rad(r)
        p = np.deg2rad(p)
        w = np.deg2rad(w)

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
        [0, 0, 0, 1]]

    return np.array(transform)
