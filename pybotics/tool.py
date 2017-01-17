import numpy as np


class Tool:
    def __init__(self, tcp=np.eye(4), mass=0, cg=np.eye(4)):
        self.tcp = tcp
        self.mass = mass
        self.cg = cg
