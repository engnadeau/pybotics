import numpy as np


class Tool:
    def __init__(self,
                 tcp: np.ndarray = np.eye(4),
                 mass: float = 0,
                 cg: np.ndarray = np.eye(4)):
        self.tcp = tcp
        self.mass = mass
        self.cg = cg
