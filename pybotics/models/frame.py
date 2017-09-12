import numpy as np

from pybotics.utilities.geometry import frame_2_euler_zyx, euler_zyx_2_frame
from pybotics.utilities.validation import is_4x4_ndarray


class Frame:
    def __init__(self, matrix: np.ndarray = None) -> None:
        # TODO: add vector: Optional[np.ndarray] = None, vector_convention: VectorConvention = None

        if matrix is not None:
            if not is_4x4_ndarray(matrix):
                raise ValueError('matrix must be 4x4 ndarray')

        self.matrix = np.eye(4) if matrix is None else matrix

    @property
    def vector(self):
        return frame_2_euler_zyx(self.matrix)

    @vector.setter
    def vector(self, vector):
        self.matrix = euler_zyx_2_frame(vector)
