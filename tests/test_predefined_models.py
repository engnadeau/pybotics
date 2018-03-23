"""Test."""
import inspect

import numpy as np

from pybotics import predefined_models
from pybotics.predefined_models import UR10


def test_models():
    """Test."""
    # test specific models
    UR10(random_state=np.random.RandomState())

    # iterate through all predefined models and init them
    for _, o in inspect.getmembers(predefined_models, inspect.isclass):
        if o.__module__ == predefined_models.__name__:
            o()
