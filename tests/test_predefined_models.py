import inspect

from pybotics import predefined_models


def test_models():
    # iterate through all predefined models and init them
    for _, o in inspect.getmembers(predefined_models, inspect.isclass):
        if o.__module__ == predefined_models.__name__:
            o()
