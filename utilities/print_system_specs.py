#!/usr/bin/env python

import platform
import sys
import pybotics
import numpy
import scipy

print('Platform: {}'.format(platform.platform()))
print('Machine: {}'.format(platform.machine()))
print('Python: {}'.format(sys.version))
try:
    print('Pybotics: {}'.format(pybotics.__version__))
except AttributeError as e:
    print(e)
print('NumPy: {}'.format(numpy.__version__))
print('SciPy: {}'.format(scipy.__version__))
