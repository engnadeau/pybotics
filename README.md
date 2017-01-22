[![PyPI Version](https://img.shields.io/pypi/v/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases)
[![DOI](https://zenodo.org/badge/66797360.svg)](https://zenodo.org/badge/latestdoi/66797360)
[![PyPI License](https://img.shields.io/pypi/l/pybotics.svg)](https://pypi.python.org/pypi/pybotics)

[![PyPI Downloads](https://img.shields.io/pypi/dm/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Wheel](https://img.shields.io/pypi/wheel/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Format](https://img.shields.io/pypi/format/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Pythons](https://img.shields.io/pypi/pyversions/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Implementation](https://img.shields.io/pypi/implementation/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Status](https://img.shields.io/pypi/status/pybotics.svg)](https://pypi.python.org/pypi/pybotics)

[![Build Status](https://travis-ci.org/nnadeau/pybotics.svg?branch=master)](https://travis-ci.org/nnadeau/pybotics)
[![CircleCI](https://circleci.com/gh/nnadeau/pybotics/tree/master.svg?style=svg)](https://circleci.com/gh/nnadeau/pybotics/tree/master)

[![Dependency Status](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58/badge.svg?style=flat-square)](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58)
[![Coverage Status](https://coveralls.io/repos/github/nnadeau/pybotics/badge.svg?branch=master)](https://coveralls.io/github/nnadeau/pybotics?branch=master)
[![codecov](https://codecov.io/gh/nnadeau/pybotics/branch/master/graph/badge.svg)](https://codecov.io/gh/nnadeau/pybotics)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/9d4f77b167874a049e97731181e2b53a)](https://www.codacy.com/app/nicholas-nadeau/pybotics?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nnadeau/pybotics&amp;utm_campaign=Badge_Grade)
[![Code Climate](https://codeclimate.com/github/nnadeau/pybotics/badges/gpa.svg)](https://codeclimate.com/github/nnadeau/pybotics)
[![Issue Count](https://codeclimate.com/github/nnadeau/pybotics/badges/issue_count.svg)](https://codeclimate.com/github/nnadeau/pybotics)

# pybotics
Python Toolbox for Robotics

Inspired from [Peter Corke's Robotics Toolbox](http://www.petercorke.com/Robotics_Toolbox.html) for MATLAB.

## Requirements/Compatibility
- [Python >= 3.4](https://travis-ci.org/nnadeau/pybotics)
- See [requirements.txt](requirements.txt) for package dependencies

## Usage
### Quick Start
```python
import numpy as np
import pybotics as pybot

# classic planar robot from textbooks
robot_model = np.array([
    [0, 0, 0, 0],
    [0, 10, 0, 0],
    [0, 20, 0, 0]
], dtype=np.float)
planar_robot = pybot.Robot(robot_model)
pose = planar_robot.fk()

# modern, collaborative, 6-axis robot (UR10 from Universal Robots)
robot_model = np.loadtxt('ur10-mdh.csv', delimiter=',')
ur10_robot = pybot.Robot(robot_model)
ur10_robot.random_joints()
ur10_robot.fk()
```

### Applications
- [Kinematics](https://github.com/nnadeau/pybotics/blob/master/examples/example_kinematics.ipynb)
- [Calibration](https://github.com/nnadeau/pybotics/blob/master/examples/example_calibration.ipynb)
- Trajectory and path planning

## Limitations
- Currently only supports [Modified Denavit–Hartenberg Parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters)

## References
- Craig, John J. Introduction to robotics: mechanics and control. Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
- Corke, Peter. Robotics, vision and control: fundamental algorithms in MATLAB. Vol. 73. Springer, 2011.
