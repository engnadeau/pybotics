[![PyPI Version](https://img.shields.io/pypi/v/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![Documentation Status](https://readthedocs.org/projects/pybotics/badge/?version=latest)](http://pybotics.readthedocs.io/en/latest/?badge=latest)
[![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases)
[![DOI](https://zenodo.org/badge/66797360.svg)](https://zenodo.org/badge/latestdoi/66797360)
[![PyPI License](https://img.shields.io/pypi/l/pybotics.svg)](https://pypi.python.org/pypi/pybotics)

[![PyPI Wheel](https://img.shields.io/pypi/wheel/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Format](https://img.shields.io/pypi/format/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Pythons](https://img.shields.io/pypi/pyversions/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![Python 3](https://pyup.io/repos/github/nnadeau/pybotics/python-3-shield.svg)](https://pyup.io/repos/github/nnadeau/pybotics/)
[![PyPI Implementation](https://img.shields.io/pypi/implementation/pybotics.svg)](https://pypi.python.org/pypi/pybotics)
[![PyPI Status](https://img.shields.io/pypi/status/pybotics.svg)](https://pypi.python.org/pypi/pybotics)

[![Build Status](https://travis-ci.org/nnadeau/pybotics.svg?branch=master)](https://travis-ci.org/nnadeau/pybotics)
[![CircleCI](https://circleci.com/gh/nnadeau/pybotics/tree/master.svg?style=svg)](https://circleci.com/gh/nnadeau/pybotics/tree/master)

[![Dependency Status](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58/badge.svg?style=flat-square)](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58)
[![Updates](https://pyup.io/repos/github/nnadeau/pybotics/shield.svg)](https://pyup.io/repos/github/nnadeau/pybotics/)
[![Dependency Status](https://dependencyci.com/github/nnadeau/pybotics/badge)](https://dependencyci.com/github/nnadeau/pybotics)

[![Coverage Status](https://coveralls.io/repos/github/nnadeau/pybotics/badge.svg?branch=master)](https://coveralls.io/github/nnadeau/pybotics?branch=master)
[![codecov](https://codecov.io/gh/nnadeau/pybotics/branch/master/graph/badge.svg)](https://codecov.io/gh/nnadeau/pybotics)

[![Scrutinizer Code Quality](https://scrutinizer-ci.com/g/nnadeau/pybotics/badges/quality-score.png?b=master)](https://scrutinizer-ci.com/g/nnadeau/pybotics/?branch=master)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/9d4f77b167874a049e97731181e2b53a)](https://www.codacy.com/app/nicholas-nadeau/pybotics?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nnadeau/pybotics&amp;utm_campaign=Badge_Grade)
[![Code Climate](https://codeclimate.com/github/nnadeau/pybotics/badges/gpa.svg)](https://codeclimate.com/github/nnadeau/pybotics)
[![Issue Count](https://codeclimate.com/github/nnadeau/pybotics/badges/issue_count.svg)](https://codeclimate.com/github/nnadeau/pybotics)

# pybotics ![](https://github.com/nnadeau/pybotics/blob/master/logo/robotic-arm.png)
Python Toolbox for Robotics

## Usage
### Installation
```
pip install pybotics
```

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
planar_robot.joint_angles = np.deg2rad([30, 60, 0])
pose = planar_robot.fk() # forward kinematics, returns 4x4 pose transform    

# modern, collaborative, 6-axis robot (UR10 from Universal Robots)
robot_model = np.loadtxt('ur10-mdh.csv', delimiter=',')
ur10_robot = pybot.Robot(robot_model)
ur10_robot.random_joints()
pose = ur10_robot.fk() # forward kinematics, returns 4x4 pose transform
```

### Applications
- [Kinematics](https://github.com/nnadeau/pybotics/blob/master/examples/example_kinematics.ipynb)
- [Calibration](https://github.com/nnadeau/pybotics/blob/master/examples/example_calibration.ipynb)
- Trajectory and path planning

## Development
- All branches are deployed to [PyPI's Test Site](https://testpypi.python.org/pypi/pybotics/)
- Only tags on the `master` branch are deployed to [PyPI](https://pypi.python.org/pypi/pybotics)

## References
- Craig, John J. Introduction to robotics: mechanics and control. Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
- Corke, Peter. Robotics, vision and control: fundamental algorithms in MATLAB. Vol. 73. Springer, 2011.

---
<div>Logo made by <a href="http://www.freepik.com" title="Freepik">Freepik</a> from <a href="http://www.flaticon.com" title="Flaticon">www.flaticon.com</a> is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a></div>
