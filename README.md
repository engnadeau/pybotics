[![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases)
[![Build Status](https://travis-ci.org/nnadeau/pybotics.svg?branch=master)](https://travis-ci.org/nnadeau/pybotics)
[![Dependency Status](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58/badge.svg?style=flat-square)](https://www.versioneye.com/user/projects/57d87a4a7129660045cf3a58)
[![Coverage Status](https://coveralls.io/repos/github/nnadeau/pybotics/badge.svg?branch=master)](https://coveralls.io/github/nnadeau/pybotics?branch=master)
[![codecov](https://codecov.io/gh/nnadeau/pybotics/branch/master/graph/badge.svg)](https://codecov.io/gh/nnadeau/pybotics)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/9d4f77b167874a049e97731181e2b53a)](https://www.codacy.com/app/nicholas-nadeau/pybotics?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nnadeau/pybotics&amp;utm_campaign=Badge_Grade)
[![Code Climate](https://codeclimate.com/github/nnadeau/pybotics/badges/gpa.svg)](https://codeclimate.com/github/nnadeau/pybotics)
[![Issue Count](https://codeclimate.com/github/nnadeau/pybotics/badges/issue_count.svg)](https://codeclimate.com/github/nnadeau/pybotics)

# pybotics
Python Toolbox for Robotics

## Context
Inspired from [Peter Corke's Robotics Toolbox](http://www.petercorke.com/Robotics_Toolbox.html) for MATLAB. However, MATLAB is not necessarily widespread outside of academia (and I prefer Python), thus Pybotics was born.

## Requirements/Compatibility
- Python 2.7, 3.4, 3.5
- See [requirements.txt](requirements.txt) for package dependencies

## Applications and Usage
- [Kinematics](examples/example_kinematics.ipynb)
- [Calibration](examples/example_calibration.ipynb)
- Trajectory and path planning

## Contributing
1. Fork it!
2. Create your feature branch
3. Follow PEP 8 style guide
1. Don't break the current codebase (without good reason)
1. Have tests for all your code
5. Submit a pull request :D

## Limitations
- Currently only supports [Modified Denavit–Hartenberg Parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters)

## References
- Craig, John J. Introduction to robotics: mechanics and control. Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
- Corke, Peter. Robotics, vision and control: fundamental algorithms in MATLAB. Vol. 73. Springer, 2011.
