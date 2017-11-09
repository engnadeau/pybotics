# ![](media/robotic-arm.png) pybotics
The Python Toolbox for Robotics

|Component|Badges|
|---|---|
|GitHub|[![GitHub issues](https://img.shields.io/github/issues/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/issues) [![GitHub forks](https://img.shields.io/github/forks/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/network) [![GitHub stars](https://img.shields.io/github/stars/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/stargazers) [![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases)|
|PyPI|[![PyPI Version](https://img.shields.io/pypi/v/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI License](https://img.shields.io/pypi/l/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Wheel](https://img.shields.io/pypi/wheel/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Format](https://img.shields.io/pypi/format/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Pythons](https://img.shields.io/pypi/pyversions/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Implementation](https://img.shields.io/pypi/implementation/pybotics.svg)](https://pypi.python.org/pypi/pybotics)|
|Documentation|[![Documentation Status](https://readthedocs.org/projects/pybotics/badge/?version=latest)](http://pybotics.readthedocs.io/en/latest/?badge=latest)|
|Referencing|[![DOI](https://zenodo.org/badge/66797360.svg)](https://zenodo.org/badge/latestdoi/66797360)|
|CI|[![Build Status](https://travis-ci.org/nnadeau/pybotics.svg?branch=master)](https://travis-ci.org/nnadeau/pybotics) [![CircleCI](https://circleci.com/gh/nnadeau/pybotics/tree/master.svg?style=svg)](https://circleci.com/gh/nnadeau/pybotics/tree/master)|
|Dependencies|[![Requirements Status](https://requires.io/github/nnadeau/pybotics/requirements.svg?branch=big-refactor)](https://requires.io/github/nnadeau/pybotics/requirements/?branch=big-refactor) [![Updates](https://pyup.io/repos/github/nnadeau/pybotics/shield.svg)](https://pyup.io/repos/github/nnadeau/pybotics/) [![Dependency Status](https://dependencyci.com/github/nnadeau/pybotics/badge)](https://dependencyci.com/github/nnadeau/pybotics)|
|Coverage|[![Coverage Status](https://coveralls.io/repos/github/nnadeau/pybotics/badge.svg?branch=master)](https://coveralls.io/github/nnadeau/pybotics?branch=master) [![codecov](https://codecov.io/gh/nnadeau/pybotics/branch/master/graph/badge.svg)](https://codecov.io/gh/nnadeau/pybotics)|
|Code Quality|[![Scrutinizer Code Quality](https://scrutinizer-ci.com/g/nnadeau/pybotics/badges/quality-score.png?b=master)](https://scrutinizer-ci.com/g/nnadeau/pybotics/?branch=master) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/9d4f77b167874a049e97731181e2b53a)](https://www.codacy.com/app/nicholas-nadeau/pybotics?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=nnadeau/pybotics&amp;utm_campaign=Badge_Grade) [![Code Climate](https://codeclimate.com/github/nnadeau/pybotics/badges/gpa.svg)](https://codeclimate.com/github/nnadeau/pybotics) [![Issue Count](https://codeclimate.com/github/nnadeau/pybotics/badges/issue_count.svg)](https://codeclimate.com/github/nnadeau/pybotics)|

## Usage
### Installation
```
pip install pybotics
```

### Applications & Examples
- [Kinematics](examples/forward_kinematics.ipynb)
- [Calibration](examples/calibration.ipynb)
- Trajectory and path planning

## Development
- All branches are deployed to [PyPI's Test Site](https://testpypi.python.org/pypi/pybotics/)
- Only tags on the `master` branch are deployed to [PyPI](https://pypi.org/project/pybotics/)
- Requirements used for CI (e.g., `requirements/static-testing.txt`) are pinned to a specific version to avoid the `master` branch from suddenly failing due to a package update.

## References
- Craig, John J. Introduction to robotics: mechanics and control. Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
- Corke, Peter. Robotics, vision and control: fundamental algorithms in MATLAB. Vol. 73. Springer, 2011.

---
<div>Logo made by <a href="http://www.freepik.com" title="Freepik">Freepik</a> from <a href="http://www.flaticon.com" title="Flaticon">www.flaticon.com</a> is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a></div>
