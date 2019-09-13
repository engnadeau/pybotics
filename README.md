# ![](https://raw.githubusercontent.com/nnadeau/pybotics/master/media/robotic-arm.png) pybotics

The Python Toolbox for Robotics

|Component|Badges|
|---|---|
| GitHub | [![GitHub issues](https://img.shields.io/github/issues/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/issues) [![GitHub forks](https://img.shields.io/github/forks/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/network) [![GitHub stars](https://img.shields.io/github/stars/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/stargazers) [![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases) |
| PyPI | [![PyPI Version](https://img.shields.io/pypi/v/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI License](https://img.shields.io/pypi/l/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Wheel](https://img.shields.io/pypi/wheel/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Format](https://img.shields.io/pypi/format/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Pythons](https://img.shields.io/pypi/pyversions/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Implementation](https://img.shields.io/pypi/implementation/pybotics.svg)](https://pypi.python.org/pypi/pybotics) |
| CI | [![Build Status](https://travis-ci.org/nnadeau/pybotics.svg?branch=master)](https://travis-ci.org/nnadeau/pybotics) |
| Coverage | [![codecov](https://codecov.io/gh/nnadeau/pybotics/branch/master/graph/badge.svg)](https://codecov.io/gh/nnadeau/pybotics) |

## Overview

`Pybotics` is an open-source Python toolbox for robot kinematics and calibration.
It was designed to provide a simple, clear, and concise interface to quickly simulate and evaluate common robot concepts, such as kinematics, dynamics, trajectory generations, and calibration.
The toolbox is specifically designed for use with the [Modified Denavit–Hartenberg parameters convention](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters).

## Usage

### Documentation

- Please visit https://pybotics.readthedocs.io/

### Installation

```bash
# python3 is mapped to pip or inside a venv
pip install pybotics

# python3-pip
pip3 install pybotics

# https://github.com/pypa/pipenv
pipenv install pybotics

# https://github.com/sdispater/poetry
poetry add pybotics
```

### Applications & Examples

- [Basic Usage](examples/basic_usage.py)
- [Kinematics](examples/kinematics.ipynb)
- [Calibration](examples/calibration.ipynb)
- Trajectory and path planning
- Machine learning

## Featured In

- [Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling](https://www.mdpi.com/2218-6581/8/2/33/htm)
- [PyCon Canada 2017](https://2017.pycon.ca/schedule/53/)
  - [Talk Photos](https://500px.com/nicholasnadeau/galleries/pycon-canada-2017)
  - [Slides](https://github.com/nnadeau/pycon-canada-2017)
- [Montreal-Python 2017](https://www.youtube.com/watch?v=wgKoGA69YXQ)

## Citing

- Please cite the following articles if you use `pybotics` in your research:

> Nadeau, Nicholas A., Ilian A. Bonev, and Ahmed Joubair. "Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling." Robotics 8.2 (2019): 33.

```
@article{nadeau2019impedance,
  title={Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling},
  author={Nadeau, Nicholas A and Bonev, Ilian A and Joubair, Ahmed},
  journal={Robotics},
  volume={8},
  number={2},
  pages={33},
  year={2019},
  publisher={Multidisciplinary Digital Publishing Institute}
}
```

## Development

- Install dev virtual environment:

```bash
poetry install
```

- Make changes
- Submit a PR

---

<div>Logo made by <a href="http://www.freepik.com" title="Freepik">Freepik</a> from <a href="http://www.flaticon.com" title="Flaticon">www.flaticon.com</a> is licensed by <a href="http://creativecommons.org/licenses/by/3.0/" title="Creative Commons BY 3.0" target="_blank">CC 3.0 BY</a></div>
