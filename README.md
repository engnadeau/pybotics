# Pybotics

![Robot arm logo](media/robotic-arm.png)

The Python Toolbox for Robotics

- [Explore the docs](https://pybotics.readthedocs.io)
- [View demos and examples](https://github.com/nnadeau/pybotics/tree/master/examples)
- [Report a bug](https://github.com/nnadeau/pybotics/issues)
- [Request a feature](https://github.com/nnadeau/pybotics/issues)

| Item          | Badges                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Repo          | [![GitHub issues](https://img.shields.io/github/issues/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/issues) [![GitHub forks](https://img.shields.io/github/forks/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/network) [![GitHub stars](https://img.shields.io/github/stars/nnadeau/pybotics.svg)](https://github.com/nnadeau/pybotics/stargazers) ![GitHub repo size](https://img.shields.io/github/repo-size/engnadeau/pybotics)                                                                                                                                                                                                                                                 |
| Releases      | [![GitHub tag](https://img.shields.io/github/tag/nnadeau/pybotics.svg?maxAge=2592000?style=flat-square)](https://github.com/nnadeau/pybotics/releases) [![PyPI Version](https://img.shields.io/pypi/v/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Wheel](https://img.shields.io/pypi/wheel/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Format](https://img.shields.io/pypi/format/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![semantic-release](https://img.shields.io/badge/%20%20%F0%9F%93%A6%F0%9F%9A%80-semantic--release-e10079.svg)](https://github.com/semantic-release/semantic-release) [![semantic-release: angular](https://img.shields.io/badge/semantic--release-angular-e10079?logo=semantic-release)](https://github.com/semantic-release/semantic-release) ![PyPI - Downloads](https://img.shields.io/pypi/dm/pybotics) |
| Compatibility | [![PyPI Pythons](https://img.shields.io/pypi/pyversions/pybotics.svg)](https://pypi.python.org/pypi/pybotics) [![PyPI Implementation](https://img.shields.io/pypi/implementation/pybotics.svg)](https://pypi.python.org/pypi/pybotics)                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| Workflows     | [![Test](https://github.com/nnadeau/pybotics/workflows/Test/badge.svg)](https://github.com/nnadeau/pybotics/actions) [![Release](https://github.com/nnadeau/pybotics/workflows/Release/badge.svg)](https://github.com/nnadeau/pybotics/actions) [![Publish](https://github.com/nnadeau/pybotics/workflows/Publish/badge.svg)](https://github.com/nnadeau/pybotics/actions)                                                                                                                                                                                                                                                                                                                                     |
| Documentation | [![Documentation Status](https://readthedocs.org/projects/pybotics/badge/?version=latest)](https://pybotics.readthedocs.io/en/latest/?badge=latest)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Citations     | [![DOI](https://joss.theoj.org/papers/10.21105/joss.01738/status.svg)](https://doi.org/10.21105/joss.01738) [![DOI](https://zenodo.org/badge/66797360.svg)](https://zenodo.org/badge/latestdoi/66797360)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| License       | [![PyPI License](https://img.shields.io/pypi/l/pybotics.svg)](https://pypi.python.org/pypi/pybotics)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Social        | ![Twitter Follow](https://img.shields.io/twitter/follow/engnadeau?style=social) [![Twitter](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Fgithub.com%2Fnnadeau%2Fpybotics)](https://twitter.com/intent/tweet?text=Wow:&url=https%3A%2F%2Fgithub.com%2Fnnadeau%2Fpybotics)                                                                                                                                                                                                                                                                                                                                                                                                                  |

## Contents

- [Pybotics](#pybotics)
  - [Contents](#contents)
  - [Overview](#overview)
  - [Usage](#usage)
    - [Documentation](#documentation)
    - [Installation](#installation)
    - [Applications & Examples](#applications--examples)
  - [Featured In](#featured-in)
  - [Citing](#citing)
  - [Development and Community Guidelines](#development-and-community-guidelines)
    - [Local Development](#local-development)
    - [Docker Development](#docker-development)
    - [Commits](#commits)
    - [Dependency Management](#dependency-management)
    - [Submit an Issue](#submit-an-issue)
    - [Contributing](#contributing)
    - [Testing](#testing)
    - [GitHub Actions](#github-actions)

## Overview

`Pybotics` is an open-source Python toolbox for robot kinematics and calibration.
It was designed to provide a simple, clear, and concise interface to quickly simulate and evaluate common robot concepts, such as kinematics, dynamics, trajectory generations, and calibration.
The toolbox is specifically designed for use with the [Modified Denavit–Hartenberg parameters convention](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters).

## Usage

### Documentation

Please visit https://pybotics.readthedocs.io/

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
- [Trajectory and Path Planning](examples/trajectory_generation.ipynb)
- [Machine Learning](examples/machine_learning.ipynb)
- [Dynamics](examples/dynamics.ipynb)

## Featured In

- [Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling](https://www.mdpi.com/2218-6581/8/2/33/htm)
- [PyCon Canada 2017](https://2017.pycon.ca/schedule/53/)
  - [Talk Photos](https://500px.com/nicholasnadeau/galleries/pycon-canada-2017)
  - [Slides](https://github.com/nnadeau/pycon-canada-2017)
- [Montreal-Python 2017](https://www.youtube.com/watch?v=wgKoGA69YXQ)

## Citing

Please cite the following articles if you use `pybotics` in your research:

> Nadeau, (2019). Pybotics: Python Toolbox for Robotics. Journal of Open Source Software, 4(41), 1738, https://doi.org/10.21105/joss.01738

```
@article{nadeau2019pybotics,
  doi = {10.21105/joss.01738},
  url = {https://doi.org/10.21105/joss.01738},
  year = {2019},
  month = sep,
  publisher = {The Open Journal},
  volume = {4},
  number = {41},
  pages = {1738},
  author = {Nicholas Nadeau},
  title = {Pybotics: Python Toolbox for Robotics},
  journal = {Journal of Open Source Software}
}
```

> Nadeau, Nicholas A., Ilian A. Bonev, and Ahmed Joubair. "Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling." Robotics 8.2 (2019): 33.

```
@article{nadeau2019impedance,
  title={Impedance Control Self-Calibration of a Collaborative Robot Using Kinematic Coupling},
  volume={8},
  ISSN={2218-6581},
  url={http://dx.doi.org/10.3390/robotics8020033},
  DOI={10.3390/robotics8020033},
  number={2},
  journal={Robotics},
  publisher={MDPI AG},
  author={Nadeau, Nicholas A. and Bonev, Ilian A. and Joubair, Ahmed},
  year={2019},
  month={Apr},
  pages={33}
}
```

## Development and Community Guidelines

### Local Development

- Use [poetry](https://python-poetry.org/) to install the dev virtual environment:

```bash
poetry install
```

### Docker Development

- Docker is a great tool to test the package in an isolated environment
- It is especially useful for debugging issues between python versions

```bash
# launch container attached to current directory
docker run -v $(pwd):/$(basename $(pwd)) -w /$(basename $(pwd)) -it python:3 bash

# install deps
pip install poetry
poetry install

# run tests
make test
```

### Commits

- The repo abides by [SemVer](https://semver.org/), [`semantic-release`](https://github.com/semantic-release/semantic-release), and [Angular commit message syntax](https://github.com/angular/angular/blob/main/CONTRIBUTING.md)
- It is highly recommended to use the [`commitizen` CLI](https://github.com/commitizen/cz-cli)
- See commit examples below:

| Change                    | Commit Type |
| ------------------------- | ----------- |
| Bumped dependency version | `build`     |
| Bumped Python requirement | `feat`      |

### Dependency Management

```bash
# refresh lock file
poetry lock --no-update
```

### Submit an Issue

- Navigate to the repository's [issue tab](https://github.com/nnadeau/pybotics/issues)
- Search for related existing issues
- If necessary, create a new issue using the provided templates

### Contributing

- Please see [`CONTRIBUTING.md`](.github/CONTRIBUTING.md) and the [Code of Conduct](CODE_OF_CONDUCT.md) for how to contribute to the project

### Testing

- Please review the [`Makefile`](Makefile) for an overview of all available tests
- The most important tests and `make` commands are highlighted below:

```bash
# auto-format code
make format

# perform all static tests
make lint

# run all python tests
make test
```

### GitHub Actions

- This repo uses [`semantic-releases`](https://github.com/semantic-release/) to generate releases and release notes automatically from commits
  - A [`PERSONAL_TOKEN` Actions secret](https://github.com/nnadeau/pybotics/settings/secrets/actions) from a [Personal Token](https://github.com/settings/tokens) with a [`public_repo` scope](https://github.com/semantic-release/github#github-authentication) is needed for CI releases

---

Icons made by <a href="https://icon54.com/" title="Pixel perfect">Pixel perfect</a> from <a href="https://www.flaticon.com/" title="Flaticon"> www.flaticon.com</a>
