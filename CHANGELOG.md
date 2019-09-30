# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [0.7.3]

### Fixed

- Set poetry version to git tag (https://github.com/nnadeau/pybotics/commit/9f3be62b66c8c1a8fb541f9522f5e9ebe3a74c9c)

## [0.7.2]

### Added

- Added issue templates (7d53e8f31756c316a0c84662a094a644bb376e3a)
- Updated README.md with additional community info, direct links to the relevant docs/issues, and extended the testing how-to (2fe8b2127d560c676666c1a041d903716d3b18f8)
- Updates for JOSS paper

### Fixed

- ReadTheDocs now fully works

## [0.7.1]

### Added

- RTD documentation
- JOSS paper

## [0.7.0]

### Changed

- General code cleanup
- Classes now use attrs to manage boilerplate initialization
- poetry used for package and dependency management

## [0.6.0] - 2018-04-10

### Added

- Jacobian methods
- Basic usage script: `docs/examples/basic_usage.py`
- `pybotics.__version__` built-in using `setuptools_scm`
- `least_squares` IK method
- `Robot.home_joints`
- Various setters/getters
- More angle conventions and conversions
- Various useful `__repr__`
- OSX and Linux CircleCI builds
- Windows builds with AppVeyor
- `.flake8` config
- Repo bots (e.g., `.github/config.yml`, `.github/stale.yml`)

### Changed

- Simplified requirements files
- Simplified `pybotics/__init__.py` structure
- Versions now managed by `setuptools_scm`
- Simplified overall API, see examples for more detail
- Optimization vector/mask only exist in the context of `OptimizationHandler`
- `Robot.random_joints()` can return array or has `inplace` flag
- PyUp now creates PRs as individual packages are updated instead of a daily check
- Pruning/excluding files in `MANIFEST`
- Improved RTD documentation

### Fixed

- Joint limits are checked when setting `Robot.joints`
- Various CI and test improvements
- Typo in `.github/ISSUE_TEMPLATE.md`
- TravisCI `distributions` key in wrong location

## [0.5.0] - 2017-11-18

### Fixed

- [Implicit `GitPython` dependency](https://github.com/nnadeau/pybotics/issues/214)

## [0.4.1] - 2017-11-10

### Fixed

- Error in `setup.py` when installing released package from PyPI
- Fixed `requirements.txt` location in `MANIFEST.in`

### Added

- `sdist` and `bdist` are now tested in CI

## [0.4.0] - 2017-11-08

- A big refactor of `pybotics`
- Many breaking changes to how models are designed and used
- The goal was to clean technical debt and simplify the architecture

### Added

- `pybotics.__version__`
- `CODE_OF_CONDUCT.md`
- CI helper scripts
- `docs`

### Fixed

- New PyPI test server in `.travis.yml`

### Changed

- All modules have been significantly updated
- Split requirements
- Typing now heavily relies on `collections.abc`
- Simplified `.travis.yml` configuration
- Simplified `README.md`
- More static testing
- Simplified `setup.py`
- `100%` test coverage requirement

### Removed

- Inverse kinematics (IK) for the moment
- `README.rst`

## [0.3.2] - 2017-06-20

### CI

- Added `clean_readme.py` for `pandoc` filtering to strip and convert `README.md` to a simpler `README.rst` for PyPI
- Branches are now deployed to [Test PyPI](https://testpypi.python.org/pypi/pybotics/) upon successful build
- `pandocfilters` added to `ci-requirements.txt`
- `update_version.py` now uses Travis CI environment variables to help differentiate between tag and branch builds in order to upload unique versions (conforming to PEP440) to Test PyPI

### Misc

- Changed relative URL of logo in `README` to absolute link
- Added `Development` section

## 0.3.1

### Misc

- Migrated to pure pandoc README conversion
- Added GitHub issue and PR templates

## 0.3.0

### Travis CI

- Switched to sudo-less containers
- Upgraded to `ubuntu:trusty`
- Simplified `pip install` stage with `ci-requirements.txt`
- Added `convert_readme.py` to convert `README.md` to `README.rst` for `PyPI`
- Added strict enforcement of:
    - `mypy` (typing)
    - `flake8` (PEP8 coding style)
    - `vulture` (dead code)
    - `bandit` (security issues)
    - `pydocstyle` (docstrings)
    - `pipdeptree` (dependencies)

### Misc

- Removed `PyPI` downloads from `README` (deprecated)
- Simplified `README`
- Added logo
- Added `pyup` service
- Added `QuantifiedCode` service
- Added `Scrutinizer CI` service
- Discontinued `Python` `3.2` and `3.3` support
- Updated docstring and typing
