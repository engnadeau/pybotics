# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]
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
