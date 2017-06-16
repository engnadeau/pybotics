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
