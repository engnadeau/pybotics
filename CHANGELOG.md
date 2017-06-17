## 0.3.2
### Misc
- Changed relative URL of logo in `README` to absolute link

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
