#!/usr/bin/env bash

# type checking
mypy --strict pybotics

# linters
flake8
vulture --exclude=docs,conftest.py,__init__.py .

# quality
xenon --max-absolute B --max-modules A --max-average A pybotics

# security & dependencies
pipdeptree -w fail -p pybotics
bandit -r -v pybotics
safety check --full-report \
    -r requirements/coverage.txt \
    -r requirements/examples.txt \
    -r requirements/packaging.txt \
    -r requirements/requirements.txt \
    -r requirements/static-testing.txt
