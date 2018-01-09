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
pipenv check pybotics
safety check --full-report \
    -r requirements/main.txt \
    -r requirements/example-testing.txt \
    -r requirements/unit-testing.txt \
    -r requirements/deployment.txt \
    -r requirements/versioning.txt \
    -r requirements/static-testing.txt
