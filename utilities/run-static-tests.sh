#!/usr/bin/env bash

# type checking
echo "mypy"
mypy --strict pybotics

# linters
echo "flake8"
flake8

echo "vulture"
vulture --exclude=docs,conftest.py,__init__.py .

# quality
echo "xenon"
xenon --max-absolute B --max-modules A --max-average A pybotics

# security & dependencies
echo "pipdeptree"
pipdeptree -w fail -p pybotics

echo "bandit"
bandit -r -v pybotics

echo "pipenv check"
pipenv check pybotics

echo "safety check"
safety check --full-report \
    -r requirements/main.txt \
    -r requirements/example-testing.txt \
    -r requirements/unit-testing.txt \
    -r requirements/deployment.txt \
    -r requirements/versioning.txt \
    -r requirements/static-testing.txt
