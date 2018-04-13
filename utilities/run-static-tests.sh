#!/usr/bin/env bash

# quit script if any test fails
set -e

# echo each command
set -x

# packaging checks
python setup.py check --strict --metadata
check-manifest -v

# type checking
mypy --strict .

# general linting
# settings found in .flake8
flake8

# check dead code
vulture --min-confidence 80 --exclude=docs,build,.eggs --sort-by-size .

# dependency linting
pip list
pipdeptree -w fail -p pybotics
bandit -r pybotics
pipenv check pybotics
pipenv graph
