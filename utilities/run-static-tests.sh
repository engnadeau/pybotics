#!/usr/bin/env bash

# quit script if any test fails
set -e

# echo each command
set -x

# check README
python setup.py check --strict --metadata

# type checking
mypy --strict .

# general linting
# settings found in .flake8
flake8

# check dead code
vulture --min-confidence 80 --exclude=docs,build,.eggs --sort-by-size .

# dependency linting
pipdeptree -w fail -p pybotics
bandit -r pybotics
pipenv check pybotics
pipenv graph
