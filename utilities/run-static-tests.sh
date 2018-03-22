#!/usr/bin/env bash

# quit script if any test fails
set -e

# echo each command
set -x

# type checking
mypy --strict .

# general linting
# settings found in .flake8
flake8

# check dead code
vulture --min-confidence 80 --exclude=docs --sort-by-size .

# check complexity check
xenon --max-absolute B --max-modules A --max-average A pybotics

# dependency linting
pipdeptree -w fail -p pybotics
bandit -r pybotics
pipenv check pybotics
