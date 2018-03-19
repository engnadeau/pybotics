#!/usr/bin/env bash

set -e
set -x

mypy --strict .
flake8
vulture --exclude=docs,conftest.py,__init__.py .
xenon --max-absolute B --max-modules A --max-average A pybotics
pipdeptree -w fail -p pybotics
bandit -r -v pybotics
pipenv check pybotics
safety check --full-report \
    -r requirements.txt \
    -r dev-requirements.txt
