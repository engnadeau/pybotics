#!/usr/bin/env bash

mypy --strict pybotics
flake8
pylint pybotics
vulture --exclude=docs,conftest.py,__init__.py .
bandit -r -v pybotics
pipdeptree -w fail -p pybotics