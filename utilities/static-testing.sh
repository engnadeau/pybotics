#!/usr/bin/env bash

python -V
echo PWD: $PWD

mypy --strict pybotics
flake8 --show-source -v --count --statistics pybotics
vulture --exclude=docs .
bandit -r -v pybotics
pydocstyle -e -s -v pybotics
pipdeptree -w fail -p pybotics