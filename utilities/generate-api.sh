#!/usr/bin/env bash

# echo each command
set -x

# build pybotics api
sphinx-apidoc pybotics/ -o docs/pybotics/ --separate

# build examples apidoc
sphinx-apidoc docs/examples/ -o docs/examples/ --separate
