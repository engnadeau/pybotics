#!/usr/bin/env bash

# quit script if any test fails
set -e

# echo each command
set -x

# install
gem install github_changelog_generator

# run
github_changelog_generator

# review
cat CHANGELOG.md