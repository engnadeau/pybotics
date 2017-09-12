#!/usr/bin/env bash

pandoc -v
pandoc -s -f markdown_github -t json -o README.json README.md
cat README.json
pandoc -s -f markdown_github -t json README.md | python clean_readme.py | pandoc -s -f json -t rst -o README.rst
cat README.rst
