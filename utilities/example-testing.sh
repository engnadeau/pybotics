#!/usr/bin/env bash

for F in examples/*.ipynb; do jupyter nbconvert --to script --execute --stdout $F | python; done