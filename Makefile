.PHONY: check-package
check-package:
	poetry check -v

.PHONY: check-typing
check-typing:
	poetry run mypy --strict .

.PHONY: check-format
check-format:
	poetry run black --check .
	poetry run isort -rc -c .

.PHONY: lint
lint:
	poetry run flake8 pybotics tests examples
	poetry run vulture --min-confidence 80 --sort-by-size pybotics tests examples

.PHONY: check
check: check-format check-package check-typing lint

.PHONY: format
format:
	poetry run black .
	poetry run isort -rc .

.PHONY: test
test:
	poetry run pytest

.PHONY: build
build:
	poetry build

.PHONY: paper
paper:
	cd paper && pandoc paper.md -o paper.pdf --bibliography=paper.bib

.PHONY: docs
docs:
	poetry run sphinx-build -b html docs docs/_build

.PHONY: docs-api
docs-api:
	poetry run sphinx-apidoc -o docs --separate pybotics

.PHONY: docs-autobuild
docs-autobuild:
	poetry run sphinx-autobuild docs docs/_build
