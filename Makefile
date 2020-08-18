# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# checks and linting

.PHONY: check-package
check-package:
	poetry check -v

.PHONY: check-typing
check-typing:
	poetry run mypy --strict .

.PHONY: check-format
check-format:
	poetry run black --check .
	poetry run isort -c .

.PHONY: lint
lint:
	poetry run flake8 pybotics tests examples
	poetry run vulture --min-confidence 80 --sort-by-size pybotics tests examples

.PHONY: check
check: check-format check-package check-typing lint

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# formatting

.PHONY: format
format:
	poetry run black .
	poetry run isort .

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# testing

.PHONY: test
test:
	PYTHONPATH=. poetry run pytest --cov=pybotics --cov-report term-missing --cov-config .coveragerc --verbose

.PHONY: test-notebooks
test-notebooks:
	poetry run jupyter nbconvert --execute examples/*.ipynb

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# packaging

.PHONY: build
build:
	poetry build

.PHONY: update-dev-dependencies
update-dev-dependencies:
	poetry run python scripts/update_dev_dependencies.py

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# documentation

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
