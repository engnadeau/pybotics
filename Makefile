.DEFAULT_GOAL := build

.PHONY: check
check: lint

.PHONY: lint
lint:
	poetry check -v
	poetry run mypy --strict --show-error-codes .
	poetry run black --check .
	poetry run isort -c .
	poetry run flake8 pybotics tests examples
	poetry run vulture --min-confidence 80 --sort-by-size pybotics tests examples

.PHONY: debug
debug:
	@echo "Git version: $(shell git describe --tags)"
	poetry debug

.PHONY: format
format:
	poetry run black .
	poetry run isort .

.PHONY: test
test:
	PYTHONPATH=. \
		poetry run pytest \
		--cov=pybotics \
		--cov-report term-missing \
		--cov-config .coveragerc \
		--verbose

.PHONY: test-notebooks
test-notebooks: examples/*.ipynb
	for file in $^; do \
		poetry run jupyter nbconvert --execute $${file}; \
	done

.PHONY: test-examples
test-examples: examples/*.py
	for file in $^; do \
		poetry run python $${file}; \
	done

.PHONY: build
build:
	# bump local package version to match git tag and build package
	poetry version $(shell git describe --tags --abbrev=0)
	poetry build

.PHONY: paper
paper:
	cd paper && \
		pandoc paper.md \
		-o paper.pdf \
		--bibliography=paper.bib

.PHONY: docs
docs: docs-api
	poetry run sphinx-build \
		-b html docs docs/_build

.PHONY: docs-api
docs-api:
	poetry run sphinx-apidoc -o docs --separate pybotics

.PHONY: serve-docs
docs-autobuild:
	poetry run sphinx-autobuild docs docs/_build
