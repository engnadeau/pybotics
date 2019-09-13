check-package:
	poetry check -v

check-typing:
	poetry run mypy --strict .

check-format:
	poetry run black --check .
	poetry run isort -rc -c .

lint:
	poetry run flake8 pybotics tests examples
	poetry run vulture --min-confidence 80 --sort-by-size pybotics tests examples

check: check-format check-package check-typing lint

format:
	poetry run black .
	poetry run isort -rc .

test:
	poetry run pytest

build:
	poetry build

paper:
	cd paper && pandoc paper.md -o paper.pdf --bibliography=paper.bib

docs:
	poetry run sphinx-build -b html docs docs/_build

docs-api:
	poetry run sphinx-apidoc -o docs --separate pybotics

docs-autobuild:
	poetry run sphinx-autobuild docs docs/_build

.PHONY: static check-package check-typing lint check test check-format format docs docs-autobuild docs-api paper
