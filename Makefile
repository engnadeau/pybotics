check-package:
	poetry run python setup.py check --strict --metadata
	poetry run check-manifest -v

check-typing:
	poetry run mypy --strict .

check-format:
	poetry run black --check .
	poetry run isort -rc -c .

lint:
	poetry run flake8
	poetry run vulture --min-confidence 80 --exclude=docs,build,.eggs --sort-by-size .

check: check-format check-package check-typing lint

format:
	poetry run black .
	poetry run isort -rc .

test:
	poetry run python setup.py test

clean:
	poetry run python setup.py clean --all

.PHONY: static check-package check-typing lint check test clean check-format format
