check-package:
	pipenv run python setup.py check --strict --metadata
	pipenv run check-manifest -v

check-typing:
	pipenv run mypy --strict .

check-format:
	pipenv run black --check .
	pipenv run isort -rc -c .

lint:
	pipenv run flake8
	pipenv run vulture --min-confidence 80 --exclude=docs,build,.eggs --sort-by-size .

check: check-format check-package check-typing lint

format:
	pipenv run black .
	pipenv run isort -rc .

test:
	python setup.py test

clean:
	pipenv run python setup.py clean --all

.PHONY: static check-package check-typing lint check test clean check-format format
