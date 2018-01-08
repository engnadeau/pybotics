from pathlib import Path
from setuptools_scm import get_version
import logging


def main():
    # get version
    version = get_version(root='..', relative_to=__file__)
    version = str(version).replace('+', '')
    logging.info('Version: {}'.format(version))

    # get root path
    root_path = Path(__file__).parents[1]
    logging.info('Root path: {}'.format(root_path.resolve()))

    # dump version
    version_path = root_path / 'VERSION'
    logging.info('Dumping version to: {}'.format(version_path.resolve()))

    with open(str(version_path), 'w') as f:
        f.write(version)
    logging.info('Version dumped')


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
