import os
import logging
from pathlib import Path

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    # fetch travis variables
    travis_commit = os.environ.get('TRAVIS_COMMIT')
    travis_branch = os.environ.get('TRAVIS_BRANCH')
    travis_pr_branch = os.environ.get('TRAVIS_PULL_REQUEST_BRANCH')
    travis_tag = os.environ.get('TRAVIS_TAG')

    # log variables
    logging.info('Travis commit: {}'.format(travis_commit))
    logging.info('Travis branch: {}'.format(travis_branch))
    logging.info('Travis PR branch: {}'.format(travis_pr_branch))
    logging.info('Travis tag: {}'.format(travis_tag))

    # generate version
    if travis_tag:
        version = travis_tag
    else:
        if travis_commit:
            version = '{}'.format(int(travis_commit, 16))
        else:
            version = '0.0.0'
    logging.info('Version: {}'.format(version))

    current_path = Path()
    if 'utilities' in str(current_path):
        current_path = current_path.parent
    logging.info('Current path: {}'.format(current_path.resolve()))

    version_path = current_path / 'pybotics' / 'version.py'
    logging.info('Version path: {}'.format(version_path.resolve()))

    with open(str(version_path), 'w') as f:
        f.write('VERSION = {}'.format(version))
