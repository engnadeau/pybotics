import os
import logging

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
            version = '0.0.0.dev{}'.format(int(travis_commit, 16))
        else:
            version = '0.0.0.dev'

    version_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'VERSION'
    )
    logging.info('Version path: {}'.format(version_path))

    with open(version_path, 'w') as f:
        f.write(version)
