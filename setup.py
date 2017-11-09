"""Setup module."""
from setuptools import setup, find_packages
import git
import os
import logging

REPO_ROOT_PATH = os.path.dirname(__file__)
VERSION = 'dev'
GIT_SHA = ''
GIT_SHA_SHORT = ''
IS_RELEASE = False
TAG = ''
LAST_TAG = '0.0.0'


def write_version_py():
    """
    Write version info to version.py.

    :return:
    """
    content = """
# THIS FILE IS GENERATED FROM SETUP.PY
VERSION = '{version}'
GIT_SHA = '{git_sha}'
GIT_SHA_SHORT = '{git_sha_short}'
IS_RELEASE = {is_release}
"""
    filename = os.path.join(REPO_ROOT_PATH, 'pybotics', 'version.py')
    with open(filename, 'w') as file:
        file.write(content.format(version=VERSION,
                                  git_sha=GIT_SHA,
                                  git_sha_short=GIT_SHA_SHORT,
                                  is_release=IS_RELEASE))


def update_git_info():
    """
    Get the current git info.

    :return:
    """
    repo = git.Repo(REPO_ROOT_PATH)
    logging.info('Repo:\t{}'.format(repo))

    global GIT_SHA
    GIT_SHA = repo.head.object.hexsha
    logging.info('Git sha:\t{}'.format(GIT_SHA))

    global GIT_SHA_SHORT
    GIT_SHA_SHORT = repo.git.rev_parse(GIT_SHA, short=4)
    logging.info('Git short sha:\t{}'.format(GIT_SHA_SHORT))

    if len(repo.tags) > 0:
        global LAST_TAG
        LAST_TAG = repo.tags[-1]
        logging.info('Last tag:\t{}'.format(LAST_TAG))
    else:
        logging.warning('No tags found:\t{}'.format(LAST_TAG))


def check_travis_ci():
    """
    Investigate if the current environment is Travis CI.

    :return:
    """
    travis_commit = os.environ.get('TRAVIS_COMMIT')
    logging.info('Travis commit:\t{}'.format(travis_commit))

    travis_branch = os.environ.get('TRAVIS_BRANCH')
    logging.info('Travis branch:\t{}'.format(travis_branch))

    travis_pr_branch = os.environ.get('TRAVIS_PULL_REQUEST_BRANCH')
    logging.info('Travis PR branch:\t{}'.format(travis_pr_branch))

    travis_tag = os.environ.get('TRAVIS_TAG')
    travis_tag = travis_tag if travis_tag is not None else ''
    logging.info('Travis tag:\t{}'.format(travis_tag))

    if len(travis_tag) > 0:
        global IS_RELEASE
        IS_RELEASE = True

        global TAG
        TAG = '{}'.format(travis_tag)


def update_version():
    """
    Update the version info.

    :return:
    """
    global VERSION
    if IS_RELEASE:
        VERSION = '{}'.format(TAG)
    else:
        VERSION = '{}.dev{}'.format(LAST_TAG, int(GIT_SHA_SHORT, 16))

    logging.info('Package version:\t{}'.format(VERSION))


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    try:
        # pypi package should already have version.py
        from pybotics import version
    except ImportError:
        # in CI and on dev, we don't have version.py yet
        update_git_info()
        check_travis_ci()
        update_version()
        write_version_py()
    else:
        VERSION = version.VERSION

    with open(os.path.join(os.path.dirname(__file__),
                           'requirements', 'requirements.txt')) as f:
        requirements = f.read().splitlines()

    with open('README.md', encoding='utf-8') as f:
        description = f.read()

    setup(
        name='pybotics',
        version=VERSION,
        packages=find_packages(exclude=['*tests*', 'utilities', 'examples']),
        url='https://github.com/nnadeau/pybotics',
        license='MIT',
        author='Nicholas Nadeau',
        author_email='nicholas.nadeau@gmail.com',
        description='Python Toolbox for Robotics',
        long_description=description,
        install_requires=requirements,
        setup_requires=['pytest-runner'],
        tests_require=['pytest'],
        classifiers=[
            'Development Status :: 4 - Beta',
            'Intended Audience :: Developers',
            'Intended Audience :: Education',
            'Intended Audience :: End Users/Desktop',
            'Intended Audience :: Manufacturing',
            'Intended Audience :: Science/Research',
            'Topic :: Education',
            'Topic :: Scientific/Engineering',
            'Topic :: Scientific/Engineering :: Artificial Intelligence',
            'Topic :: Scientific/Engineering :: Human Machine Interfaces',
            'Topic :: Scientific/Engineering :: Mathematics',
            'Topic :: Scientific/Engineering :: Physics',
            'Topic :: Utilities',
            'License :: OSI Approved :: MIT License',
            'Programming Language :: Python :: 3 :: Only',
            'Programming Language :: Python :: 3',
            'Programming Language :: Python :: 3.4',
            'Programming Language :: Python :: 3.5',
            'Programming Language :: Python :: 3.6',
        ],
        keywords='python robot robotics research '
                 'automation kinematics geometry',
    )
