"""Setup module."""
import logging
from pathlib import Path

from setuptools import find_packages, setup  # type: ignore


def main() -> None:
    """Run setup."""
    # run setup
    setup(
        name='pybotics',
        packages=find_packages(),
        url='https://github.com/nnadeau/pybotics',
        license='MIT',
        author='Nicholas Nadeau',
        author_email='nicholas.nadeau@gmail.com',
        description='Python Toolbox for Robotics',
        long_description=get_long_description(),
        long_description_content_type='text/markdown',
        use_scm_version=True,
        setup_requires=[
            'setuptools',
            'setuptools_scm'
        ],
        install_requires=get_requirements(),  # type: ignore
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
        keywords=[
            'python',
            'robot',
            'robotics',
            'research',
            'automation',
            'kinematics',
            'geometry',
        ]
    )


def get_long_description() -> str:
    """Get description text."""
    # description init
    description = ''

    # add README
    path = Path(__file__).parent / 'README.md'
    logging.info('README path: {}'.format(path.resolve()))
    with open(str(path)) as f:
        description += '\n'
        description += f.read()

    # add changelog
    path = Path(__file__).parent / 'CHANGELOG.md'
    logging.info('CHANGELOG path: {}'.format(path.resolve()))
    with open(str(path)) as f:
        description += '\n'
        description += f.read()

    return description


# don't want to import typing... so ignore type
def get_requirements():  # type: ignore
    """Get requirements list."""
    # requirements
    requirements_path = Path(__file__).parent / 'requirements.txt'
    logging.info('Requirements path: {}'.format(requirements_path.resolve()))
    with open(str(requirements_path)) as f:
        requirements = f.read().splitlines()
    return requirements


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
