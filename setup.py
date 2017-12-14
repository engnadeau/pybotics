"""Setup module."""
from pathlib import Path
from setuptools import setup, find_packages
import logging
from pybotics import __version__

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    # version
    logging.info('Version: {}'.format(__version__))

    # paths
    current_dir = Path(__file__).parent
    logging.info('Current dir: {}'.format(current_dir))

    # requirements
    path = current_dir / 'requirements' / 'requirements.txt'
    logging.info('Requirements path: {}'.format(path))
    with open(str(path)) as f:
        requirements = f.read().splitlines()
    logging.info('Requirements: {}'.format(requirements))

    # description
    path = current_dir / 'README.md'
    logging.info('Requirements path: {}'.format(path))
    with open(path, encoding='utf-8') as f:
        description = f.read()

    setup(
        name='pybotics',
        version=__version__,
        packages=find_packages(
            exclude=[
                '*tests*',
                'utilities',
                'examples'
            ]),
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
