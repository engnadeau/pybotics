"""Setup module."""
from setuptools import setup, find_packages
import logging
from pathlib import Path


def main():
    # paths
    root_path = Path(__file__).parent
    logging.info('Root path: {}'.format(root_path))

    # version
    version_path = root_path / 'VERSION'
    logging.info('Version path: {}'.format(version_path))

    with open(str(version_path)) as f:
        version = f.read()
    logging.info('Version: {}'.format(version))

    # requirements
    requirements_path = root_path / 'requirements.txt'
    logging.info('Requirements path: {}'.format(requirements_path))

    with open(str(requirements_path)) as f:
        requirements = f.read().splitlines()
    for i, req in enumerate(requirements):
        requirements[i] = req.split()[0]
    logging.info('Requirements: {}'.format(requirements))

    # description
    description = '`Please visit the GitHub repository for a full ' \
                  'description. <https://github.com/nnadeau/pybotics>`_ '

    # run setup
    setup(name='pybotics',
          version=version,
          packages=find_packages(exclude=['*tests*', 'utilities', 'examples']),
          url='https://github.com/nnadeau/pybotics',
          license='MIT',
          author='Nicholas Nadeau',
          author_email='nicholas.nadeau@gmail.com',
          description='Python Toolbox for Robotics',
          long_description=description,
          install_requires=requirements,
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
                   'automation kinematics geometry')


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
