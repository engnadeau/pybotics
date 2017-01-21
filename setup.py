import pypandoc
from setuptools import setup
import os
import git

# tag version
repo = git.Repo(os.getcwd())
tag = repo.tags[-1]
version = tag.name

# requirements
with open('requirements.txt') as f:
    requirements = f.read().splitlines()

# long description
# TODO: pypandoc.convert() function returns wrong RST format, but saving/loading a file works
file_path = os.path.abspath(os.path.dirname(__file__))
pypandoc.convert_file('README.md', 'rst', outputfile=os.path.join(file_path, 'README.rst'))
with open(os.path.join(file_path, 'README.rst'), encoding='utf-8') as f:
    description = f.read()

setup(
    name='pybotics',
    version=version,
    packages=['pybotics'],
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
        'Programming Language :: Python :: 3.2',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    keywords='python robot robotics research automation kinematics geometry',
)
