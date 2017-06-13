from setuptools import setup

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

with open('README.rst', encoding='utf-8') as f:
    description = f.read()

with open('VERSION', encoding='utf-8') as f:
    version = f.read()

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
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    keywords='python robot robotics research automation kinematics geometry',
)
