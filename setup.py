from distutils.core import setup

setup(
    name='pybotics',
    version='0.1.0',
    packages=['examples', 'pybotics', 'tests'],
    url='https://github.com/nnadeau/pybotics',
    license='MIT',
    author='Nicholas Nadeau',
    author_email='',
    description='Python Toolbox for Robotics',
    setup_requires=['pytest-runner'],
    tests_require=['pytest']
)
