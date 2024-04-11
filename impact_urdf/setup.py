from setuptools import find_packages, setup
setup(
    name='impact_urdf',
    packages=find_packages(include=['impact_urdf']),
    version='0.0.0',
    description='utility to build urdf robot descriptions',
    author='Brent Christensen',
    license='MIT',
    install_requires=[],
    setup_requires=['pytest-runner'],
    tests_require=['pytest==4.4.1'],
    test_suite='tests'
)