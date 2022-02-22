import setuptools

setuptools.setup(
    name="ros2-helpers",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="help tools",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'numpy',
        'pyyaml',
        'netifaces',
        'packaging',
        'ros2-model',
        'Click'],
    entry_points = {
        'console_scripts': [
            'ros2helper = ros2_helpers:cli',
        ],
    },
)
