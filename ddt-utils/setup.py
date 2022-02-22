import setuptools

setuptools.setup(
    name="ddt-utils",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="ddt probe",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'pydantic',
        'ros2-model',
        'psutil',],
    entry_points = {
    },
)
