import setuptools

setuptools.setup(
    name="ros-model",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="ros model",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'pydantic'],
    entry_points = {
    },
)
