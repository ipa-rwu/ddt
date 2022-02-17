import setuptools

setuptools.setup(
    name="ddt-probe",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="ddt probe",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'Click',
        'python-socketio[asyncio_client]',
        'ros2-model',
        'ros2-helpers',
        'ddt-utils'],
    entry_points = {
        'console_scripts': [
            'ddt_probe = ddt_probe:cli',
        ],
    },
)
