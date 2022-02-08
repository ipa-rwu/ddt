import setuptools

setuptools.setup(
    name="ddt-orchestration",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="ddt probe and manager",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'python-socketio[asyncio_client]',
        'flask',
        'flask_sockets',
        'flask-socketio',
        'pygraphviz'
    ],
    entry_points = {
    },
)
