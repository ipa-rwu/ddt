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
        'flask-sockets',
        'flask-socketio',
        'pygraphviz',
        'svg_stack',
        'pydantic',
        'svgwrite',
        'ros2-model',
        # fix svf-stcak
        'six',
        'lxml'],
    entry_points = {
    },
)
