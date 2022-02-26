import setuptools

setuptools.setup(
    name="ddt-manager",
    version="0.0.1",
    author="Ruichao Wu",
    license="Apache License, Version 2.0",
    description="ddt manager",
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    install_requires=[
        'flask',
        'flask-sockets',
        'flask-socketio',
        'pygraphviz',
        'svg_stack',
        'pydantic',
        'svgwrite',
        'ros2-model',
        'ddt-utils',
        'Click',
        # fix svf-stcak
        'six',
        'lxml',
        'pyyaml',
        'kubernetes'],
    entry_points = {
    },
)
