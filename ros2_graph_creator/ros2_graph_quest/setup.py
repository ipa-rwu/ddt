import os
from glob import glob
from setuptools import setup

package_name = 'ros2_graph_quest'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruichao',
    maintainer_email='ruichao.wu@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_parser = ros2_graph_quest.parse_graph:main',
            'node_parser = ros2_graph_quest.parse_node:main',
            'lifecycle_node_parser = ros2_graph_quest.parse_lifecycle:main'
        ],
    },
)