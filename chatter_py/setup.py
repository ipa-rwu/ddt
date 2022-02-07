from setuptools import setup
import os
from glob import glob

package_name = 'chatter_py'

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
            'talker = chatter_py.talker:main',
            'listener = chatter_py.listener:main',
        ],
    },
)
