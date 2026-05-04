from setuptools import setup
import os
from glob import glob

package_name = 'forever_explore'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OmniLRS',
    maintainer_email='omar_anwar@aucegypt.edu',
    description='Forever-exploration wrapper around explore_lite: random reachable goals + costmap reset when the room is fully mapped.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'forever_explore = forever_explore.forever_explore_node:main',
        ],
    },
)
