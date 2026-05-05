from setuptools import setup
import os
from glob import glob

package_name = 'stuck_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Stuck detection (wheels spinning, SLAM pose static) with Nav2 lifecycle pause/resume.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stuck_guard = stuck_guard.stuck_guard_node:main',
        ],
    },
)
