from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'heartbeat'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marsrover',
    maintainer_email='hilton.chloe.y@gmail.com',
    description='Heartbeat',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_rover = heartbeat.HeartbeatRover:main',
        ],
    },
)