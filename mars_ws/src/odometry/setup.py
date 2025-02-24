from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'odometry'

# print(os.path.join('share', package_name, 'launch'))
# print(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install all launch files
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),

        # Install config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),

        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marsrover',
    maintainer_email='marsrover@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_state_singleton_creator = odometry.rover_state_singleton_creator:main',
            'rover_state_singleton_creator_new = odometry.rover_state_singleton_creator_new:main',
            'position_velocity_time_translator = odometry.position_velocity_time_translator:main',
            'dummy_singleton_publisher = odometry.dummy_singleton_publisher:main',
            'global_heading_tf_publisher = odometry.global_heading_tf_publisher:main',
        ],
    },
)
