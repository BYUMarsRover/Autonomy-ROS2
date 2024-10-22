from setuptools import find_packages, setup
from glob import glob

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install launch files
        ('share/' + package_name, glob('/launch*launch.[pxy][yma]*')),

        # Install config files
        ('share/' + package_name + '/config', ['config/estimation.yaml']),

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
            'position_velocity_time_translator = odometry.position_velocity_time_translator:main',
        ],
    },
)
