from setuptools import find_packages, setup
from glob import glob

package_name = 'mobility'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
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
            'transition = mobility.transition:main',
            'drive_manager = mobility.drive_manager:main',
            'aruco_autopilot_manager = mobility.aruco_autopilot_manager:main',
            'autopilot_manager = mobility.autopilot_manager:main',
            'path_manager = mobility.path_manager:main',
            'wheel_manager = mobility.wheel_manager:main',
            'mega_middleman = mobility.mega_middleman:main',
            'joystick = mobility.joystick_control:main',
        ],
    },
)
