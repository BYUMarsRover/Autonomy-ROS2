from setuptools import find_packages, setup
from glob import glob

package_name = 'home_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'base_home_gui = home_gui.base_home_gui:main',
            'rover_camera_control = home_gui.rover_camera_control:main',
            'rover_dev_update = home_gui.rover_dev_update:main',
        ],
    },
)
