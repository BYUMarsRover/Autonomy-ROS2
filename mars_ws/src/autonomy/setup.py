from setuptools import find_packages, setup
from glob import glob

package_name = 'autonomy'

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
    description='The autonomous state machine package',
    license='BYU YOU CANT HAVE IT"',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiducial_data = autonomy.fiducial_data:main', 
            'drive_controller_api = autonomy.drive_controller_api:main',
            'state_machine = autonomy.state_machine:main',
        ],
    },
)
