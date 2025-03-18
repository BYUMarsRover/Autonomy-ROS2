from setuptools import find_packages, setup
from glob import glob

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install all launch files
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),

        # Install all data files
        ('share/' + package_name + '/data', glob('data/*')),

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

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
            'path_planner = path_planning.path_planner:main',
        ],
    },
)
