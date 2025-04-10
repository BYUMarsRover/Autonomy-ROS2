from setuptools import find_packages, setup
from glob import glob

package_name = 'hazard_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', glob('params/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marsrover',
    maintainer_email='marsrover@todo.todo',
    description='A package for detecting hazards using SLAM outputs.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_detector = hazard_detection.hazard_detector:main',
            'hazard_avoidance_test = hazard_detection.hazard_avoidance_test:main',
        ],
    },
)
