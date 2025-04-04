from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulation_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Ensure __init__.py exists in simulation_pubsub/
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # If you have launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JARVIS',
    maintainer_email='stevenrowe771@gmail.com',
    description='Subscribes to rover data and publishes to simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker_and_listener = simulation_pubsub.subscriber_publisher_function:main'
        ],
    },
)
