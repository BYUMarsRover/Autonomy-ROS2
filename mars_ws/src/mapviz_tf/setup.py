from setuptools import find_packages, setup
import os
import glob

package_name = 'mapviz_tf'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params', glob.glob('params/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Crandall',
    maintainer_email="wyomike2020@gmail.com",
    description='The MapViz package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_waypoint = mapviz_tf.click_waypoint:main',  # Separate each entry with a comma
            'gps_to_mapviz = mapviz_tf.gps_to_mapviz:main',
            'path_to_mapviz = mapviz_tf.path_to_mapviz:main',
            'rover_tf_broadcaster = mapviz_tf.rover_tf_broadcaster:main',
        ],
    },
)
